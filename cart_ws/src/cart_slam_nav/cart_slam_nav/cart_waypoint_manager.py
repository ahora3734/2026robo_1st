#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
카트 목적지(웨이포인트) 저장 및 자율주행 호출 매니저 노드
=============================================================================
* 기능:
  1. 지도(map) 위에서 현재 위치를 커스텀 이름('1학년 1반', '화장실')으로 저장
  2. "1학년 1반으로 가" 요청 시 Nav2 NavigateToPose Action을 호출해 자동 주행
  3. JSON 파일(/home/pogo/2026_ROBO_1/maps/waypoints.json) 영구 저장/로드
  4. 토픽(/cart/goto_waypoint) 및 터미널 대화형 CLI 동시 지원
=============================================================================
"""

import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros


WAYPOINTS_FILE = "/home/pogo/2026_ROBO_1/maps/waypoints.json"


def yaw_to_quaternion(yaw_rad):
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    return (0.0, 0.0, sy, cy)


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CartWaypointManager(Node):
    def __init__(self):
        super().__init__("cart_waypoint_manager")

        self.waypoints_file = WAYPOINTS_FILE
        self.waypoints = self._load_waypoints()

        # TF 리스너 (현재 위치 추적)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Nav2 NavigateToPose 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # 토픽 인터페이스: "1학년 1반" 문자열을 받으면 바로 주행
        self.sub_goto = self.create_subscription(
            String, "/cart/goto_waypoint", self._callback_topic_goto, 10
        )
        self.pub_status = self.create_publisher(String, "/cart/nav_status", 10)

        self.current_goal_handle = None
        self.is_navigating = False

        self.get_logger().info("=" * 60)
        self.get_logger().info("📍 [카트 웨이포인트 매니저 가동]")
        self.get_logger().info(f"📁 저장 파일: {self.waypoints_file}")
        self.get_logger().info(f"📋 등록된 위치: {list(self.waypoints.keys())}")
        self.get_logger().info("=" * 60)

    def _load_waypoints(self):
        if os.path.exists(self.waypoints_file):
            try:
                with open(self.waypoints_file, "r", encoding="utf-8") as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().warn(f"파일 로드 실패: {e}")
        return {}

    def _save_waypoints(self):
        os.makedirs(os.path.dirname(self.waypoints_file), exist_ok=True)
        with open(self.waypoints_file, "w", encoding="utf-8") as f:
            json.dump(self.waypoints, f, indent=4, ensure_ascii=False)
        self.get_logger().info("💾 웨이포인트 파일 저장 완료")

    def save_current_pose(self, name: str):
        """현재 카트 위치를 map 좌표계 기준으로 저장"""
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            yaw = quaternion_to_yaw(
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            )

            self.waypoints[name] = {
                "x": round(x, 3),
                "y": round(y, 3),
                "yaw_deg": round(math.degrees(yaw), 1),
            }
            self._save_waypoints()
            print(f"✅ 위치 저장 성공! [{name}]: x={x:.2f}m, y={y:.2f}m, yaw={math.degrees(yaw):.1f}°")
            return True
        except Exception as e:
            print(f"❌ 현재 위치 조회 실패 (SLAM이나 AMCL이 켜져 있나요?): {e}")
            return False

    def goto_waypoint(self, name: str):
        """지정된 이름의 웨이포인트로 자율주행 명령 송신"""
        if name not in self.waypoints:
            msg = f"❌ '{name}' 등록된 위치를 찾을 수 없습니다! 목록: {list(self.waypoints.keys())}"
            print(msg)
            self._pub_status(msg)
            return False

        wp = self.waypoints[name]
        print(f"🚀 [{name}] (x={wp['x']}, y={wp['y']}) (으)로 이동을 시작합니다...")
        self._pub_status(f"이동 중: {name}")

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            print("⚠️ Nav2 내비게이션 서버(navigate_to_pose)가 아직 준비되지 않았습니다.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = float(wp["x"])
        goal_msg.pose.pose.position.y = float(wp["y"])

        qx, qy, qz, qw = yaw_to_quaternion(math.radians(wp.get("yaw_deg", 0.0)))
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.is_navigating = True
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
        return True

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("❌ Nav2 서버가 목표 주행 명령을 거절했습니다.")
            self.is_navigating = False
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.is_navigating = False
        print("\n🎉 목표 지점에 무사히 도착했습니다!")
        self._pub_status("목적지 도착 완료")

    def _feedback_callback(self, feedback_msg):
        # 주기적인 남은 거리 출력
        feedback = feedback_msg.feedback
        rem = feedback.distance_remaining
        print(f"\r🚗 이동 중... 남은 거리: {rem:.2f} m   ", end="", flush=True)

    def _callback_topic_goto(self, msg: String):
        name = msg.data.strip()
        self.goto_waypoint(name)

    def _pub_status(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def cancel_navigation(self):
        if self.current_goal_handle and self.is_navigating:
            self.current_goal_handle.cancel_goal_async()
            self.is_navigating = False
            print("🛑 주행이 취소되었습니다.")


def run_interactive_cli(node: CartWaypointManager):
    """터미널 대화형 메뉴"""
    time.sleep(1.0)
    while rclpy.ok():
        print("\n" + "=" * 50)
        print("  📍 카트 위치 관리 & 자율주행 제어기")
        print("=" * 50)
        print("  1. 등록된 위치 목록 보기 (list)")
        print("  2. 현재 위치를 새 목적지로 저장 (save)")
        print("  3. 저장된 목적지로 자율주행 (goto)")
        print("  4. 진행 중인 주행 즉시 정지 (cancel)")
        print("  5. 목적지 삭제 (delete)")
        print("  0. 종료")
        print("=" * 50)

        cmd = input("👉 선택하세요 [1-5]: ").strip()

        if cmd == "1":
            print("\n📋 [등록된 목적지 목록]")
            if not node.waypoints:
                print("   (아직 등록된 위치가 없습니다.)")
            for k, v in node.waypoints.items():
                print(f"   • {k:15s} : x={v['x']:6.2f}m, y={v['y']:6.2f}m, yaw={v.get('yaw_deg', 0):5.1f}°")

        elif cmd == "2":
            name = input("   저장할 목적지 이름을 입력하세요 (예: 1학년 1반): ").strip()
            if name:
                node.save_current_pose(name)

        elif cmd == "3":
            print(f"   현재 등록된 목록: {list(node.waypoints.keys())}")
            name = input("   어디로 이동할까요? 이름 입력: ").strip()
            if name:
                node.goto_waypoint(name)

        elif cmd == "4":
            node.cancel_navigation()

        elif cmd == "5":
            name = input("   삭제할 목적지 이름 입력: ").strip()
            if name in node.waypoints:
                del node.waypoints[name]
                node._save_waypoints()
                print(f"🗑️ '{name}' 삭제 완료!")

        elif cmd == "0":
            break


def main(args=None):
    rclpy.init(args=args)
    node = CartWaypointManager()

    # CLI는 별도 스레드에서 실행
    cli_thread = threading.Thread(target=run_interactive_cli, args=(node,), daemon=True)
    cli_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

