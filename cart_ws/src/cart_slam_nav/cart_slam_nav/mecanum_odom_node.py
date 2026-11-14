#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
4륜 메카넘 휠 오도메트리 노드 (ESP32 엔코더 피드백 기반)
=============================================================================
* 입력: ESP32 시리얼 패킷 ($ODOM,enc_fl,enc_fr,enc_rl,enc_rr,battery_volt)
* 메카넘 기구학:
    - 바퀴 반지름 R = 0.04m (40mm)
    - 중심에서 전후 바퀴 거리 Lx = 0.10m (총 20cm)
    - 중심에서 좌우 바퀴 거리 Ly = 0.09m (총 18cm)
* 출력: /wheel/odom (nav_msgs/msg/Odometry) -> robot_localization(EKF) 입력용
=============================================================================
"""

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import serial.tools.list_ports


def yaw_to_quaternion(yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = sy
    q.w = cy
    return q


class MecanumOdomNode(Node):
    def __init__(self):
        super().__init__("mecanum_odom_node")

        self.declare_parameter("port", "/dev/ttyCartESP32")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius", 0.04)     # 4cm
        self.declare_parameter("wheel_base_x", 0.10)     # Lx = 10cm
        self.declare_parameter("wheel_base_y", 0.09)     # Ly = 9cm
        self.declare_parameter("encoder_cpr", 1960.0)    # 모터 감속비 1/49 x 40 PPR = ~1960 ticks/rev

        self.port_name = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.R = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.Lx = self.get_parameter("wheel_base_x").get_parameter_value().double_value
        self.Ly = self.get_parameter("wheel_base_y").get_parameter_value().double_value
        self.cpr = self.get_parameter("encoder_cpr").get_parameter_value().double_value

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 20)

        # 포트 탐색
        if not os.path.exists(self.port_name):
            for candidate in ["/dev/ttyUSB2", "/dev/ttyUSB3", "/dev/ttyACM0"]:
                if os.path.exists(candidate):
                    self.port_name = candidate
                    break

        self.ser = None
        self.is_running = True

        # 상태 추정치
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = time.time()
        self.last_ticks = None

        self.thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

        self.get_logger().info(f"🏎️ [메카넘 오도메트리 노드 가동] 포트: {self.port_name}")

    def serial_loop(self):
        while self.is_running and rclpy.ok():
            if not self.ser or not self.ser.is_open:
                try:
                    self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
                    self.get_logger().info(f"✅ ESP32 연결 성공: {self.port_name}")
                except Exception:
                    time.sleep(2.0)
                    continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line.startswith("$ODOM"):
                    continue

                parts = line.split(',')
                if len(parts) >= 6:
                    t_fl = float(parts[1])
                    t_fr = float(parts[2])
                    t_rl = float(parts[3])
                    t_rr = float(parts[4])
                    now = time.time()
                    dt = now - self.last_time

                    if self.last_ticks is not None and dt > 0.001:
                        # 틱 변화량 -> 각 바퀴 회전 각속도 (rad/s)
                        d_fl = (t_fl - self.last_ticks[0]) / self.cpr * 2.0 * math.pi
                        d_fr = (t_fr - self.last_ticks[1]) / self.cpr * 2.0 * math.pi
                        d_rl = (t_rl - self.last_ticks[2]) / self.cpr * 2.0 * math.pi
                        d_rr = (t_rr - self.last_ticks[3]) / self.cpr * 2.0 * math.pi

                        w_fl = d_fl / dt
                        w_fr = d_fr / dt
                        w_rl = d_rl / dt
                        w_rr = d_rr / dt

                        # 메카넘 정기구학 (Forward Kinematics)
                        vx = (self.R / 4.0) * (w_fl + w_fr + w_rl + w_rr)
                        vy = (self.R / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
                        wz = (self.R / (4.0 * (self.Lx + self.Ly))) * (-w_fl + w_fr - w_rl + w_rr)

                        # 전역 좌표 적분
                        delta_x = (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt
                        delta_y = (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt
                        self.x += delta_x
                        self.y += delta_y
                        self.th += wz * dt

                        self.publish_odom(vx, vy, wz)

                    self.last_ticks = (t_fl, t_fr, t_rl, t_rr)
                    self.last_time = now

            except Exception:
                if self.ser:
                    try: self.ser.close()
                    except: pass
                self.ser = None
                time.sleep(1.0)

    def publish_odom(self, vx, vy, wz):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_footprint"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self.th)

        # 공분산 행렬 (메카넘 휠 슬립 고려)
        msg.pose.covariance = [
            0.05, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.05, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.1, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.1, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.1, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.1
        ]

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.twist.covariance = [
            0.02, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.02, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.1, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.1, 0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.1, 0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.05
        ]

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

