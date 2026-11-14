#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
카트 맞춤형 D500 (STL-19P) LiDAR ROS 2 드라이버 노드 (v2.1)
=============================================================================
* CP2102 전용 포트 감지 (IMU/ESP32 충돌 방지)
* 발행 토픽: /scan (sensor_msgs/msg/LaserScan)
* 내장 TF 발행:
    - odom -> base_footprint (라이다 단독 매핑 지원용)
    - base_footprint -> base_link
    - base_link -> laser_frame (실측 위치: x=+0.035m, y=+0.005m, z=0.855m)
* 실측 카트 비대칭 마스킹 (전방 17cm, 후방 15cm, 좌 25.5cm, 우 24.5cm)
* 450빈 고해상도(0.8°), 최대 12.0m 개방, CRC-8 검증
=============================================================================
"""

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import serial
import serial.tools.list_ports


def _init_crc_table():
    poly = 0x4D
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ poly) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        table.append(crc)
    return table

CRC_TABLE = _init_crc_table()

def cal_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[crc ^ b]
    return crc


def build_body_mask_table(front_m=0.170, back_m=0.150, left_m=0.255, right_m=0.245, margin_m=0.03):
    table = []
    for i in range(360):
        rad = math.radians(i)
        cos_a = math.cos(rad)
        sin_a = math.sin(rad)
        candidates = []
        if cos_a > 1e-9:
            t = front_m / cos_a
            y = t * sin_a
            if -left_m <= y <= right_m:
                candidates.append(t)
        if cos_a < -1e-9:
            t = -back_m / cos_a
            y = t * sin_a
            if -left_m <= y <= right_m:
                candidates.append(t)
        if sin_a > 1e-9:
            t = right_m / sin_a
            x = t * cos_a
            if -back_m <= x <= front_m:
                candidates.append(t)
        if sin_a < -1e-9:
            t = -left_m / sin_a
            x = t * cos_a
            if -back_m <= x <= front_m:
                candidates.append(t)
        table.append(min(candidates) + margin_m if candidates else margin_m)
    return table


def find_lidar_port():
    if os.path.exists("/dev/ttyCartLiDAR"):
        return "/dev/ttyCartLiDAR"
    ports = list(serial.tools.list_ports.comports())
    # CP210x 칩셋 우선 탐색 (D500 라이다 어댑터)
    for p in ports:
        if "CP210" in (p.description or "") or "10C4:EA60" in (p.hwid or ""):
            return p.device
    for p in ports:
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    return "/dev/ttyUSB0"


class CartLidarNode(Node):
    def __init__(self):
        super().__init__("cart_lidar_node")

        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 230400)
        self.declare_parameter("frame_id", "laser_frame")
        self.declare_parameter("topic_name", "/scan")
        self.declare_parameter("angle_offset_deg", 0.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("num_bins", 450)
        self.declare_parameter("publish_odom_tf", True)

        self.port_name = self.get_parameter("port").get_parameter_value().string_value
        if not self.port_name:
            self.port_name = find_lidar_port()

        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.angle_offset = self.get_parameter("angle_offset_deg").get_parameter_value().double_value
        self.range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self.range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self.num_bins = self.get_parameter("num_bins").get_parameter_value().integer_value
        self.publish_odom_tf = self.get_parameter("publish_odom_tf").get_parameter_value().bool_value

        self.deg_per_bin = 360.0 / self.num_bins
        self.body_mask = build_body_mask_table()

        self.scan_pub = self.create_publisher(LaserScan, self.topic_name, qos_profile_sensor_data)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf = TransformBroadcaster(self)

        self._publish_static_transforms()

        self.scan_ranges = [float("inf")] * self.num_bins
        self.scan_intensities = [0.0] * self.num_bins
        self.last_start_angle = 0.0
        self.scan_start_time = self.get_clock().now()

        self.ser = None
        self.is_running = True
        self.packet_count = 0
        self.scan_count = 0

        self.thread = threading.Thread(target=self.serial_loop, daemon=True)
        self.thread.start()

        # odom -> base_footprint 주기적 브로드캐스트 (SLAM 필수)
        if self.publish_odom_tf:
            self.create_timer(0.05, self._publish_odom_tf_callback)

        self.create_timer(2.0, self.heartbeat_callback)

        self.get_logger().info("=" * 60)
        self.get_logger().info("📡 [카트 맞춤형 D500 LiDAR ROS 2 드라이버 가동]")
        self.get_logger().info(f"📍 연결 포트: {self.port_name} | 속도: {self.baudrate} bps")
        self.get_logger().info(f"📤 토픽: {self.topic_name} | 프레임: {self.frame_id}")
        self.get_logger().info(f"📐 해상도: {self.num_bins} bins (0.8°) | 최대거리: {self.range_max}m")
        self.get_logger().info("=" * 60)

    def _publish_static_transforms(self):
        now = self.get_clock().now().to_msg()
        transforms = []

        # 1. base_footprint -> base_link
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = "base_footprint"
        t1.child_frame_id = "base_link"
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.04
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # 2. base_link -> laser_frame
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = "base_link"
        t2.child_frame_id = self.frame_id
        t2.transform.translation.x = 0.035
        t2.transform.translation.y = 0.005
        t2.transform.translation.z = 0.855
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        self.tf_broadcaster.sendTransform(transforms)

    def _publish_odom_tf_callback(self):
        """odom -> base_footprint TF (엔코더가 없는 상태에서도 SLAM 구동 가능)"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.dynamic_tf.sendTransform(t)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"✅ LiDAR 포트 연결 성공: {self.port_name}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ LiDAR 포트 연결 실패 ({self.port_name}): {e}")
            return False

    def serial_loop(self):
        rx_buf = bytearray()
        while self.is_running and rclpy.ok():
            if not self.ser or not self.ser.is_open:
                if not self.connect_serial():
                    time.sleep(1.5)
                    continue

            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    rx_buf.extend(self.ser.read(waiting))
                else:
                    rx_buf.extend(self.ser.read(47))

                while len(rx_buf) >= 47:
                    idx = -1
                    for i in range(len(rx_buf) - 1):
                        if rx_buf[i] == 0x54 and rx_buf[i + 1] == 0x2C:
                            idx = i
                            break
                    if idx < 0:
                        rx_buf = rx_buf[-1:]
                        break
                    if idx > 0:
                        del rx_buf[:idx]
                    if len(rx_buf) < 47:
                        break

                    packet = bytes(rx_buf[:47])
                    del rx_buf[:47]

                    if cal_crc8(packet[:46]) != packet[46]:
                        continue

                    self.packet_count += 1
                    self.parse_packet(packet)

            except Exception:
                if self.ser:
                    try: self.ser.close()
                    except: pass
                self.ser = None
                time.sleep(1.0)

    def parse_packet(self, packet: bytes):
        start_angle = (((packet[5] << 8) | packet[4]) / 100.0 + self.angle_offset) % 360.0
        end_angle = (((packet[43] << 8) | packet[42]) / 100.0 + self.angle_offset) % 360.0
        if end_angle < start_angle:
            end_angle += 360.0
        step = (end_angle - start_angle) / 11.0

        if start_angle < self.last_start_angle and (self.last_start_angle - start_angle) > 180.0:
            self.publish_scan()

        self.last_start_angle = start_angle

        for i in range(12):
            offset = 6 + i * 3
            dist_mm = (packet[offset + 1] << 8) | packet[offset]
            conf = packet[offset + 2]

            if conf < 15 or dist_mm < 30:
                continue

            angle_deg = (start_angle + step * i) % 360.0
            dist_m = dist_mm / 1000.0

            deg_int = int(round(angle_deg)) % 360
            if dist_m < self.body_mask[deg_int]:
                continue

            if 150 <= deg_int <= 210 and dist_m < 0.60:
                continue

            if dist_m > self.range_max:
                continue

            bin_idx = int(round(angle_deg / self.deg_per_bin)) % self.num_bins
            self.scan_ranges[bin_idx] = dist_m
            self.scan_intensities[bin_idx] = float(conf)

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.scan_start_time.to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = (2.0 * math.pi) / self.num_bins
        msg.time_increment = 0.1 / self.num_bins
        msg.scan_time = 0.1
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = list(self.scan_ranges)
        msg.intensities = list(self.scan_intensities)

        self.scan_pub.publish(msg)
        self.scan_count += 1

        self.scan_ranges = [float("inf")] * self.num_bins
        self.scan_intensities = [0.0] * self.num_bins
        self.scan_start_time = self.get_clock().now()

    def heartbeat_callback(self):
        if self.scan_count > 0:
            fps = self.scan_count / 2.0
            self.get_logger().info(f"🛰️ [LiDAR 정상 발행 중] 속도: {fps:.1f} Hz | 패킷: {self.packet_count}개")
            self.scan_count = 0
            self.packet_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CartLidarNode()
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
