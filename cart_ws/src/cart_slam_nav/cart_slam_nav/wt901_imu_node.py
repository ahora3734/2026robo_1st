#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
WT901 9축 IMU ROS 2 드라이버 노드 (v2.1)
=============================================================================
* CP210x(라이다) 충돌 방지: CH340 칩셋만 탐색
* 미연결 시 무한 재연결 루프 방지
=============================================================================
"""

import math
import os
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import serial
import serial.tools.list_ports


def euler_to_quaternion(roll_rad, pitch_rad, yaw_rad):
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def find_imu_port():
    if os.path.exists("/dev/ttyCartIMU"):
        return "/dev/ttyCartIMU"
    ports = list(serial.tools.list_ports.comports())
    # CH340 / QinHeng 칩셋만 탐색 (CP2102 라이다 침범 방지)
    for p in ports:
        if "1A86" in (p.hwid or "").upper() or "CH340" in (p.description or "").upper():
            return p.device
    return None


class WT901DriverNode(Node):
    def __init__(self):
        super().__init__("wt901_driver_node")

        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("topic_name", "/imu/data")

        self.port_name = self.get_parameter("port").get_parameter_value().string_value
        if not self.port_name:
            self.port_name = find_imu_port()

        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value

        self.imu_pub = self.create_publisher(Imu, self.topic_name, 20)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.ser = None
        self.is_running = True

        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.roll = self.pitch = self.yaw = 0.0

        if self.port_name:
            self.thread = threading.Thread(target=self.serial_loop, daemon=True)
            self.thread.start()
            self.get_logger().info(f"🧭 [WT901 IMU 가동] 포트: {self.port_name}")
        else:
            self.get_logger().warn("⚠️ [WT901 IMU] 연결된 IMU 포트를 찾지 못했습니다. (라이다 단독 매핑 진행)")

    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = self.frame_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.40
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def connect_serial(self):
        if not self.port_name:
            return False
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            self.ser.reset_input_buffer()
            return True
        except Exception:
            return False

    def serial_loop(self):
        while self.is_running and rclpy.ok():
            if not self.ser or not self.ser.is_open:
                if not self.connect_serial():
                    time.sleep(3.0)
                    continue

            try:
                b = self.ser.read(1)
                if len(b) == 0 or b[0] != 0x55:
                    continue

                flag = self.ser.read(1)
                if len(flag) == 0:
                    continue

                if flag[0] == 0x51:
                    data = self.ser.read(9)
                    if len(data) == 9:
                        raw_ax = struct.unpack('<h', data[0:2])[0]
                        raw_ay = struct.unpack('<h', data[2:4])[0]
                        raw_az = struct.unpack('<h', data[4:6])[0]
                        self.ax = (raw_ax / 32768.0) * 16.0 * 9.80665
                        self.ay = (raw_ay / 32768.0) * 16.0 * 9.80665
                        self.az = (raw_az / 32768.0) * 16.0 * 9.80665

                elif flag[0] == 0x52:
                    data = self.ser.read(9)
                    if len(data) == 9:
                        raw_gx = struct.unpack('<h', data[0:2])[0]
                        raw_gy = struct.unpack('<h', data[2:4])[0]
                        raw_gz = struct.unpack('<h', data[4:6])[0]
                        self.gx = math.radians((raw_gx / 32768.0) * 2000.0)
                        self.gy = math.radians((raw_gy / 32768.0) * 2000.0)
                        self.gz = math.radians((raw_gz / 32768.0) * 2000.0)

                elif flag[0] == 0x53:
                    data = self.ser.read(9)
                    if len(data) == 9:
                        raw_r = struct.unpack('<h', data[0:2])[0]
                        raw_p = struct.unpack('<h', data[2:4])[0]
                        raw_y = struct.unpack('<h', data[4:6])[0]
                        self.roll = math.radians((raw_r / 32768.0) * 180.0)
                        self.pitch = math.radians((raw_p / 32768.0) * 180.0)
                        self.yaw = math.radians((raw_y / 32768.0) * 180.0)
                        self.publish_imu_msg()

            except Exception:
                if self.ser:
                    try: self.ser.close()
                    except: pass
                self.ser = None
                time.sleep(2.0)

    def publish_imu_msg(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        qx, qy, qz, qw = euler_to_quaternion(self.roll, self.pitch, self.yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = self.gx
        msg.angular_velocity.y = self.gy
        msg.angular_velocity.z = self.gz
        msg.linear_acceleration.x = self.ax
        msg.linear_acceleration.y = self.ay
        msg.linear_acceleration.z = self.az
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WT901DriverNode()
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
