#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
4륜 메카넘 카트 2D SLAM 실시간 매핑 통합 Launch 파일
=============================================================================
실행 구성:
  1. D500 LiDAR 드라이버 노드 (실측 치수 TF 포함)
  2. slam_toolbox (실시간 지도 작성)
  3. 실시간 웹 뷰어 노드 (0.0.0.0:8765)
  (선택: rviz:=true 로 RViz2 실행 가능, 기본값은 안정성을 위해 false)
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = "/home/pogo/2026_ROBO_1/cart_ws/src/cart_slam_nav"
    config_dir = os.path.join(pkg_dir, "config")
    tools_dir = "/home/pogo/2026_ROBO_1/tools"

    slam_config = os.path.join(config_dir, "slam_toolbox.yaml")
    rviz_config = os.path.join("/home/pogo/2026_ROBO_1/config", "cart_slam.rviz")

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="RViz2 실행 여부 (젯슨 환경 크래시 방지를 위해 기본값 false)"
    )

    # 1. 라이다 드라이버 노드 (python 독립 프로세스 실행)
    lidar_node = ExecuteProcess(
        cmd=["python3", os.path.join(tools_dir, "cart_lidar_node.py")],
        output="screen"
    )

    # 2. slam_toolbox 온라인 동기 매핑 노드
    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_config]
    )

    # 3. 실시간 웹 뷰어 노드 (포트 8765 웹 서버)
    web_node = ExecuteProcess(
        cmd=["python3", "/home/pogo/2026_ROBO_1/scripts/jetson_lidar_web.py"],
        output="screen"
    )

    # 4. RViz2 시각화 (선택 실행)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    return LaunchDescription([
        rviz_arg,
        lidar_node,
        slam_node,
        web_node,
        rviz_node
    ])
