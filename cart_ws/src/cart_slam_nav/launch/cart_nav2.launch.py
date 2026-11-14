#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
4륜 메카넘 카트 Nav2 자율주행 및 웨이포인트 내비게이션 통합 Launch 파일
=============================================================================
실행 구성:
  1. D500 LiDAR 드라이버 노드
  2. WT901 IMU 드라이버 노드
  3. 메카넘 휠 오도메트리 노드
  4. robot_localization (EKF 센서 융합)
  5. Nav2 자율주행 스택 (AMCL 위치추정 + Costmap + DWB Local Planner)
  6. 카트 웨이포인트 매니저 (위치 저장/호출)
  7. RViz2 (경로 및 주행 모니터링)
=============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = "/home/pogo/2026_ROBO_1/cart_ws/src/cart_slam_nav"
    config_dir = os.path.join(pkg_dir, "config")
    tools_dir = "/home/pogo/2026_ROBO_1/tools"

    default_map_path = "/home/pogo/2026_ROBO_1/maps/school_map.yaml"
    nav2_params = os.path.join(config_dir, "nav2_params.yaml")
    ekf_config = os.path.join(config_dir, "ekf.yaml")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_path,
        description="불러올 지도 파일 전체 경로 (.yaml)"
    )

    # 1. 라이다 드라이버
    lidar_node = Node(
        package=None,
        executable=os.path.join(tools_dir, "cart_lidar_node.py"),
        name="cart_lidar_node",
        output="screen"
    )

    # 2. WT901 IMU 드라이버
    imu_node = Node(
        package=None,
        executable=os.path.join(pkg_dir, "cart_slam_nav", "wt901_imu_node.py"),
        name="wt901_driver_node",
        output="screen"
    )

    # 3. 메카넘 휠 오도메트리
    odom_node = Node(
        package=None,
        executable=os.path.join(pkg_dir, "cart_slam_nav", "mecanum_odom_node.py"),
        name="mecanum_odom_node",
        output="screen"
    )

    # 4. EKF 융합
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config]
    )

    # 5. Nav2 스택 공식 브링업 인클루드
    try:
        nav2_bringup_dir = get_package_share_directory("nav2_bringup")
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": LaunchConfiguration("map"),
                "params_file": nav2_params,
                "use_sim_time": "false",
                "autostart": "true"
            }.items()
        )
    except Exception:
        nav2_launch = Node(
            package="nav2_bringup",
            executable="bringup_launch.py",
            output="screen"
        )

    # 6. 웨이포인트 매니저 노드
    waypoint_node = Node(
        package=None,
        executable=os.path.join(pkg_dir, "cart_slam_nav", "cart_waypoint_manager.py"),
        name="cart_waypoint_manager",
        output="screen"
    )

    return LaunchDescription([
        map_arg,
        lidar_node,
        imu_node,
        odom_node,
        ekf_node,
        nav2_launch,
        waypoint_node
    ])

