# Copyright 2025 Enactic, Inc.
# Licensed under the Apache License, Version 2.0

"""
REAL (Torque-Free) Leader → Isaac Sim Follower

- ros2_control_node 띄우지 않음
- openarm_state_publisher: CAN → /joint_states & /real_joint_states (통합됨)
- robot_state_publisher: TF
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robot_description(context: LaunchContext, description_package, description_file, arm_type):
    pkg = context.perform_substitution(description_package)
    desc = context.perform_substitution(description_file)
    arm = context.perform_substitution(arm_type)

    xacro_path = os.path.join(
        get_package_share_directory(pkg),
        "urdf", "robot", desc
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm,
            "bimanual": "true",
            "ros2_control": "false",   # ✅ 토크-free 모드: ros2_control 비활성
            "use_fake_hardware": "true",
        }
    ).toprettyxml(indent="  ")

    return robot_description


def robot_state_pub_spawner(context: LaunchContext, description_package, description_file, arm_type):
    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type)
    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package",
                              default_value="openarm_description"),
        DeclareLaunchArgument("description_file",
                              default_value="v10.urdf.xacro"),
        DeclareLaunchArgument("arm_type", default_value="v10"),

        # CAN state publisher
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("enable_fd", default_value="true"),
        DeclareLaunchArgument("state_publish_rate_hz", default_value="100.0"),
        DeclareLaunchArgument("try_torque_off_on_start", default_value="true"),
        DeclareLaunchArgument("reassert_torque_off_hz", default_value="1.0"),

        # feedback topic
        DeclareLaunchArgument("isaac_feedback_topic",
                              default_value="/real_joint_states"),
        # ✅ 리더 모드에서는 필터가 보통 필요함
        DeclareLaunchArgument("enable_lpf", default_value="true"),
        DeclareLaunchArgument("lpf_alpha", default_value="0.5"),
    ]

    robot_state_pub = OpaqueFunction(
        function=robot_state_pub_spawner,
        args=[
            LaunchConfiguration("description_package"),
            LaunchConfiguration("description_file"),
            LaunchConfiguration("arm_type"),
        ],
    )

    state_pub = Node(
        package="openarm_converters",
        executable="real_to_sim_converter",
        name="real_to_sim_converter",
        output="screen",
        parameters=[{
            "publish_topic": "/joint_states",
            # ✅ 통합된 피드백 토픽
            "feedback_publish_topic": LaunchConfiguration("isaac_feedback_topic"),
            "publish_rate_hz": LaunchConfiguration("state_publish_rate_hz"),
            "left_can_interface": LaunchConfiguration("left_can_interface"),
            "right_can_interface": LaunchConfiguration("right_can_interface"),
            "enable_fd": LaunchConfiguration("enable_fd"),

            # LPF 설정 ✅
            "enable_lpf": LaunchConfiguration("enable_lpf"),
            "lpf_alpha": LaunchConfiguration("lpf_alpha"),

            # candump 기반 기본 ID (원하면 여기서 override)
            "arm_motor_types": ["DM4310"] * 7,
            "arm_send_ids": ["0x01", "0x02", "0x03", "0x04", "0x05", "0x06", "0x07"],
            "arm_recv_ids": ["0x11", "0x12", "0x13", "0x14", "0x15", "0x16", "0x17"],
            "gripper_motor_type": "DM4310",
            "gripper_send_id": "0x08",
            "gripper_recv_id": "0x18",

            "try_torque_off_on_start": LaunchConfiguration("try_torque_off_on_start"),
            "reassert_torque_off_hz": LaunchConfiguration("reassert_torque_off_hz"),

            "left_joint_names": [
                "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3", "openarm_left_joint4",
                "openarm_left_joint5", "openarm_left_joint6", "openarm_left_joint7", "openarm_left_finger_joint1",
            ],
            "right_joint_names": [
                "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3", "openarm_right_joint4",
                "openarm_right_joint5", "openarm_right_joint6", "openarm_right_joint7", "openarm_right_finger_joint1",
            ],
        }],
    )

    return LaunchDescription(declared_arguments + [
        robot_state_pub,
        state_pub,
    ])
