# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Physical Leader → Isaac Sim Follower Teleop Launch File

물리 팔을 리더(Leader)로, Isaac Sim을 팔로워(Follower)로 동작시킵니다.

데이터 흐름:
  물리 팔 (CAN) → ros2_control → /joint_states
                                       ↓
                           [physical_leader_relay 노드]
                                       ↓
                               /real_joint_states
                                       ↓
                          Isaac Sim (구독하여 로봇 동기화)
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_robot_description(context: LaunchContext, description_package,
                                description_file, arm_type,
                                right_can_interface, left_can_interface):
    """Generate robot description using xacro processing."""
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    right_can_str = context.perform_substitution(right_can_interface)
    left_can_str = context.perform_substitution(left_can_interface)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        'urdf', 'robot', description_file_str
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            'arm_type': arm_type_str,
            'bimanual': 'true',
            'use_fake_hardware': 'false',   # 항상 실제 하드웨어 사용
            'ros2_control': 'true',
            'right_can_interface': right_can_str,
            'left_can_interface': left_can_str,
        }
    ).toprettyxml(indent='  ')

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package,
                         description_file, arm_type, controllers_file,
                         right_can_interface, left_can_interface):
    """Spawn robot_state_publisher and ros2_control_node."""
    robot_description = generate_robot_description(
        context, description_package, description_file, arm_type,
        right_can_interface, left_can_interface,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {'robot_description': robot_description}

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def generate_launch_description():
    """Launch physical arm as leader, relay joint_states to Isaac Sim."""

    declared_arguments = [
        DeclareLaunchArgument(
            'description_package',
            default_value='openarm_description',
            description='Description package with robot URDF/xacro files.',
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='v10.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        ),
        DeclareLaunchArgument(
            'arm_type',
            default_value='v10',
            description='Type of arm (e.g., v10).',
        ),
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='openarm_bringup',
            description='Package with the controller configuration.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='openarm_v10_bimanual_controllers.yaml',
            description='Controllers YAML file.',
        ),
        DeclareLaunchArgument(
            'right_can_interface',
            default_value='can0',
            description='CAN interface for the right arm.',
        ),
        DeclareLaunchArgument(
            'left_can_interface',
            default_value='can1',
            description='CAN interface for the left arm.',
        ),
        # Relay 노드 파라미터
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='60.0',
            description='Rate at which joint states are relayed to Isaac Sim (Hz).',
        ),
        DeclareLaunchArgument(
            'enable_lpf',
            default_value='false',
            description='Enable low-pass filter on joint states before sending to Isaac Sim.',
        ),
        DeclareLaunchArgument(
            'lpf_alpha',
            default_value='0.5',
            description='LPF alpha (0=no change, 1=no filter). Valid when enable_lpf=true.',
        ),
    ]

    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    arm_type = LaunchConfiguration('arm_type')
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file_arg = LaunchConfiguration('controllers_file')
    right_can_interface = LaunchConfiguration('right_can_interface')
    left_can_interface = LaunchConfiguration('left_can_interface')

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'config',
         'v10_controllers', controllers_file_arg]
    )

    # 1) Robot state publisher + ros2_control_node (물리 팔 CAN 제어)
    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[description_package, description_file, arm_type,
              controllers_file, right_can_interface, left_can_interface],
    )

    # 2) joint_state_broadcaster (물리 팔 상태 → /joint_states publish)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # 3) physical_leader_relay 노드 (/joint_states → /real_joint_states)
    physical_leader_relay_node = Node(
        package='openarm_converters',
        executable='physical_leader_relay',
        name='physical_leader_relay',
        output='screen',
        parameters=[{
            'hw_joint_state_topic': '/joint_states',
            'isaac_feedback_topic': '/real_joint_states',
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'enable_lpf': LaunchConfiguration('enable_lpf'),
            'lpf_alpha': LaunchConfiguration('lpf_alpha'),
        }],
    )

    LAUNCH_DELAY = 1.0

    delayed_joint_state_broadcaster = TimerAction(
        period=LAUNCH_DELAY,
        actions=[joint_state_broadcaster_spawner],
    )

    # relay 노드는 joint_state_broadcaster가 올라온 뒤 시작
    delayed_relay = TimerAction(
        period=LAUNCH_DELAY + 1.0,
        actions=[physical_leader_relay_node],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            delayed_joint_state_broadcaster,
            delayed_relay,
        ]
    )
