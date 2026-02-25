# Copyright 2025 Enactic, Inc.
# Copyright 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def namespace_from_context(context, arm_prefix):
    arm_prefix_str = context.perform_substitution(arm_prefix)
    if arm_prefix_str:
        return arm_prefix_str.strip('/')
    return None


def generate_robot_description(context: LaunchContext, description_package, description_file,
                               arm_type, use_fake_hardware, right_can_interface, left_can_interface):
    """Generate robot description using xacro processing."""

    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf",
        "robot",
        description_file_str,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "right_can_interface": right_can_interface_str,
            "left_can_interface": left_can_interface_str,
        },
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(context: LaunchContext, description_package, description_file,
                         arm_type, use_fake_hardware, controllers_file,
                         right_can_interface, left_can_interface, arm_prefix):
    """Spawn robot_state_publisher and ros2_control_node."""

    robot_description = generate_robot_description(
        context,
        description_package,
        description_file,
        arm_type,
        use_fake_hardware,
        right_can_interface,
        left_can_interface,
    )

    robot_description_param = {"robot_description": robot_description}
    controllers_file_str = context.perform_substitution(controllers_file)

    ns = namespace_from_context(context, arm_prefix)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=ns,
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ns,
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node]


def controller_spawner(context: LaunchContext, robot_controller, arm_prefix):
    """Spawn either forward_position_controller or joint_trajectory_controller."""
    controller_to_start = context.perform_substitution(robot_controller)
    ns = namespace_from_context(context, arm_prefix)
    cm = f"/{ns}/controller_manager" if ns else "/controller_manager"

    # NOTE: 원래 코드 로직 유지 (여기 내용은 기존 파일에 있던 그대로라고 가정)
    return [Node(
        package="controller_manager",
        executable="spawner",
        namespace=ns,
        arguments=[
            f"left_{controller_to_start}",
            f"right_{controller_to_start}",
            "-c",
            cm,
        ],
    )]


# === [추가] bimanual_converter 노드 생성 함수 ===
def bimanual_converter_spawner(context: LaunchContext, arm_prefix):
    ns = namespace_from_context(context, arm_prefix)

    # 네임스페이스 적용된 토픽 유틸
    def nstopic(topic: str) -> str:
        # topic은 "/xxx" 형태로 들어온다고 가정
        if ns:
            return f"/{ns}{topic}"
        return topic

    return [Node(
        package="openarm_converters",
        executable="bimanual_converter",
        name="bimanual_converter",
        namespace=ns,
        output="screen",
        parameters=[{
            # Isaac Sim → converter 입력
            "joint_state_topic": context.perform_substitution(LaunchConfiguration("isaac_joint_state_topic")),

            # converter → 물리 팔 컨트롤러 명령 (forward_position_controller 전제)
            "left_cmd_topic": nstopic("/left_forward_position_controller/commands"),
            "right_cmd_topic": nstopic("/right_forward_position_controller/commands"),
            "left_gripper_cmd_topic": nstopic("/left_gripper_controller/commands"),
            "right_gripper_cmd_topic": nstopic("/right_gripper_controller/commands"),

            # 주기/필터
            "command_rate_hz": float(context.perform_substitution(LaunchConfiguration("command_rate_hz"))),
            "lpf_alpha": float(context.perform_substitution(LaunchConfiguration("lpf_alpha"))),

            # 피드백 릴레이 (선택)
            "enable_feedback_relay": context.perform_substitution(LaunchConfiguration("enable_feedback_relay")).lower() in ["true", "1", "yes"],
            "hw_joint_state_topic": nstopic("/joint_states"),
            "sim_feedback_topic": nstopic("/real_joint_states"),
        }],
    )]


def generate_launch_description():
    """Generate launch description for OpenArm bimanual configuration."""

    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
            description="Description package with robot URDF/xacro files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "arm_type",
            default_value="v10",
            description="Type of arm (e.g., v10).",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware instead of real hardware.",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="forward_position_controller",
            choices=["forward_position_controller", "joint_trajectory_controller"],
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="openarm_bringup",
            description="Package with the controller's configuration in config folder.",
        ),
        DeclareLaunchArgument(
            "arm_prefix",
            default_value="",
            description="Prefix for the arm for topic namespacing.",
        ),
        DeclareLaunchArgument(
            "right_can_interface",
            default_value="can0",
            description="CAN interface to use for the right arm.",
        ),
        DeclareLaunchArgument(
            "left_can_interface",
            default_value="can1",
            description="CAN interface to use for the left arm.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_bimanual_controllers.yaml",
            description="Controllers file(s) to use. Can be a single file or comma-separated list of files.",
        ),

        # === [추가] converter 관련 런치 인자 ===
        DeclareLaunchArgument(
            "isaac_joint_state_topic",
            default_value="/isaac_joint_states",
            description="Topic name Isaac Sim publishes joint states to.",
        ),
        DeclareLaunchArgument(
            "command_rate_hz",
            default_value="20.0",
            description="Rate at which commands are sent to the physical arm (Hz).",
        ),
        DeclareLaunchArgument(
            "lpf_alpha",
            default_value="0.25",
            description="Low-pass filter alpha for command smoothing (0=no update, 1=no filter).",
        ),
        DeclareLaunchArgument(
            "enable_feedback_relay",
            default_value="true",
            description="Relay /joint_states -> /real_joint_states for Isaac Sim feedback.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    rightcan_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "v10_controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package, description_file, arm_type, use_fake_hardware,
            controllers_file, rightcan_interface, left_can_interface, arm_prefix
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "bimanual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                if namespace_from_context(context, arm_prefix) else "/controller_manager",
            ],
        )]
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner,
        args=[robot_controller, arm_prefix],
    )

    gripper_controller_spawner = OpaqueFunction(
        function=lambda context: [Node(
            package="controller_manager",
            executable="spawner",
            namespace=namespace_from_context(context, arm_prefix),
            arguments=[
                "left_gripper_controller",
                "right_gripper_controller",
                "-c",
                f"/{namespace_from_context(context, arm_prefix)}/controller_manager"
                if namespace_from_context(context, arm_prefix) else "/controller_manager",
            ],
        )]
    )

    # === [추가] converter 노드 (컨트롤러 이후 기동) ===
    bimanual_converter_node = OpaqueFunction(
        function=bimanual_converter_spawner,
        args=[arm_prefix],
    )

    LAUNCH_DELAY_SECONDS = 1.0

    delayed_joint_state_broadcaster = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[joint_state_broadcaster_spawner],
    )

    delayed_robot_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[controller_spawner_func],
    )

    delayed_gripper_controller = TimerAction(
        period=LAUNCH_DELAY_SECONDS,
        actions=[gripper_controller_spawner],
    )

    delayed_bimanual_converter = TimerAction(
        period=LAUNCH_DELAY_SECONDS + 1.5,
        actions=[bimanual_converter_node],
    )

    return LaunchDescription(
        declared_arguments + [
            robot_nodes_spawner_func,
            rviz_node,
            delayed_joint_state_broadcaster,
            delayed_robot_controller,
            delayed_gripper_controller,
            delayed_bimanual_converter,  # <= 추가
        ]
    )