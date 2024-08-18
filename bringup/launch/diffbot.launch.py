# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import socket


def generate_launch_description():

    def create_robot_nodes(namespace):
        # Get URDF via xacro
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("diffdrive_msp432"), "urdf", "diffbot.urdf.xacro"]
                ),
            ]
        )
        robot_description = {"robot_description": robot_description_content}

        robot_controllers = PathJoinSubstitution(
            [
                FindPackageShare("diffdrive_msp432"),
                "config",
                "diffbot_controllers.yaml",
            ]
        )
        # rviz_config_file = PathJoinSubstitution(
        #     [FindPackageShare("diffdrive_msp432"), "rviz", "diffbot.rviz"]
        # )


        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
            namespace=namespace
        )

        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            namespace=namespace,
            remappings=[
                (f"/{namespace}/diff_drive_controller/cmd_vel_unstamped", f"/{namespace}/cmd_vel"),
            ],
        )

        # rviz_node = Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     output="log",
        #     arguments=["-d", rviz_config_file],
        # )

        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", f"/{namespace}/controller_manager"],
            namespace=namespace,
        )

        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diffbot_base_controller", "--controller-manager", f"/{namespace}/controller_manager"],
            namespace=namespace,
        )

        # # Delay rviz start after `joint_state_broadcaster`
        # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[rviz_node],
        #     )
        # )

        # # Delay start of robot_controller after `joint_state_broadcaster`
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[robot_controller_spawner],
        #     )
        # )

        return [
            control_node,
            robot_state_pub_node,
            joint_state_broadcaster_spawner,
            robot_controller_spawner
            # delay_rviz_after_joint_state_broadcaster_spawner,
            # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        ]
    
    # Hostname de RPI
    hostname = socket.gethostname()

    # Asignando namespace basado en el hostname
    if hostname == "rpirobot1":
        nodes = create_robot_nodes("robot_1")
    elif hostname == "rpirobot2":
        nodes = create_robot_nodes("robot_2")
    else:
        raise RuntimeError(f"Unknown hostname: {hostname}")


    return LaunchDescription(nodes)
