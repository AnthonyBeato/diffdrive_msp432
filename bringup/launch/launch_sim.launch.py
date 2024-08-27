from launch import LaunchDescription
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
 

def generate_launch_description():
    robot_id = 'robot_1'

    # Obtener URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("diffdrive_msp432"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "use_sim:=true ",  # Establece la simulación como verdadera
            f"prefix:={robot_id}_"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Path to the controllers configuration file
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_msp432"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ]),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    # Nodo de control ros2_control usando el hardware simulado
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        namespace=robot_id,
        output="screen",
        remappings=[('/hardware_interface/DiffDriveMSP432Hardware', '/hardware_interface/DiffDriveMSP432Simulated')]
    )

    # Nodo de robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"frame_prefix": f"{robot_id}_"}],
        namespace=robot_id,
    )

    # Spawner para los controladores
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_id,
        arguments=["joint_state_broadcaster", "--controller-manager", f"/{robot_id}/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=robot_id,
        arguments=["diffbot_base_controller", "--controller-manager", f"/{robot_id}/controller_manager", "--activate"],
    )

    # Nodo para spawnear el robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', f'/{robot_id}/robot_description', '-entity', 'diffbot'],
        output='screen',
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        gazebo,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        spawn_entity,
    ]

    return LaunchDescription(nodes)
