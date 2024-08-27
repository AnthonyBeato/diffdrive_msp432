import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import TextSubstitution

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='robot_1', description='Namespace for the robot'
    )

    namespace = LaunchConfiguration('namespace')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("diffdrive_msp432"), "urdf", "diffbot.urdf.xacro"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ", 
            xacro_file,
            " ",
            "use_sim_time:=", LaunchConfiguration('use_sim_time'),
            " ",
            "use_sim:=", "true",  
            " ",
            "prefix:=", namespace, TextSubstitution(text="_")
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("diffdrive_msp432"),
            "config",
            "diffbot_controllers_sim_gazebo.yaml",
        ]
    )

    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
)

    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', namespace],
        namespace=namespace,
        output='screen'
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        namespace=namespace,
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"frame_prefix": [namespace, TextSubstitution(text="_")]}],
        namespace=namespace,
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", f"/{namespace}/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["diffbot_base_controller", "--controller-manager", f"/{namespace}/controller_manager", "--activate"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_namespace,
        gazebo,
        spawn_entity,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ])
   
