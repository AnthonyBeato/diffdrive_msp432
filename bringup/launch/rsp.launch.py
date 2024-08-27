import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Argumentos del launch
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    robot_id = LaunchConfiguration('robot_id')
    use_sim = LaunchConfiguration('use_sim')

    # Procesando el archivo URDF/Xacro
    pkg_path = os.path.join(get_package_share_directory('diffdrive_msp432'))
    xacro_file = os.path.join(pkg_path, 'urdf', 'diffbot.urdf.xacro')
    robot_description_config = Command([
        FindExecutable(name="xacro"),
        " ",  # Aquí agregamos un espacio para separar el comando de los argumentos
        xacro_file,
        " ",  # Agregamos un espacio para separar el archivo de los siguientes argumentos
        "use_sim:=", use_sim,
        " ",  # Espacio para separar argumentos
        "use_ros2_control:=", use_ros2_control,
        " ",  # Espacio para separar argumentos
        "prefix:=", robot_id
    ])

    # Configuración del robot_state_publisher
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        namespace=robot_id
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        DeclareLaunchArgument(
            'robot_id',
            default_value='robot_1',
            description='Robot ID for namespace'),
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',  # El valor por defecto es 'false' para hardware real
            description='Set to true if running in simulation'),
        node_robot_state_publisher
    ])
