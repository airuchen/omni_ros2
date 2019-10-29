from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory('omni_ros2'), 'urdf', 'omni_bot.urdf')
    assert urdf.is_file()
    hardware_config = Path(get_package_share_directory('omni_ros2'), 'config', 'hardware.yaml')
    assert hardware_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='rplidar_ros',
            node_executable='rplidar_composition',
            output='screen',
            parameters=[hardware_config],
        ),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            output='screen',
            arguments=[str(urdf)],
            parameters=[hardware_config]
         ),

         Node(
             package='joint_state_publisher', 
             node_executable='joint_state_publisher',
             output='screen',
             arguments=[str(urdf)], 
             parameters=[hardware_config]
         ),
        ])
