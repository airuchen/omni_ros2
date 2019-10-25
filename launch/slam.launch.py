from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='cartographer_ros',
            node_executable='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', get_package_share_directory('omni_ros2') + '/config',
                '-configuration_basename', 'cartographer.lua'
                ],
            ),
        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.02', '-publish_period_sec', '1.0']
            ),
        ])
