from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # urdf = Path(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    # assert urdf.is_file()
    # hardware_config = Path(get_package_share_directory('openrover_demo'), 'config', 'hardware.yaml')
    # assert hardware_config.is_file()

    return LaunchDescription([
        Node(
            package='ydlidar',
            node_executable='ydlidar_node',
            output='screen',
            # parameter=[hardware_config],
            ),
    
        # walkaround: pub laser tf    
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.235', '3.14', '0', '0', 'base_link', 'laser_frame']
            ),
    
    
        # todo: create joint_state_publisher
        # Node(
        #     package='joint_state_publisher',
        #     node_executable='joint_state_publisher',
        #     output='screen',
        #     arguments=[str(urdf)],
        #     parameters=[hardware_config]
        #     ),
        ])
