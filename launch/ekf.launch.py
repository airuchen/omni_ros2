from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    config = Path(get_package_share_directory('omni_demo'),'config','presence.yaml')
    assert config.is_file()

    return LaunchDescription([
        Node(
            package='robot_localization',
            node_executable='se_node',
            output='screen',
            parameters=[config],
            # remappings=[('odometry/filtered', 'odom')]
            ),
        ])
