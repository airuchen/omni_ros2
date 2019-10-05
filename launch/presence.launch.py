from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    presence_config = Path(get_package_share_directory('omni_demo'), 'config', 'presence.yaml')
    assert presence_config.is_file()
    # urdf = Path(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    # assert urdf.is_file()
    # assert presence_config.is_file()

    return LaunchDescription([

    # todo: add robot_state_publisher, but why do they add this for the second time?
    # Node(
    #     package='robot_state_publisher', node_executable='robot_state_publisher', output='screen', argument=[str(urdf)], parameters=[presence_config]
    #     ),
    Node(
        package='robot_localization',
        node_executable='se_node',
        output='screen',
        parameters=[presence_config],
        remappings=[('odometry/filtered', 'odom')]
        ),
    ])
    
    
