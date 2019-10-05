from pathlib import Path

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

def generate_launch_description():
    config = Path(get_package_share_directory('omni_demo'), 'config')
    nav2_yaml = config / 'nav2.yaml'
    assert nav2_yaml.is_file()
    bt_xml_path = config / 'navigate.xml'
    assert bt_xml_path.is_file()
    map_yaml_filename = config / 'map.yaml'
    assert map_yaml_filename.is_file()

    return LaunchDescription([
        launch_ros.actions.Node(
            node_name='lifecycle_manager',
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            output='screen',
            parameters=[
                nav2_yaml,
                {
                    'autostart': True,
                    'node_names': ['map_server', 'amcl', 'world_model', 'dwb_controller', 'navfn_planner', 'bt_navigator'],
                }
            ]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='map_server',
            package='nav2_map_server',
            node_executable='map_server',
            output='screen',
            parameters=[nav2_yaml,{'yaml_filename':str(map_yaml_filename)}]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='amcl',
            package='nav2_amcl',
            node_executable='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='world_model',
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[nav2_yaml]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='dwb_controller',
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[nav2_yaml]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='navfn_planner',
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            output='screen',
            parameters=[nav2_yaml]
        ),
        launch_ros.actions.LifecycleNode(
            node_name='bt_navigator',
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'bt_xml_filename':str(bt_xml_path)}]
        ),
        launch_ros.actions.Node(
            node_name='recoveries_node',
            package='nav2_recoveries',
            node_executable='recoveries_node',
            output='screen',
            parameters=[nav2_yaml]
        ),
    ])
