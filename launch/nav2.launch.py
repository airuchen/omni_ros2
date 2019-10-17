from pathlib import Path

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # define configuration file path 
    config = Path(get_package_share_directory('omni_ros2'), 'config')
    nav2_yaml = config / 'nav2.yaml'
    assert nav2_yaml.is_file()
    bt_xml_path = config / 'navigate.xml'
    assert bt_xml_path.is_file()
    map_yaml_filename = config / 'map.yaml'
    assert map_yaml_filename.is_file()
    rviz_config = Path(get_package_share_directory('omni_ros2'), 'config', 'default.rviz').resolve()
    assert rviz_config.is_file()

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
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
    )

    start_map_server_cmd = launch_ros.actions.Node(
        # node_name='map_server',
        package='nav2_map_server',
        node_executable='map_server',
        output='screen',
        parameters=[nav2_yaml,{'yaml_filename':str(map_yaml_filename)}]
    )

    # start_rviz_cmd = launch_ros.actions.Node(
    #         package='rviz2',
    #         node_executable='rviz2',
    #         node_name='rviz2',
    #         arguments=['--display-config', str(rviz_config), '--fixed-frame', 'map'],
    #         output='log'
    # )

    start_amcl_cmd = launch_ros.actions.Node(
        # node_name='amcl',
        package='nav2_amcl',
        node_executable='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )
    start_world_model_cmd = launch_ros.actions.Node(
        # node_name='world_model',
        package='nav2_world_model',
        node_executable='world_model',
        output='screen',
        parameters=[nav2_yaml]
    )
    start_dwb_cmd = launch_ros.actions.Node(
        # node_name='dwb_controller',
        package='dwb_controller',
        node_executable='dwb_controller',
        output='screen',
        parameters=[nav2_yaml]
    )
    start_planner_cmd = launch_ros.actions.Node(
        # node_name='navfn_planner',
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        output='screen',
        parameters=[nav2_yaml]
    )
    start_navigator_cmd = launch_ros.actions.Node(
        # node_name='bt_navigator',
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        output='screen',
        parameters=[nav2_yaml, {'bt_xml_filename':str(bt_xml_path)}]
    )
    start_recovery_cmd = launch_ros.actions.Node(
        # node_name='recoveries_node',
        package='nav2_recoveries',
        node_executable='recoveries_node',
        output='screen',
        parameters=[nav2_yaml]
    )

    # I totally fail
    # start_teleop_cmd = launch_ros.actions.Node(
    #     package='teleop_twist_keyboard',
    #     node_executable='teleop_twist_keyboard',
    #     output='screen'
    # )
   
    # create the launch description and populate
    ld = launch.LaunchDescription()
    
    # set environment varibales
    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd )
    ld.add_action(start_amcl_cmd )
    ld.add_action(start_world_model_cmd )
    ld.add_action(start_dwb_cmd )
    ld.add_action(start_planner_cmd )
    ld.add_action(start_navigator_cmd )
    ld.add_action(start_recovery_cmd )

    return ld
