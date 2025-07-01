import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    map_yaml_sim = os.path.join(get_package_share_directory('map_server'), 'config', 'apartment_sim.yaml')
    localization_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    rviz_config_sim = os.path.join(get_package_share_directory('fastbot_slam'), 'rviz', 'fastbot_nav.rviz')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_sim.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator_sim.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_sim.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recoveries_sim.yaml')
    
    remappings=[('/cmd_vel', '/fastbot_1/cmd_vel'), ('/scan', '/fastbot_1/scan'), ('/odom', '/fastbot_1/odom')]

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
 
    load_nodes_sim = GroupAction(
        condition=IfCondition(use_sim_time),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_yaml_sim
                }],
            ),
            
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[localization_yaml_sim],
                remappings=remappings
            ),

            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[controller_yaml],
                remappings=remappings
            ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[planner_yaml]
            ),
                
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='recoveries_server',
                parameters=[recovery_yaml],
                output='screen'
            ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[bt_navigator_yaml]
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_sim]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_pathplanner',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server',
                                            'amcl',
                                            'planner_server',
                                            'controller_server',
                                            'bt_navigator',
                                            'recoveries_server',
                                            ]}]
            )
        ]
    )

    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(load_nodes_sim)
    return ld