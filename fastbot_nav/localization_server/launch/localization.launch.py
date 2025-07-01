import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    
    map_yaml_sim = os.path.join(get_package_share_directory('map_server'), 'config', 'apartment_sim.yaml')
    localization_yaml_sim = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_sim.yaml')
    rviz_config_sim = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localizer_sim.rviz')

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
                remappings=[('/scan', '/fastbot_1/scan'),('/odom', '/fastbot_1/odom')]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': True},
                            {'node_names': ['map_server',
                                            'amcl',
                                            ]}]
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_sim]
            )
        ]
    )

    # Declare the launch options
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)

    ld.add_action(load_nodes_sim)

    return ld