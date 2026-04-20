from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value='',
        description='Path to .yaml costmap'
    )
    map_yaml = LaunchConfiguration('map_yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml,
                'use_sim_time': True,
            }
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 4.0,
                'attempt_respawn_reconnection': True,
                'bond_respawn_max_duration': 10.0,
                'bond_heartbeat_period': 0.25,
            }
        ]
    )

    return LaunchDescription([
        declare_map_yaml,
        map_server,
        lifecycle_manager,
    ])