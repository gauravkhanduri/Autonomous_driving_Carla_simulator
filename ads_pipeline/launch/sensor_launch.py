import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ads_pipeline')
    config_file = os.path.join(pkg_share, 'config', 'config_carla.yaml')

    host_arg = DeclareLaunchArgument(
        'host', default_value='localhost',
        description='CARLA server host'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='2000',
        description='CARLA server port'
    )
    spawn_index_arg = DeclareLaunchArgument(
        'spawn_index', default_value='6',
        description='Spawn point index for the ego vehicle'
    )

    sensor_manager_node = Node(
        package='ads_pipeline',
        executable='sensor_manager',
        name='sensor_manager',
        output='screen',
        parameters=[
            config_file,
            {
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'spawn_index': LaunchConfiguration('spawn_index'),
            },
        ],
    )
    

    return LaunchDescription([
        host_arg,
        port_arg,
        spawn_index_arg,
        sensor_manager_node,
    ])
