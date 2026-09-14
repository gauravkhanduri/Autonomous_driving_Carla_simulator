import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('ads_pipeline')

    launch_args = [
        DeclareLaunchArgument('host', default_value='localhost', description='CARLA server host'),
        DeclareLaunchArgument('port', default_value='2000', description='CARLA server port'),
        DeclareLaunchArgument('spawn_index', default_value='6', description='Spawn point index for the ego vehicle'),
    ]

    # Reuse sensor_launch.py so the node gets the same name and config_carla.yaml
    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sensor_launch.py')),
        launch_arguments={
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'spawn_index': LaunchConfiguration('spawn_index'),
        }.items(),
    )

    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(
        os.path.expanduser('~'), 'recorded_sensor_bag',
        f'sensor_bag_{timestamp}'
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/carla/camera/rgb/image',
            '/carla/lidar/points',
            '/carla/imu/data',
            '-o', bag_path,
        ],
        output='screen'
    )
    return LaunchDescription(launch_args + [sensor_launch, bag_record])
