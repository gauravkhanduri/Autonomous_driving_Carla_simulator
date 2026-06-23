import os 
from datetime import datetime
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ads_pipeline')
    
    sensor_manager_node = Node(
        package='ads_pipeline',
        executable='sensor_manager',

    )
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(
        os.path.expanduser('~'),'recorded_sensor_bag',
        f'sensor_bag_{timestamp}'
    )

    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/carla/camera/rgb/image',
            '/carla/lidar/points',
            '/carla/imu/data',
            '-o',bag_path,
        ],
        output='screen'
    )
    return LaunchDescription([sensor_manager_node,bag_record])