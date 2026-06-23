import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image, Imu, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import carla
import numpy as np
import time
import cv2
import sys
import threading

class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')
        self.get_logger().info("Sensor Manager Node has been started.")

        #-----------paramters---------
        self.declare_parameter('host','localhost')
        self.declare_parameter('port',2000)
        self.declare_parameter('vehicle_blueprint','vehicle.tesla.model3')
        self.declare_parameter('spawn_index',6)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        #---------------CARLA connection-----------
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_lib = self.world.get_blueprint_library()
        self.actors = []
        self.bridge = CvBridge()
  



        # Initialize sensor data storage
        self.sensor_data = {}
        # Set up subscriptions to sensor topics here (e.g., LiDAR, camera, IMU)
        # Example: self.create_subscription(LidarMsgType, 'lidar_topic', self.lidar_callback, 10)
        self.declare_parameter('camera_topic','/carla/camera/rgb/image')
        camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self.declare_parameter('lidar_topic','/carla/lidar/points')
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        self.declare_parameter('imu_topic','/carla/imu/data')
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.declare_parameter('camera.width', 1280)
        self.declare_parameter('camera.height',720)
        self.declare_parameter('camera.fov',90)

        self.declare_parameter('rotation_frequency', 20)
        self.declare_parameter('channels',32)
        self.declare_parameter('range',100)
        self.declare_parameter('points_per_second',1000000)

       
       
        self.cam_pub_ = self.create_publisher(Image, camera_topic , 10)
        self.lidar_pub_=  self.create_publisher(PointCloud2, lidar_topic, 10)
        self.imu_pub_ = self.create_publisher(Imu, imu_topic, 10)

        self._spawn_vehicle()
        self._attach_sensors()
        self.get_logger().info('SensorManager initialized.')


    def _spawn_vehicle(self):
        bp_name = self.get_parameter('vehicle_blueprint').value
        bp = self.blueprint_lib.find(bp_name)
        spawn_idx = self.get_parameter('spawn_index').value
        spawn_point = (self.world.get_map().get_spawn_points()[spawn_idx])
        self.vehicle = self.world.spawn_actor(bp,spawn_point)
        self.vehicle.set_autopilot(True)
        self.actors.append(self.vehicle)
        self.get_logger().info(f'spawned {bp_name} at spawn point {spawn_idx}.')

    def _attach_sensors(self):
        self._attach_rgb_camera()
        self._attach_lidar()
        self._attach_imu()

    def _attach_rgb_camera(self):
        bp = self.blueprint_lib.find('sensor.camera.rgb')
        #set attributes from ros2 parameters
        bp.set_attribute('image_size_x',str(self.get_parameter('camera.width').value))
        bp.set_attribute('image_size_y',str(self.get_parameter('camera.height').value))
        bp.set_attribute('fov',str(self.get_parameter('camera.fov').value))

        transform = carla.Transform(carla.Location(x = 1.5, z = 2.4))
        sensor = self.world.spawn_actor(bp,transform, attach_to=self.vehicle)
        sensor.listen(self._rgb_callback)
        self.actors.append(sensor)

    def _rgb_callback(self,carla_image):
        array = np.frombuffer(carla_image.raw_data,dtype=np.uint8)
        array = array.reshape((carla_image.height,carla_image.width,4))
        array = array[:, :, :3] #drop alpha
        msg = self.bridge.cv2_to_imgmsg(array,'rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera+rgb_front'
        self.cam_pub_.publish(msg)

    def _attach_lidar(self):
        bp  = self.blueprint_lib.find('sensor.lidar.ray_cast')
        #set attributes from ros2 parameters
        bp.set_attribute('channels', str(self.get_parameter('channels').value))
        bp.set_attribute('range', str(self.get_parameter('range').value))
        bp.set_attribute('points_per_second', str(self.get_parameter('points_per_second').value))
        bp.set_attribute('rotation_frequency', str(self.get_parameter('rotation_frequency').value))

        transform = carla.Transform(carla.Location(x=0.0, z=2.5))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._lidar_callback)
        self.actors.append(sensor)

    def _lidar_callback(self, point_cloud):
        data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
        array = data.reshape(-1,4)
        points = array.view(np.uint8)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'
        msg.height = 1
        msg.width = array.shape[0]
        msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = True
        msg.data = points.tobytes()
        self.lidar_pub_.publish(msg)
    
    def _attach_imu(self):
    
        bp = self.blueprint_lib.find('sensor.other.imu')

        transform = carla.Transform(carla.Location(x=0.0, z=2.0))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._imu_callback)
        self.actors.append(sensor)


    def _imu_callback(self, imu_data):
        
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu'

        #acceleration
        msg.linear_acceleration.x = imu_data.accelerometer.x
        msg.linear_acceleration.y = imu_data.accelerometer.y
        msg.linear_acceleration.z = imu_data.accelerometer.z

        #gyroscope
        msg.angular_velocity.x = imu_data.gyroscope.x
        msg.angular_velocity.y = imu_data.gyroscope.y
        msg.angular_velocity.z = imu_data.gyroscope.z
        self.imu_pub_.publish(msg)




    def destroy(self):
        self.get_logger().info('destroying actors...')
        for actor in reversed(self.actors):
            if actor.is_alive:
                actor.destroy()


def main(args=None):
    rclpy.init(args=args)
    node  = SensorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()