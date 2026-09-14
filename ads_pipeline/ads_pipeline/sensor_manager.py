import carla
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField


class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')
        self.get_logger().info("Sensor Manager Node has been started.")

        #-----------parameters---------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 2000)
        self.declare_parameter('vehicle_blueprint', 'vehicle.tesla.model3')
        self.declare_parameter('spawn_index', 6)

        self.declare_parameter('camera_topic', '/carla/camera/rgb/image')
        self.declare_parameter('lidar_topic', '/carla/lidar/points')
        self.declare_parameter('imu_topic', '/carla/imu/data')

        self.declare_parameter('camera.width', 1920)
        self.declare_parameter('camera.height', 1080)
        self.declare_parameter('camera.fov', 90)

        self.declare_parameter('rotation_frequency', 20)
        self.declare_parameter('channels', 32)
        self.declare_parameter('range', 100)
        self.declare_parameter('points_per_second', 100000)

        self.declare_parameter('imu_update_rate', 100)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value

        #---------------CARLA connection-----------
        self.client = carla.Client(host, port)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        self.blueprint_lib = self.world.get_blueprint_library()
        self.actors = []
        self.bridge = CvBridge()

        camera_topic = self.get_parameter('camera_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self.cam_pub_ = self.create_publisher(Image, camera_topic, 10)
        self.lidar_pub_ = self.create_publisher(PointCloud2, lidar_topic, 10)
        self.imu_pub_ = self.create_publisher(Imu, imu_topic, 10)

        self._spawn_vehicle()
        self._attach_sensors()
        self.get_logger().info('SensorManager initialized.')

    @staticmethod
    def _stamp(sensor_data):
        """Header stamp from the CARLA simulation time of the measurement, so that
        camera, LiDAR and IMU samples from the same simulation tick share a stamp."""
        t = sensor_data.timestamp
        sec = int(t)
        return Time(sec=sec, nanosec=int((t - sec) * 1e9))

    def _spawn_vehicle(self):
        bp_name = self.get_parameter('vehicle_blueprint').value
        bp = self.blueprint_lib.find(bp_name)
        spawn_idx = self.get_parameter('spawn_index').value
        spawn_points = self.world.get_map().get_spawn_points()
        if not 0 <= spawn_idx < len(spawn_points):
            raise ValueError(f'spawn_index {spawn_idx} out of range (map has {len(spawn_points)} spawn points)')
        self.vehicle = self.world.spawn_actor(bp, spawn_points[spawn_idx])
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
        bp.set_attribute('image_size_x', str(self.get_parameter('camera.width').value))
        bp.set_attribute('image_size_y', str(self.get_parameter('camera.height').value))
        bp.set_attribute('fov', str(self.get_parameter('camera.fov').value))

        transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._rgb_callback)
        self.actors.append(sensor)

    def _rgb_callback(self, carla_image):
        # CARLA delivers pixels as BGRA
        array = np.frombuffer(carla_image.raw_data, dtype=np.uint8)
        array = array.reshape((carla_image.height, carla_image.width, 4))
        array = array[:, :, :3]  # drop alpha -> BGR
        msg = self.bridge.cv2_to_imgmsg(array, 'bgr8')
        msg.header.stamp = self._stamp(carla_image)
        msg.header.frame_id = 'camera_rgb_front'
        self.cam_pub_.publish(msg)

    def _attach_lidar(self):
        bp = self.blueprint_lib.find('sensor.lidar.ray_cast')
        #set attributes from ros2 parameters
        bp.set_attribute('channels', str(self.get_parameter('channels').value))
        bp.set_attribute('range', str(self.get_parameter('range').value))
        bp.set_attribute('points_per_second', str(self.get_parameter('points_per_second').value))
        bp.set_attribute('rotation_frequency', str(self.get_parameter('rotation_frequency').value))

        self._lidar_chunks = []
        self._lidar_prev_stamp = None
        self._lidar_covered = 0.0
        self._lidar_period = 1.0 / float(self.get_parameter('rotation_frequency').value)

        transform = carla.Transform(carla.Location(x=0.0, z=2.5))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._lidar_callback)
        self.actors.append(sensor)

    def _lidar_callback(self, point_cloud):
        # CARLA delivers only the slice of the rotation covered during one simulation tick;
        # accumulate slices and publish once they cover one rotation period (a full 360 degree
        # sweep). Simulation time is used because horizontal_angle is always 0 in CARLA 0.9.16.
        dt = 0.0 if self._lidar_prev_stamp is None else point_cloud.timestamp - self._lidar_prev_stamp
        self._lidar_prev_stamp = point_cloud.timestamp
        self._lidar_covered += dt
        # copy: raw_data is only valid during this callback, but chunks are kept across callbacks
        self._lidar_chunks.append(np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4).copy())
        if self._lidar_covered < self._lidar_period - 0.5 * dt:
            return
        self._lidar_covered = 0.0

        points = np.concatenate(self._lidar_chunks)
        self._lidar_chunks = []
        # CARLA is left-handed (y right); flip y for the ROS convention (x forward, y left, z up)
        points[:, 1] *= -1.0

        msg = PointCloud2()
        msg.header.stamp = self._stamp(point_cloud)
        msg.header.frame_id = 'lidar'
        msg.height = 1
        msg.width = points.shape[0]
        msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                      PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                      PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                      PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        self.lidar_pub_.publish(msg)

    def _attach_imu(self):
        bp = self.blueprint_lib.find('sensor.other.imu')
        rate = float(self.get_parameter('imu_update_rate').value)
        if rate > 0:
            bp.set_attribute('sensor_tick', str(1.0 / rate))

        transform = carla.Transform(carla.Location(x=0.0, z=2.0))
        sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        sensor.listen(self._imu_callback)
        self.actors.append(sensor)

    def _imu_callback(self, imu_data):
        msg = Imu()
        msg.header.stamp = self._stamp(imu_data)
        msg.header.frame_id = 'imu'

        # Convert from CARLA's left-handed frame to ROS right-handed (same as carla-ros-bridge)
        msg.linear_acceleration.x = imu_data.accelerometer.x
        msg.linear_acceleration.y = -imu_data.accelerometer.y
        msg.linear_acceleration.z = imu_data.accelerometer.z

        msg.angular_velocity.x = -imu_data.gyroscope.x
        msg.angular_velocity.y = imu_data.gyroscope.y
        msg.angular_velocity.z = -imu_data.gyroscope.z

        # orientation is not estimated
        msg.orientation_covariance[0] = -1.0
        self.imu_pub_.publish(msg)

    def destroy_actors(self):
        self.get_logger().info('destroying actors...')
        for actor in reversed(self.actors):
            if isinstance(actor, carla.Sensor) and actor.is_listening:
                actor.stop()
            # release vehicles from the Traffic Manager first; destroying an autopilot vehicle
            # can make the TM thread abort with "trying to operate on a destroyed actor"
            elif isinstance(actor, carla.Vehicle) and actor.is_alive:
                actor.set_autopilot(False)
        for actor in reversed(self.actors):
            if actor.is_alive:
                actor.destroy()
        self.actors.clear()


def main(args=None):
    # Handle Ctrl+C ourselves: rclpy's default handler invalidates the context while CARLA
    # sensor threads are still publishing. Sensors are stopped before shutdown instead.
    # spin_once with a timeout returns to Python regularly so KeyboardInterrupt is delivered.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SensorManager()
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_actors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
