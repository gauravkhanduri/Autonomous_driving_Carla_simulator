import os

import cv2
import rclpy
import ros2_numpy as rnp
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from ads_pipeline.lidar_projection import (build_extrinsic, build_intrinsic_matrix, colorize_depth,
                                           overlay_projection, projection_lidar_to_image)

# Default sensor mounts in CARLA vehicle coordinates (must match sensor_manager.py)
LIDAR_LOC = (0.0, 0.0, 2.5)
CAMERA_LOC = (1.5, 0.0, 2.4)


class ProjectionNode(Node):
    def __init__(self):
        super().__init__('projection_node')
        self.get_logger().info('Projection Node has been started')

        self.declare_parameter('camera_topic', '/carla/camera/rgb/image')
        self.declare_parameter('lidar_topic', '/carla/lidar/points')
        self.declare_parameter('camera.fov', 90.0)
        self.declare_parameter('max_depth', 50.0)
        self.declare_parameter('output_path', '~/results/projection_result.png')

        self.fov = float(self.get_parameter('camera.fov').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.output_path = os.path.expanduser(self.get_parameter('output_path').value)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)

        self.latest_lidar = None
        self.bridge = CvBridge()

        # K is built from the incoming image size so it always matches the camera config
        self.K = None
        self.K_size = None
        self.T = build_extrinsic(lidar_loc=LIDAR_LOC, camera_loc=CAMERA_LOC)

        self.lidar_sub = self.create_subscription(
            PointCloud2, self.get_parameter('lidar_topic').value, self.lidar_callback, 10)
        self.cam_sub = self.create_subscription(
            Image, self.get_parameter('camera_topic').value, self.camera_callback, 10)

    def lidar_callback(self, msg):
        self.latest_lidar = msg

    def camera_callback(self, msg):
        if self.latest_lidar is None:
            return

        if self.K_size != (msg.width, msg.height):
            self.K = build_intrinsic_matrix(image_w=msg.width, image_h=msg.height, fov_deg=self.fov)
            self.K_size = (msg.width, msg.height)

        # ros2_numpy returns a dict with an (N,3) 'xyz' array (ROS frame: x forward, y left, z up)
        points_xyz = rnp.numpify(self.latest_lidar)['xyz']

        pixels, depths = projection_lidar_to_image(points_xyz=points_xyz, K=self.K,
                                                   T_cam_lidar=self.T,
                                                   image_w=msg.width, image_h=msg.height)
        if len(depths) == 0:
            return

        colors = colorize_depth(depths, max_depth=self.max_depth)
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        overlay = overlay_projection(image_bgr=bgr, pixels=pixels, colors=colors)

        if not cv2.imwrite(self.output_path, overlay):
            self.get_logger().error(f'failed to write {self.output_path}')


def main(args=None):
    rclpy.init(args=args)
    projection = ProjectionNode()
    try:
        rclpy.spin(projection)
    except KeyboardInterrupt:
        pass
    finally:
        projection.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
