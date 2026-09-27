import os
import time

import cv2
import rclpy
import ros2_numpy as rnp
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

from ads_pipeline.lidar_projection import (build_extrinsic, build_intrinsic_matrix, colorize_depth,
                                           overlay_projection, projection_lidar_to_image)
from ads_pipeline.perf_stats import FrameStats

STAGES = ['lidar_decode', 'project', 'colorize', 'image_decode', 'overlay', 'output']

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
        self.declare_parameter('save_output', True)
        self.declare_parameter('overlay_topic', '')
        # benchmarking: per-frame timings/counters are written here on shutdown ('' = off)
        self.declare_parameter('bench.stats_dir', '')
        # set when the publisher stamps messages with wall-clock time (bench_replay), so
        # end-to-end latency (publish -> overlay out) can be measured
        self.declare_parameter('bench.wall_stamps', False)

        self.fov = float(self.get_parameter('camera.fov').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.output_path = os.path.expanduser(self.get_parameter('output_path').value)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        self.save_output = bool(self.get_parameter('save_output').value)
        self.stats_dir = os.path.expanduser(self.get_parameter('bench.stats_dir').value)
        self.wall_stamps = bool(self.get_parameter('bench.wall_stamps').value)
        self.stats = FrameStats(STAGES)

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
        overlay_topic = self.get_parameter('overlay_topic').value
        self.overlay_pub = self.create_publisher(Image, overlay_topic, 10) if overlay_topic else None

    def lidar_callback(self, msg):
        self.latest_lidar = msg

    def camera_callback(self, msg):
        self.stats.count('received')
        if self.latest_lidar is None:
            self.stats.count('skipped_no_lidar')
            return

        if self.K_size != (msg.width, msg.height):
            self.K = build_intrinsic_matrix(image_w=msg.width, image_h=msg.height, fov_deg=self.fov)
            self.K_size = (msg.width, msg.height)

        ms = {}
        t = time.perf_counter()

        def lap(stage):
            nonlocal t
            now = time.perf_counter()
            ms[stage] = (now - t) * 1e3
            t = now

        # ros2_numpy returns a dict with an (N,3) 'xyz' array (ROS frame: x forward, y left, z up)
        points_xyz = rnp.numpify(self.latest_lidar)['xyz']
        lap('lidar_decode')

        pixels, depths = projection_lidar_to_image(points_xyz=points_xyz, K=self.K,
                                                   T_cam_lidar=self.T,
                                                   image_w=msg.width, image_h=msg.height)
        lap('project')
        if len(depths) == 0:
            self.stats.count('skipped_no_points')
            return

        colors = colorize_depth(depths, max_depth=self.max_depth)
        lap('colorize')
        bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        lap('image_decode')
        overlay = overlay_projection(image_bgr=bgr, pixels=pixels, colors=colors)
        lap('overlay')

        if self.overlay_pub is not None:
            out = self.bridge.cv2_to_imgmsg(overlay, 'bgr8')
            out.header = msg.header
            self.overlay_pub.publish(out)
        if self.save_output and not cv2.imwrite(self.output_path, overlay):
            self.get_logger().error(f'failed to write {self.output_path}')
        lap('output')

        self.stats.count('processed')
        e2e = float('nan')
        if self.wall_stamps:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            e2e = (time.time() - stamp) * 1e3
        self.stats.add_frame(time.perf_counter(), ms, e2e)

    def dump_stats(self):
        if not self.stats_dir:
            return
        summary = self.stats.dump(self.stats_dir, extra={
            'save_output': self.save_output,
            'overlay_topic': self.get_parameter('overlay_topic').value,
            'image_size': list(self.K_size) if self.K_size else None,
        })
        self.get_logger().info(f'benchmark stats written to {self.stats_dir}: {summary["counters"]}')

def main(args=None):
    rclpy.init(args=args)
    projection = ProjectionNode()
    try:
        rclpy.spin(projection)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        projection.dump_stats()
        projection.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
