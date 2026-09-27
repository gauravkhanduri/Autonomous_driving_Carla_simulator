#ads_pipeline/bench_replay.py
"""
Replay camera + LiDAR frames from a recorded bag at a fixed rate, for benchmarking the
projection node independently of how fast CARLA renders.

Messages are re-stamped with wall-clock time at publish so the subscriber can measure
end-to-end latency. The number of published camera frames is written to <stats_dir>/published.json.

    ros2 run ads_pipeline bench_replay --bag ~/recorded_sensor_bag/<bag> --fps 60 --frames 3000
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Image, PointCloud2

CAMERA_TOPIC = '/carla/camera/rgb/image'
LIDAR_TOPIC = '/carla/lidar/points'


def load_pairs(bag_path, max_frames):
    """(camera, latest lidar sweep) pairs in recording order."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id='sqlite3'),
                ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'))
    pairs, lidar = [], None
    while reader.has_next() and len(pairs) < max_frames:
        topic, data, _ = reader.read_next()
        if topic == LIDAR_TOPIC:
            lidar = deserialize_message(data, PointCloud2)
        elif topic == CAMERA_TOPIC and lidar is not None:
            pairs.append((deserialize_message(data, Image), lidar))
    return pairs


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--frames', type=int, default=1000, help='camera frames to publish')
    ap.add_argument('--load', type=int, default=100, help='distinct frames loaded from the bag (looped)')
    ap.add_argument('--warmup', type=float, default=2.0, help='seconds to wait for subscribers')
    ap.add_argument('--stats-dir', default='')
    return ap.parse_args()


def main():
    args = parse_args()
    pairs = load_pairs(os.path.expanduser(args.bag), args.load)
    if not pairs:
        raise SystemExit(f'no camera/lidar pairs found in {args.bag}')

    rclpy.init()
    node = rclpy.create_node('bench_replay')
    cam_pub = node.create_publisher(Image, CAMERA_TOPIC, 10)
    lidar_pub = node.create_publisher(PointCloud2, LIDAR_TOPIC, 10)
    img = pairs[0][0]
    node.get_logger().info(f'loaded {len(pairs)} frames ({img.width}x{img.height}); '
                           f'publishing {args.frames} at {args.fps} FPS')
    time.sleep(args.warmup)

    period = 1.0 / args.fps
    published, last_lidar, late = 0, None, 0
    t_next = time.perf_counter()
    try:
        for i in range(args.frames):
            cam, lidar = pairs[i % len(pairs)]
            if lidar is not last_lidar:
                lidar.header.stamp = node.get_clock().now().to_msg()
                lidar_pub.publish(lidar)
                last_lidar = lidar
            # stamp with wall time (ROS clock == system time here) just before publishing
            cam.header.stamp = node.get_clock().now().to_msg()
            cam_pub.publish(cam)
            published += 1

            t_next += period
            delay = t_next - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                late += 1
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'published {published} camera frames ({late} behind schedule)')
        if args.stats_dir:
            os.makedirs(args.stats_dir, exist_ok=True)
            with open(os.path.join(args.stats_dir, 'published.json'), 'w') as f:
                json.dump({'camera': published, 'fps_target': args.fps, 'behind_schedule': late}, f, indent=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
