import math

import numpy as np
import pytest

carla = pytest.importorskip('carla')

from ads_pipeline.multisensor_kf import nearest_lidar_target, nearest_radar_target  # noqa: E402


def test_lidar_picks_nearest_obstacle_in_cone():
    points = np.array([
        [20.0, 0.0, -1.0, 1.0],    # obstacle 20 m ahead, 1.5 m above ground
        [10.0, 1.0, -1.5, 1.0],    # obstacle 10 m ahead
        [5.0, 0.0, -2.45, 1.0],    # road return (too low)
        [1.0, 0.0, -1.0, 1.0],     # ego vehicle body (too close)
        [8.0, 8.0, -1.0, 1.0],     # outside the cone
        [-6.0, 0.0, -1.0, 1.0],    # behind
    ], dtype=np.float32)
    assert np.allclose(nearest_lidar_target(points), [10.0, 1.0])


def test_lidar_returns_none_without_obstacles():
    points = np.array([[5.0, 0.0, -2.45, 1.0]], dtype=np.float32)
    assert nearest_lidar_target(points) is None


def test_radar_converts_nearest_detection_to_vehicle_frame():
    detections = [[12.0, 0.0, 0.0], [8.0, math.radians(10), 0.0], [3.0, math.radians(40), 0.0]]
    x, y = nearest_radar_target(detections)
    assert np.isclose(x, 2.0 + 8.0 * math.cos(math.radians(10)))
    assert np.isclose(y, 8.0 * math.sin(math.radians(10)))


def test_radar_handles_empty_frame():
    assert nearest_radar_target([]) is None
