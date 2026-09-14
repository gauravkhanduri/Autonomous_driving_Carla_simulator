import numpy as np

from ads_pipeline.lidar_projection import (build_extrinsic, build_intrinsic_matrix, colorize_depth,
                                           projection_lidar_to_image)

LIDAR_LOC = (0.0, 0.0, 2.5)
CAMERA_LOC = (1.5, 0.0, 2.4)
W, H = 1920, 1080


def _project(points):
    K = build_intrinsic_matrix(W, H, 90.0)
    T = build_extrinsic(LIDAR_LOC, CAMERA_LOC)
    return projection_lidar_to_image(np.asarray(points, dtype=np.float32), K, T, W, H)


def test_intrinsic_focal_length_for_90_deg_fov():
    K = build_intrinsic_matrix(W, H, 90.0)
    assert np.isclose(K[0, 0], W / 2)
    assert (K[0, 2], K[1, 2]) == (W / 2, H / 2)


def test_point_on_camera_axis_projects_to_image_center():
    # 10 m ahead of the camera, at camera height, expressed in the LiDAR frame
    pixels, depths = _project([[11.5, 0.0, -0.1]])
    assert pixels.tolist() == [[960, 540]]
    assert np.isclose(depths[0], 10.0)


def test_left_and_up_map_to_smaller_u_and_v():
    # ROS frame: +y is left, +z is up
    pixels, _ = _project([[11.5, 2.0, -0.1], [11.5, 0.0, 1.9]])
    assert pixels.tolist() == [[768, 540], [960, 348]]


def test_points_behind_camera_are_dropped():
    pixels, depths = _project([[-10.0, 0.0, -0.1]])
    assert len(pixels) == 0 and len(depths) == 0


def test_colorize_depth_near_is_blue_far_is_red():
    colors = colorize_depth(np.array([0.0, 50.0]), max_depth=50.0)  # BGR
    assert colors.shape == (2, 3)
    assert colors[0, 0] > colors[0, 2]
    assert colors[1, 2] > colors[1, 0]
