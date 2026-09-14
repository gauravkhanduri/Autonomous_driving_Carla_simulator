#ads_pipeline/lidar_projection.py

import numpy as np
import cv2


# Rotation from a ROS body frame (x forward, y left, z up) to the
# OpenCV camera optical frame (x right, y down, z forward).
R_OPTICAL_FROM_BODY = np.array([
    [0.0, -1.0, 0.0],
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
], dtype=np.float64)


def build_intrinsic_matrix(image_w: int, image_h: int, fov_deg: float) -> np.ndarray:
    """
        Compute the 3x3 pinhole intrinsic matrix K from image dimensions and horizontal field
        of view.
    """

    focal_length = image_w / (2 * np.tan(np.radians(fov_deg) / 2.0))

    cx = image_w / 2.0
    cy = image_h / 2.0

    K = np.array([
        [focal_length, 0.0, cx],
        [0, focal_length, cy],
        [0, 0, 1],
    ], dtype=np.float64)

    return K


def build_extrinsic(lidar_loc: tuple, camera_loc: tuple) -> np.ndarray:
    """
    Build the 4x4 transform T_cam_lidar that maps points from the LiDAR frame
    (ROS convention: x forward, y left, z up) into the camera optical frame
    (x right, y down, z forward).

    Both locations are (x, y, z) mount positions in CARLA vehicle coordinates (y right).
    Assumes both sensors face forward with no rotation offset.
    """
    # LiDAR origin relative to the camera, in the ROS body frame (CARLA y flipped)
    t_body = np.array([
        lidar_loc[0] - camera_loc[0],
        -(lidar_loc[1] - camera_loc[1]),
        lidar_loc[2] - camera_loc[2],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_OPTICAL_FROM_BODY
    T[:3, 3] = R_OPTICAL_FROM_BODY @ t_body

    return T


def projection_lidar_to_image(points_xyz: np.ndarray, K: np.ndarray, T_cam_lidar: np.ndarray,
                              image_w: int, image_h: int) -> tuple:
    """
    Project (N,3) LiDAR points onto the image plane.

    Returns:
        pixels -- (M,2) array of (u,v) coordinates
        depths -- (M,) array of depth values for coloring
    """
    N = points_xyz.shape[0]
    ones = np.ones((N, 1), dtype=np.float64)
    pts_hom = np.hstack([points_xyz.astype(np.float64), ones]).T  # (4,N)

    # Transform to camera coordinate frame
    pts_cam = T_cam_lidar @ pts_hom  # (4,N)

    #keep only points in front of the camera (positive Z)
    in_front = pts_cam[2, :] > 0.1
    pts_cam = pts_cam[:, in_front]

    #project to image plane
    pts_proj = K @ pts_cam[:3, :]  # (3,M)
    pts_proj /= pts_proj[2:3, :]   #normalize by Z

    u = np.round(pts_proj[0, :]).astype(int)
    v = np.round(pts_proj[1, :]).astype(int)
    depth = pts_cam[2, :]

    # keep only pixels inside the image bounds
    valid = (u >= 0) & (u < image_w) & (v >= 0) & (v < image_h)
    return np.stack([u[valid], v[valid]], axis=1), depth[valid]


def colorize_depth(depth: np.ndarray, max_depth: float = 50.0) -> np.ndarray:
    """Map depth values to BGR colors using the JET colormap (blue = near, red = far)."""
    normalized = np.clip(depth / max_depth, 0.0, 1.0)
    normalized = (normalized * 255).astype(np.uint8)
    normalized = normalized.reshape(-1, 1)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    return colored.reshape(-1, 3)  # (M, 3)


def overlay_projection(image_bgr: np.ndarray,
                       pixels: np.ndarray,
                       colors: np.ndarray,
                       dot_size: int = 3) -> np.ndarray:
    """Draw colored LiDAR dots onto the image"""
    result = image_bgr.copy()
    for (u, v), color in zip(pixels, colors):
        cv2.circle(result, (int(u), int(v)), dot_size, color.tolist(), -1)

    return result
