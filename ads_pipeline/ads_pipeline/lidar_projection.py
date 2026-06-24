#ads_pipeline/lidar_projection.py

import numpy as np
import cv2


def build_intrinsic_matrix(image_w: int, image_h:int, fov_deg: float) -> np.ndarray:
    """
        Comoute the 3x3 pinhole intrinsic matrix K from image dimensions and horizonatal field 
        of view.
    """

    focal_length = image_w / (2 * np.tan(np.radians(fov_deg)/2.0))

    cx = image_w/2.0
    cy = image_h/2.0

    K = np.array([
        [focal_length, 0.0, cx],
        [0, focal_length, cy],
        [0, 0, 1],
    ],dtype=np.float64)

    return K

def build_extrinsic(lidar_loc: tuple, camera_loc: tuple) -> np.ndarray:
    """
    Build the 4x4 extrinsic transform from lidar to camera frame.
    both locations are (x,y,z) in CARLA vehicle coordinatte
    """
    #translation vector: camera position minus LiDAR position

    tx = camera_loc[0] - lidar_loc[0]
    ty = camera_loc[1] - lidar_loc[1]
    tz = camera_loc[2] - lidar_loc[2]

    # For Co-planner sensors with no rotation difference, R=I
    # Replace with actual position if sensors  are angled differently

    R = np.eye(3,dtype=np.float64)
    t = np.array([
        [tx], [ty], [tz]
    ], dtype=np.float64)

    T  = np.hstack([R,t]) # 3x4
    T = np.vstack([T,[0,0,0,1]]) # 4x4

    return T

def projection_lidar_to_image(points_xyz: np.ndarray, K: np.ndarray, T_cam_lidar: np.ndarray,
                              image_w: int, image_h: int,) -> tuple:

     """
     Returns: 
        pixels -- (M,2) array of (u,v) coodinates
        depths -- (M,) array of depth values for coloring
     """ 
     N = points_xyz.shape[0]
     ones = np.ones((N,1),dtype=np.float64)
     pts_hom = np.hstack([points_xyz,ones]).T # (4,N)

     # Transform to camera coordinate frame
     pts_cam = T_cam_lidar @ pts_hom # (4,N)
     
     #keep only points in from of the camera (position Z)
     in_front = pts_cam[2,:] > 0.1
     pts_cam = pts_cam[:,in_front]

     #project to image plane
     pts_proj = K @ pts_cam[:3,:] # (3,M)
     pts_proj /=pts_proj[2:3,:]   #normalize by Z
     
     u = pts_proj[0,:].astype(int)
     v = pts_proj[1,:].astype(int)
     depth = pts_cam[2,:]

     # keep only pixels only image bound

     valid = (u>=0) &(u< image_w) & (v>=0) & (v < image_h)
     return np.stack([u[valid], v[valid]], axis =1), depth[valid]

def colorize_depth(depth: np.ndarray, max_depth: float = 50.0) -> np.ndarray:
    """ map depth values to BGR colors using a jet colormap."""
    normalized = np.clip(depth / max_depth, 0.0, 1.0)
    normalized = (normalized * 255).astype(np.uint8)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    return colored.squeeze() # (M, 3)

def overlay_projection(image_bgr: np.ndarray,
                       pixels: np.ndarray,
                       colors: np.ndarray,
                       dot_size: int = 3) -> np.ndarray:
    """Draw colored LiDAR dots onto the image"""
    result = image_bgr.copy()
    for(u,v), color in zip(pixels,colors):
        cv2.circle(result,(u,v), dot_size, color.tolist(),-1)
    
    return result


        
