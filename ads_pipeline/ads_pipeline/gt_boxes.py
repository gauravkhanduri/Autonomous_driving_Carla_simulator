#ads_pipeline/gt_boxes.py
"""
2D ground-truth boxes from a recorded CARLA drive (tools/record_drive.py).

Per actor:
  1. project the 8 corners of its 3D box (cut at the camera near plane) -> loose image box
  2. keep the pixels inside the loose box whose semantic tag is one of the actor's tags and whose
     depth lies within the actor's depth range -> the final box fits only what the camera sees,
     so partly occluded actors get a box around their visible part and hidden actors get none.

Frames: CARLA world/actor (x forward, y right, z up, left-handed), camera optical (x right, y down,
z forward). Depth images hold planar depth (optical z) in centimetres.
"""
import json
import os

import cv2
import numpy as np

NEAR = 0.1               # m, near clipping plane
MIN_BOX = 8              # px, boxes narrower or shorter than this are dropped
MIN_PIXELS = 10          # visible pixels an actor needs to get a box
DEPTH_MARGIN = 0.3       # m, tolerance around the actor's depth range
WALKER_WIDEN = 0.3       # m, added to a pedestrian box's length and width
CLASSES = ('vehicle', 'pedestrian')

# rotation from CARLA axes (x forward, y right, z up) to the optical frame (x right, y down, z forward)
R_OPT_FROM_CARLA = np.array([[0.0, 1.0, 0.0],
                             [0.0, 0.0, -1.0],
                             [1.0, 0.0, 0.0]])

# 12 edges of a box given as 8 corners in CARLA's get_world_vertices order
# (index bits: 4 = +x, 2 = +y, 1 = +z)
BOX_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7),
             (0, 2), (1, 3), (4, 6), (5, 7),
             (0, 4), (1, 5), (2, 6), (3, 7)]


def carla_matrix(location, rotation):
    """4x4 local->world matrix of a CARLA transform (location xyz, rotation pitch/yaw/roll in degrees),
    identical to carla.Transform.get_matrix()."""
    pitch, yaw, roll = np.radians(rotation)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cr, sr = np.cos(roll), np.sin(roll)
    M = np.eye(4)
    M[:3, :3] = [[cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
                 [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
                 [sp, -cp * sr, cp * cr]]
    M[:3, 3] = location
    return M


def world_to_optical(camera_pose):
    """4x4 transform from CARLA world coordinates to the camera optical frame."""
    T_world_cam = carla_matrix(camera_pose['location'], camera_pose['rotation'])
    T = np.eye(4)
    T[:3, :3] = R_OPT_FROM_CARLA
    return T @ np.linalg.inv(T_world_cam)


def project_points(points_world, T_opt_world, K):
    """Project (N,3) world points. Returns (N,2) pixel coordinates and (N,) optical depth for every
    point; points behind the camera get meaningless pixels and z <= 0, so callers must check z."""
    pts = np.c_[np.asarray(points_world, dtype=np.float64), np.ones(len(points_world))]
    cam = (T_opt_world @ pts.T)[:3]
    z = cam[2]
    with np.errstate(divide='ignore', invalid='ignore'):
        uv = (np.asarray(K) @ cam)[:2] / z
    return uv.T, z


def clip_near(corners_opt, near=NEAR):
    """Cut a box (8 corners in the optical frame) at the plane z = near.
    Returns the corners in front of the plane plus the points where box edges cross it."""
    corners_opt = np.asarray(corners_opt, dtype=np.float64)
    z = corners_opt[:, 2]
    out = [c for c in corners_opt if c[2] >= near]
    for i, j in BOX_EDGES:
        if (z[i] - near) * (z[j] - near) < 0:
            t = (near - z[i]) / (z[j] - z[i])
            out.append(corners_opt[i] + t * (corners_opt[j] - corners_opt[i]))
    return np.array(out).reshape(-1, 3)


def widen_walker(corners_world, actor_pose, widen=WALKER_WIDEN):
    """Grow a pedestrian box by `widen` metres in length and width (not height), about its centre.
    CARLA walker boxes are tighter than the rendered body, especially arms and bags."""
    T = carla_matrix(actor_pose['location'], actor_pose['rotation'])
    pts = np.c_[np.asarray(corners_world, dtype=np.float64), np.ones(8)]
    local = (np.linalg.inv(T) @ pts.T)[:3].T
    centre = local.mean(axis=0)
    sign = np.sign(local - centre)
    local[:, :2] += sign[:, :2] * (widen / 2.0)
    return (T @ np.c_[local, np.ones(8)].T)[:3].T


def box_from_corners(uv, z, width, height, near=NEAR, min_size=MIN_BOX):
    """Image box [x1, y1, x2, y2] enclosing the projected points, clipped to the image.
    None if a point is behind the near plane, the box misses the image, or it is under min_size px."""
    uv = np.asarray(uv, dtype=np.float64)
    if len(uv) == 0 or np.any(np.asarray(z) < near - 1e-9):
        return None
    x1, y1 = uv.min(axis=0)
    x2, y2 = uv.max(axis=0)
    x1, x2 = max(x1, 0.0), min(x2, float(width))
    y1, y2 = max(y1, 0.0), min(y2, float(height))
    if x2 - x1 < min_size or y2 - y1 < min_size:
        return None
    return [x1, y1, x2, y2]


def actor_pixels(box, z_corners, depth, semantic, tags, margin=DEPTH_MARGIN):
    """Boolean mask (H, W) of the pixels inside `box` that show the actor: semantic tag in `tags`
    and depth within [min(z_corners) - margin, max(z_corners) + margin]."""
    h, w = semantic.shape
    x1, y1 = int(np.floor(box[0])), int(np.floor(box[1]))
    x2, y2 = int(np.ceil(box[2])), int(np.ceil(box[3]))
    x1, y1, x2, y2 = max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)
    mask = np.zeros((h, w), dtype=bool)
    if x2 <= x1 or y2 <= y1:
        return mask
    zmin = max(float(np.min(z_corners)), 0.0) - margin
    zmax = float(np.max(z_corners)) + margin
    d = depth[y1:y2, x1:x2]
    mask[y1:y2, x1:x2] = np.isin(semantic[y1:y2, x1:x2], list(tags)) & (d >= zmin) & (d <= zmax)
    return mask


def tight_box(mask, min_size=MIN_BOX, min_pixels=MIN_PIXELS):
    """[x1, y1, x2, y2] around the True pixels of mask (pixel edges), or None if too small."""
    if mask.sum() < min_pixels:
        return None
    ys, xs = np.nonzero(mask)
    box = [float(xs.min()), float(ys.min()), float(xs.max() + 1), float(ys.max() + 1)]
    if box[2] - box[0] < min_size or box[3] - box[1] < min_size:
        return None
    return box


def ground_truth(frame_meta, K, width, height, depth_m, semantic):
    """2D ground truth for one frame.

    Returns a list of dicts: id, class, box (visible part), loose (projected 3D box), pixels
    (visible pixel count) and visible (pixels / loose box area).
    """
    T = world_to_optical(frame_meta['camera'])
    out = []
    for a in frame_meta['actors']:
        corners = np.asarray(a['corners'], dtype=np.float64)
        if a['class'] == 'pedestrian':
            corners = widen_walker(corners, a['transform'])
        cam = (T @ np.c_[corners, np.ones(8)].T)[:3].T
        if cam[:, 2].max() < NEAR:
            continue                                # entirely behind the camera
        z_corners = cam[:, 2]
        pts = clip_near(cam)
        uv, z = project_points(pts, np.eye(4), K)   # points are already in the optical frame
        loose = box_from_corners(uv, z, width, height)
        if loose is None:
            continue
        mask = actor_pixels(loose, z_corners, depth_m, semantic, a['semantic_tags'])
        box = tight_box(mask)
        if box is None:
            continue
        area = (loose[2] - loose[0]) * (loose[3] - loose[1])
        n = int(mask.sum())
        out.append({'id': a['id'], 'class': a['class'], 'box': box, 'loose': loose,
                    'pixels': n, 'visible': n / area if area > 0 else 0.0})
    return out


def load_drive(rec_dir):
    """drive.json and the list of per-frame metadata of a recording."""
    with open(os.path.join(rec_dir, 'drive.json')) as f:
        drive = json.load(f)
    with open(os.path.join(rec_dir, 'meta.jsonl')) as f:
        meta = [json.loads(line) for line in f]
    return drive, meta


def load_frame(rec_dir, index):
    """(bgr, depth in metres, semantic tags) for one frame of a recording."""
    name = f'{index:06d}.png'
    bgr = cv2.imread(os.path.join(rec_dir, 'rgb', name), cv2.IMREAD_COLOR)
    depth = cv2.imread(os.path.join(rec_dir, 'depth', name), cv2.IMREAD_UNCHANGED).astype(np.float32) / 100.0
    semantic = cv2.imread(os.path.join(rec_dir, 'semantic', name), cv2.IMREAD_UNCHANGED)
    if bgr is None or depth is None or semantic is None:
        raise FileNotFoundError(f'frame {index} incomplete in {rec_dir}')
    return bgr, depth, semantic
