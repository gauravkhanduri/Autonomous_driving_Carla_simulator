import numpy as np
import pytest

from ads_pipeline.gt_boxes import (box_from_corners, carla_matrix, clip_near, ground_truth, project_points,
                                   tight_box, actor_pixels, widen_walker, world_to_optical)

W, H = 1920, 1080
F = W / 2.0  # 90 deg horizontal FOV
K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1]])
CAM_AT_ORIGIN = {'location': [0.0, 0.0, 0.0], 'rotation': [0.0, 0.0, 0.0]}


def box_corners(center, extent, yaw=0.0):
    """8 world corners in CARLA get_world_vertices order (index bits 4=+x, 2=+y, 1=+z)."""
    T = carla_matrix(center, [0.0, yaw, 0.0])
    local = np.array([[sx * extent[0], sy * extent[1], sz * extent[2]]
                      for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return (T @ np.c_[local, np.ones(8)].T)[:3].T


def test_carla_matrix_matches_carla_api():
    carla = pytest.importorskip('carla')
    t = carla.Transform(carla.Location(3.0, -2.0, 1.5), carla.Rotation(pitch=10.0, yaw=-35.0, roll=5.0))
    assert np.allclose(np.array(t.get_matrix()), carla_matrix([3.0, -2.0, 1.5], [10.0, -35.0, 5.0]), atol=1e-5)


def test_point_ahead_projects_to_principal_point():
    T = world_to_optical(CAM_AT_ORIGIN)
    uv, z = project_points([[10.0, 0.0, 0.0]], T, K)
    assert np.allclose(uv[0], [W / 2, H / 2]) and np.isclose(z[0], 10.0)


def test_axes_right_is_plus_u_and_up_is_minus_v():
    T = world_to_optical(CAM_AT_ORIGIN)
    uv, _ = project_points([[10.0, 1.0, 0.0], [10.0, 0.0, 1.0]], T, K)
    assert uv[0, 0] > W / 2          # CARLA +y (right) -> image right
    assert uv[1, 1] < H / 2          # CARLA +z (up)    -> image up


def test_camera_yaw_and_translation():
    # camera at (5, 5, 2) looking along +y (yaw 90): a point 10 m further along +y is dead ahead
    T = world_to_optical({'location': [5.0, 5.0, 2.0], 'rotation': [0.0, 90.0, 0.0]})
    uv, z = project_points([[5.0, 15.0, 2.0]], T, K)
    assert np.allclose(uv[0], [W / 2, H / 2], atol=1e-6) and np.isclose(z[0], 10.0)


def test_point_behind_camera_has_negative_depth():
    _, z = project_points([[-5.0, 0.0, 0.0]], world_to_optical(CAM_AT_ORIGIN), K)
    assert z[0] < 0


def test_box_from_corners_clips_to_image():
    box = box_from_corners(np.array([[-50.0, 100.0], [300.0, 2000.0]]), np.array([5.0, 5.0]), W, H)
    assert box == [0.0, 100.0, 300.0, float(H)]


@pytest.mark.parametrize('uv, z', [
    ([[100.0, 100.0], [200.0, 200.0]], [5.0, 0.05]),     # a point behind the near plane
    ([[-300.0, 100.0], [-100.0, 200.0]], [5.0, 5.0]),    # entirely left of the image
    ([[100.0, 100.0], [105.0, 200.0]], [5.0, 5.0]),      # 5 px wide
])
def test_box_from_corners_rejects(uv, z):
    assert box_from_corners(np.array(uv), np.array(z), W, H) is None


def test_clip_near_keeps_front_corners_and_adds_crossings():
    corners = box_corners([0.0, 0.0, 0.0], [2.0, 1.0, 1.0])        # straddles the camera plane
    cam = (world_to_optical(CAM_AT_ORIGIN) @ np.c_[corners, np.ones(8)].T)[:3].T
    clipped = clip_near(cam, near=0.1)
    assert np.all(clipped[:, 2] >= 0.1 - 1e-9)
    assert len(clipped) == 4 + 4                                  # 4 front corners + 4 crossing edges
    assert np.sum(np.isclose(clipped[:, 2], 0.1)) == 4


def test_widen_walker_grows_horizontally_only():
    pose = {'location': [10.0, 3.0, 1.0], 'rotation': [0.0, 30.0, 0.0]}
    corners = box_corners([10.0, 3.0, 1.0], [0.2, 0.2, 0.9], yaw=30.0)
    wide = widen_walker(corners, pose, widen=0.3)
    local = (np.linalg.inv(carla_matrix(pose['location'], pose['rotation'])) @ np.c_[wide, np.ones(8)].T)[:3].T
    assert np.allclose(np.ptp(local, axis=0), [0.4 + 0.3, 0.4 + 0.3, 1.8])


def test_actor_pixels_needs_matching_tag_and_depth():
    sem = np.zeros((H, W), np.uint8)
    depth = np.full((H, W), 100.0, np.float32)
    sem[500:600, 900:1000] = 14
    depth[500:600, 900:1000] = 10.0
    sem[500:600, 1000:1100] = 14                       # same class, much farther: another car
    depth[500:600, 1000:1100] = 40.0
    mask = actor_pixels([850, 450, 1150, 650], [9.0, 11.0], depth, sem, [14])
    assert mask.sum() == 100 * 100
    assert tight_box(mask) == [900.0, 500.0, 1000.0, 600.0]


def test_tight_box_rejects_tiny_masks():
    mask = np.zeros((H, W), bool)
    mask[10:14, 10:30] = True                          # 4 px tall
    assert tight_box(mask) is None


def _scene(actors):
    return {'camera': CAM_AT_ORIGIN, 'actors': actors}


def _actor(i, cls, center, extent, tag):
    corners = box_corners(center, extent)
    return {'id': i, 'class': cls, 'semantic_tags': [tag], 'corners': corners.tolist(),
            'transform': {'location': list(center), 'rotation': [0.0, 0.0, 0.0]}}


def _render(actors, wall=None):
    """Synthetic depth/semantic: each actor fills its projected box with its tag and centre depth,
    nearer actors drawn last; an optional opaque wall (tag 3) at x = wall[0] covering wall[1] columns."""
    sem = np.zeros((H, W), np.uint8)
    depth = np.full((H, W), 1000.0, np.float32)
    T = world_to_optical(CAM_AT_ORIGIN)
    for a in sorted(actors, key=lambda a: -a['transform']['location'][0]):
        uv, z = project_points(np.array(a['corners']), T, K)
        x1, y1 = np.clip(uv.min(0), 0, [W, H]).astype(int)
        x2, y2 = np.clip(uv.max(0), 0, [W, H]).astype(int)
        sem[y1:y2, x1:x2] = a['semantic_tags'][0]
        depth[y1:y2, x1:x2] = a['transform']['location'][0]
    if wall is not None:
        sem[:, wall[1][0]:wall[1][1]] = 3
        depth[:, wall[1][0]:wall[1][1]] = wall[0]
    return depth, sem


def test_ground_truth_visible_occluded_and_hidden():
    car = _actor(1, 'vehicle', [20.0, 0.0, 0.0], [2.0, 1.0, 0.8], 14)
    ped_left = _actor(2, 'pedestrian', [15.0, -4.0, 0.0], [0.25, 0.25, 0.9], 12)
    ped_behind_wall = _actor(3, 'pedestrian', [30.0, 6.0, 0.0], [0.25, 0.25, 0.9], 12)
    actors = [car, ped_left, ped_behind_wall]
    # wall 8 m away covering the right part of the image hides pedestrian 3
    depth, sem = _render(actors, wall=(8.0, (1100, W)))   # pedestrian 3 projects to u ~ 1150
    gt = {g['id']: g for g in ground_truth(_scene(actors), K, W, H, depth, sem)}
    assert set(gt) == {1, 2}
    assert gt[1]['class'] == 'vehicle' and gt[2]['class'] == 'pedestrian'
    # the visible box sits inside the loose (projected) box, up to whole-pixel rounding
    for g in gt.values():
        assert g['loose'][0] - 1 <= g['box'][0] and g['box'][2] <= g['loose'][2] + 1


def test_ground_truth_partial_occlusion_shrinks_box():
    car = _actor(1, 'vehicle', [20.0, 0.0, 0.0], [2.0, 1.0, 0.8], 14)
    depth, sem = _render([car])
    full = ground_truth(_scene([car]), K, W, H, depth, sem)[0]['box']
    depth, sem = _render([car], wall=(8.0, (int((full[0] + full[2]) / 2), W)))   # hide right half
    half = ground_truth(_scene([car]), K, W, H, depth, sem)[0]['box']
    assert half[0] == full[0] and half[2] < full[2]


def test_ground_truth_skips_actor_behind_camera():
    car = _actor(1, 'vehicle', [-10.0, 0.0, 0.0], [2.0, 1.0, 0.8], 14)
    depth, sem = _render([])
    assert ground_truth(_scene([car]), K, W, H, depth, sem) == []
