#!/usr/bin/env python3
"""
Record a CARLA drive for perception evaluation (standalone, non-ROS).

Runs CARLA in synchronous mode at a fixed rate so every RGB, depth and semantic image belongs to
the same simulation tick as the actor boxes saved with it. The cameras share the ego camera rig
used by sensor_manager (x=1.5, z=2.4, facing forward).

Output layout (<out>/):
    drive.json      static info: map, weather, rate, image size, fov, K, camera mount, seed
    meta.jsonl      one line per frame: frame id, sim time, camera/ego world pose, actors
                    (id, class, type_id, base_type, semantic tags, 8 world box corners)
    rgb/NNNNNN.png        8-bit BGR
    depth/NNNNNN.png      16-bit depth in centimetres (clipped at 655.35 m)
    semantic/NNNNNN.png   8-bit CARLA semantic tag per pixel

    python3 tools/record_drive.py --out recordings/clear --weather clear --frames 600
    python3 tools/record_drive.py --out recordings/night --weather night --frames 600
"""
import argparse
import json
import os
import queue
import random
import time
from concurrent.futures import ThreadPoolExecutor

import carla
import cv2
import numpy as np

CAMERA_LOC = (1.5, 0.0, 2.4)   # must match sensor_manager.py
WEATHER = {'clear': carla.WeatherParameters.ClearNoon, 'night': carla.WeatherParameters.ClearNight}
MAX_ACTOR_DIST = 120.0         # m, actors farther than this from the camera are not saved


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', required=True)
    ap.add_argument('--weather', choices=sorted(WEATHER), default='clear')
    ap.add_argument('--frames', type=int, default=600)
    ap.add_argument('--warmup', type=int, default=100, help='ticks before recording (traffic gets moving)')
    ap.add_argument('--town', default='Town01')
    ap.add_argument('--fps', type=float, default=20.0)
    ap.add_argument('--width', type=int, default=1920)
    ap.add_argument('--height', type=int, default=1080)
    ap.add_argument('--fov', type=float, default=90.0)
    ap.add_argument('--vehicles', type=int, default=80)
    ap.add_argument('--walkers', type=int, default=200)
    ap.add_argument('--near', type=float, default=120.0,
                    help='walkers spawn within this distance (m) of the ego; vehicles use the nearest spawn points')
    ap.add_argument('--cross-factor', type=float, default=0.6, help='share of walkers that cross roads')
    ap.add_argument('--red-lights', action='store_true',
                    help='keep normal traffic lights (default: all frozen green so the ego keeps moving)')
    ap.add_argument('--spawn-index', type=int, default=6)
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--tm-port', type=int, default=8000)
    return ap.parse_args()


def intrinsic(width, height, fov):
    f = width / (2.0 * np.tan(np.radians(fov) / 2.0))
    return [[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]]


def pose(t):
    return {'location': [t.location.x, t.location.y, t.location.z],
            'rotation': [t.rotation.pitch, t.rotation.yaw, t.rotation.roll]}


def actor_class(actor):
    if actor.type_id.startswith('walker.pedestrian'):
        return 'pedestrian'
    return 'vehicle'


def actor_record(actor):
    corners = actor.bounding_box.get_world_vertices(actor.get_transform())
    return {
        'id': actor.id,
        'class': actor_class(actor),
        'type_id': actor.type_id,
        'base_type': actor.attributes.get('base_type', ''),
        'semantic_tags': list(actor.semantic_tags),
        'corners': [[c.x, c.y, c.z] for c in corners],
        'transform': pose(actor.get_transform()),
    }


def decode_depth_cm(image):
    """CARLA depth image (BGRA, 24-bit normalised depth over 1000 m) -> uint16 centimetres."""
    a = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4).astype(np.float64)
    depth_m = (a[:, :, 2] + a[:, :, 1] * 256.0 + a[:, :, 0] * 65536.0) / (256.0 ** 3 - 1) * 1000.0
    return np.clip(depth_m * 100.0, 0, 65535).astype(np.uint16)


def spawn_traffic(client, world, tm, args, ego_spawn, rng):
    bp_lib = world.get_blueprint_library()
    # traffic is concentrated around the ego so the camera sees it, not spread over the whole town
    spawn_points = [sp for sp in world.get_map().get_spawn_points() if sp.location.distance(ego_spawn.location) > 5.0]
    spawn_points.sort(key=lambda sp: sp.location.distance(ego_spawn.location))

    vehicle_bps = [bp for bp in bp_lib.filter('vehicle.*') if int(bp.get_attribute('number_of_wheels')) == 4]
    batch = []
    for sp in spawn_points[:args.vehicles]:
        bp = rng.choice(vehicle_bps)
        if bp.has_attribute('color'):
            bp.set_attribute('color', rng.choice(bp.get_attribute('color').recommended_values))
        bp.set_attribute('role_name', 'autopilot')
        batch.append(carla.command.SpawnActor(bp, sp).then(
            carla.command.SetAutopilot(carla.command.FutureActor, True, tm.get_port())))
    vehicle_ids = [r.actor_id for r in client.apply_batch_sync(batch, True) if not r.error]

    walker_bps = bp_lib.filter('walker.pedestrian.*')
    batch = []
    locations = []
    for _ in range(args.walkers * 30):
        loc = world.get_random_location_from_navigation()
        if loc is not None and loc.distance(ego_spawn.location) < args.near:
            locations.append(loc)
            if len(locations) == args.walkers:
                break
    for loc in locations:
        loc.z += 1.0  # spawn slightly above the navmesh to avoid ground collisions
        bp = rng.choice(walker_bps)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')
        batch.append(carla.command.SpawnActor(bp, carla.Transform(loc)))
    walker_ids = [r.actor_id for r in client.apply_batch_sync(batch, True) if not r.error]

    ctrl_bp = bp_lib.find('controller.ai.walker')
    batch = [carla.command.SpawnActor(ctrl_bp, carla.Transform(), wid) for wid in walker_ids]
    controller_ids = [r.actor_id for r in client.apply_batch_sync(batch, True) if not r.error]
    world.tick()

    world.set_pedestrians_cross_factor(args.cross_factor)
    for ctrl in world.get_actors(controller_ids):
        ctrl.start()
        ctrl.go_to_location(world.get_random_location_from_navigation())
        ctrl.set_max_speed(1.0 + rng.random() * 0.8)
    return vehicle_ids, walker_ids, controller_ids


def main():
    args = parse_args()
    rng = random.Random(args.seed)
    out = os.path.expanduser(args.out)
    for sub in ('rgb', 'depth', 'semantic'):
        os.makedirs(os.path.join(out, sub), exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    world = client.get_world()
    if not world.get_map().name.endswith(args.town):
        print(f'loading {args.town} ...')
        world = client.load_world(args.town)
    original_settings = world.get_settings()

    tm = client.get_trafficmanager(args.tm_port)
    actors, vehicle_ids, walker_ids, controller_ids = [], [], [], []
    writer = ThreadPoolExecutor(max_workers=6)
    pending = []
    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0 / args.fps
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(args.seed)
        world.set_pedestrians_seed(args.seed)
        world.set_weather(WEATHER[args.weather])
        lights = world.get_actors().filter('traffic.traffic_light')
        if not args.red_lights:
            for tl in lights:
                tl.set_state(carla.TrafficLightState.Green)
                tl.freeze(True)

        bp_lib = world.get_blueprint_library()
        ego_spawn = world.get_map().get_spawn_points()[args.spawn_index]
        ego = world.spawn_actor(bp_lib.find('vehicle.tesla.model3'), ego_spawn)
        actors.append(ego)
        ego.set_autopilot(True, tm.get_port())

        vehicle_ids, walker_ids, controller_ids = spawn_traffic(client, world, tm, args, ego_spawn, rng)
        print(f'spawned {len(vehicle_ids)} vehicles, {len(walker_ids)} walkers')
        for v in world.get_actors(vehicle_ids + [ego.id]):
            tm.update_vehicle_lights(v, True)  # headlights follow the time of day

        cam_tf = carla.Transform(carla.Location(*CAMERA_LOC))
        queues = {}
        for name, bp_name in (('rgb', 'sensor.camera.rgb'), ('depth', 'sensor.camera.depth'),
                              ('semantic', 'sensor.camera.semantic_segmentation')):
            bp = bp_lib.find(bp_name)
            bp.set_attribute('image_size_x', str(args.width))
            bp.set_attribute('image_size_y', str(args.height))
            bp.set_attribute('fov', str(args.fov))
            sensor = world.spawn_actor(bp, cam_tf, attach_to=ego)
            q = queue.Queue()
            sensor.listen(q.put)
            queues[name] = q
            actors.append(sensor)

        with open(os.path.join(out, 'drive.json'), 'w') as f:
            json.dump({
                'map': world.get_map().name, 'weather': args.weather, 'fps': args.fps,
                'width': args.width, 'height': args.height, 'fov': args.fov,
                'K': intrinsic(args.width, args.height, args.fov),
                'camera_mount': {'location': list(CAMERA_LOC), 'rotation': [0.0, 0.0, 0.0]},
                'depth_encoding': 'uint16 centimetres, clipped at 655.35 m',
                'seed': args.seed, 'spawn_index': args.spawn_index, 'traffic_lights': 'normal' if args.red_lights else 'frozen green',
                'vehicles': len(vehicle_ids), 'walkers': len(walker_ids), 'frames': args.frames,
            }, f, indent=2)

        meta = open(os.path.join(out, 'meta.jsonl'), 'w')
        t0 = time.time()
        for i in range(args.warmup + args.frames):
            frame = world.tick()
            images = {}
            for name, q in queues.items():
                img = q.get(timeout=10.0)
                while img.frame < frame:        # discard anything left from an earlier tick
                    img = q.get(timeout=10.0)
                images[name] = img
            if i < args.warmup:
                continue

            n = i - args.warmup
            rgb = np.frombuffer(images['rgb'].raw_data, dtype=np.uint8).reshape(args.height, args.width, 4)[:, :, :3].copy()
            sem = np.frombuffer(images['semantic'].raw_data, dtype=np.uint8).reshape(args.height, args.width, 4)[:, :, 2].copy()
            depth = decode_depth_cm(images['depth'])
            name = f'{n:06d}.png'
            pending += [writer.submit(cv2.imwrite, os.path.join(out, 'rgb', name), rgb),
                        writer.submit(cv2.imwrite, os.path.join(out, 'depth', name), depth),
                        writer.submit(cv2.imwrite, os.path.join(out, 'semantic', name), sem)]

            cam_world = images['rgb'].transform
            cam_loc = cam_world.location
            records = [actor_record(a) for a in world.get_actors().filter('vehicle.*')
                       if a.id != ego.id and a.get_location().distance(cam_loc) < MAX_ACTOR_DIST]
            records += [actor_record(a) for a in world.get_actors().filter('walker.pedestrian.*')
                        if a.get_location().distance(cam_loc) < MAX_ACTOR_DIST]
            meta.write(json.dumps({
                'index': n, 'frame': frame, 'sim_time': images['rgb'].timestamp,
                'camera': pose(cam_world), 'ego': pose(ego.get_transform()),
                'ego_speed': ego.get_velocity().length(), 'actors': records,
            }) + '\n')

            # keep the write queue bounded so memory stays flat
            if len(pending) > 60:
                for p in pending[:30]:
                    if not p.result():
                        raise RuntimeError('image write failed')
                pending = pending[30:]
            if n % 50 == 0:
                print(f'frame {n}/{args.frames}  {len(records)} actors  {time.time() - t0:.0f}s', flush=True)
        meta.close()
        for p in pending:
            if not p.result():
                raise RuntimeError('image write failed')
        print(f'recorded {args.frames} frames to {out}')
    finally:
        writer.shutdown(wait=True)
        for a in actors:
            if isinstance(a, carla.Sensor) and a.is_listening:
                a.stop()
        for ctrl in world.get_actors(controller_ids):
            ctrl.stop()
        # leave synchronous mode before destroying anything: in sync mode a destroy waits for a
        # tick that no client will send, and the server would stay paused for other clients
        tm.set_synchronous_mode(False)
        world.apply_settings(original_settings)
        world.freeze_all_traffic_lights(False)
        client.apply_batch_sync([carla.command.SetAutopilot(i, False, tm.get_port()) for i in vehicle_ids])
        if actors and actors[0].is_alive:
            actors[0].set_autopilot(False, tm.get_port())
        results = client.apply_batch_sync([carla.command.DestroyActor(i) for i in
                                           controller_ids + walker_ids + vehicle_ids + [a.id for a in reversed(actors)]])
        errors = [r.error for r in results if r.error]
        if errors:
            print(f'warning: {len(errors)} actors not destroyed, e.g. {errors[0]}')

if __name__ == '__main__':
    main()
