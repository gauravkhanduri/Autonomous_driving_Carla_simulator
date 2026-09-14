"""
Standalone (non-ROS) CARLA script: spawns an ego vehicle and a second vehicle, attaches a camera,
LiDAR and radar to the ego vehicle, and fuses the nearest LiDAR and radar targets ahead
with a constant-velocity Kalman filter.

All positions are in CARLA ego-vehicle coordinates (x forward, y right, z up).
"""
import argparse
import math
import threading
import time

import carla
import cv2
import numpy as np

try:
    from ads_pipeline.kalman_filter import KalmanFilter
except ImportError:  # running the file directly from this directory
    from kalman_filter import KalmanFilter


LIDAR_LOC = (0.0, 0.0, 2.5)
RADAR_LOC = (2.0, 0.0, 1.0)
CAMERA_LOC = (1.5, 0.0, 2.4)

CONE_HALF_ANGLE = np.radians(15)   # forward search cone
MIN_HEIGHT_ABOVE_GROUND = 0.3      # m, rejects road returns
MIN_RANGE = 3.0                    # m, rejects returns from the ego vehicle itself
RADAR_VELOCITY_RANGE = 7.5         # m/s, for debug-point coloring

R_LIDAR = np.eye(2) * 0.1**2
R_RADAR = np.eye(2) * 0.5**2


def nearest_lidar_target(points, lidar_loc=LIDAR_LOC):
    """Nearest (x, y) vehicle-frame LiDAR return inside the forward cone, or None.

    points -- (N,3+) raw CARLA LiDAR points in the sensor frame
    """
    xyz = points[:, :3] + np.asarray(lidar_loc, dtype=np.float32)
    forward = xyz[xyz[:, 0] > 0]
    angles = np.abs(np.arctan2(forward[:, 1], forward[:, 0]))
    dist = np.linalg.norm(forward[:, :2], axis=1)
    mask = (angles < CONE_HALF_ANGLE) & (forward[:, 2] > MIN_HEIGHT_ABOVE_GROUND) & (dist > MIN_RANGE)
    if not np.any(mask):
        return None
    cone = forward[mask]
    return cone[np.argmin(dist[mask]), :2].astype(float)


def nearest_radar_target(detections, radar_loc=RADAR_LOC):
    """Nearest (x, y) vehicle-frame radar detection inside the forward cone, or None.

    detections -- (N,3) array of [depth, azimuth, altitude] (radians)
    """
    if len(detections) == 0:
        return None
    detections = np.asarray(detections, dtype=float)
    depth, azimuth, altitude = detections[:, 0], detections[:, 1], detections[:, 2]
    mask = np.abs(azimuth) < CONE_HALF_ANGLE
    if not np.any(mask):
        return None
    i = np.flatnonzero(mask)[np.argmin(depth[mask])]
    ground_range = depth[i] * math.cos(altitude[i])
    return np.array([radar_loc[0] + ground_range * math.cos(azimuth[i]),
                     radar_loc[1] + ground_range * math.sin(azimuth[i])])


def clamp(min_v, max_v, value):
    return max(min_v, min(value, max_v))


def radar_color(velocity):
    norm_velocity = velocity / RADAR_VELOCITY_RANGE  # range [-1, 1]
    r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
    g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
    b = int(abs(clamp(-1.0, 0.0, -1.0 - norm_velocity)) * 255.0)
    return carla.Color(r, g, b)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--town', default='Town03', help="map to load ('' keeps the current map)")
    parser.add_argument('--no-display', action='store_true', help='do not open an OpenCV window')
    parser.add_argument('--duration', type=float, default=0.0, help='stop after N seconds (0 = run until q/Ctrl+C)')
    return parser.parse_args()


def main():
    args = parse_args()

    #connect with carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)

    world = client.load_world(args.town) if args.town else client.get_world()
    world.set_weather(carla.WeatherParameters.HardRainNoon)
    bp_lib = world.get_blueprint_library()
    time.sleep(2.0)

    actors = []
    lock = threading.Lock()
    state = {'frame': None, 'lidar': None, 'radar': None}

    try:
        spawn_points = world.get_map().get_spawn_points()
        vehicle_bp = bp_lib.find('vehicle.tesla.model3')

        #ego vehicle
        vehicle1 = world.spawn_actor(vehicle_bp, spawn_points[0])
        vehicle1.set_autopilot(True)
        actors.append(vehicle1)
        print(f"Spawned ego vehicle: {vehicle1.type_id} at {spawn_points[0].location}")

        #second vehicle
        vehicle2 = world.spawn_actor(vehicle_bp, spawn_points[1])
        vehicle2.set_autopilot(True)
        actors.append(vehicle2)
        print(f"Spawned second vehicle: {vehicle2.type_id} at {spawn_points[1].location}")

        camera_bp = bp_lib.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1280')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '90')
        camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(*CAMERA_LOC)), attach_to=vehicle1)
        actors.append(camera)

        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '64')
        lidar_bp.set_attribute('range', '100.0')
        lidar_bp.set_attribute('points_per_second', '1200000')
        lidar_bp.set_attribute('rotation_frequency', '20')
        lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(*LIDAR_LOC)), attach_to=vehicle1)
        actors.append(lidar)

        radar_bp = bp_lib.find('sensor.other.radar')
        radar_bp.set_attribute('horizontal_fov', '35')
        radar_bp.set_attribute('vertical_fov', '20')
        radar_bp.set_attribute('range', '20')
        radar = world.spawn_actor(radar_bp, carla.Transform(carla.Location(*RADAR_LOC)), attach_to=vehicle1)
        actors.append(radar)

        # camera callback: store the newest frame (BGR) for the main loop to render
        def camera_callback(image):
            array = np.frombuffer(image.raw_data, dtype=np.uint8)
            array = array.reshape(image.height, image.width, 4)[:, :, :3]
            with lock:
                state['frame'] = array

        def lidar_callback(point_cloud):
            points = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4)
            target = nearest_lidar_target(points)
            if target is not None:
                with lock:
                    state['lidar'] = target

        def radar_callback(radar_data):
            current_rot = radar_data.transform.rotation
            detections = []
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alti = math.degrees(detect.altitude)
                #The 0.25 adjusts a bit the distance so the dots can be properly seen
                fw_vec = carla.Vector3D(x=detect.depth - 0.25)
                carla.Transform(carla.Location(), carla.Rotation(
                    pitch=current_rot.pitch + alti,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll
                )).transform(fw_vec)
                world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=radar_color(detect.velocity))
                detections.append([detect.depth, detect.azimuth, detect.altitude])

            target = nearest_radar_target(detections)
            if target is not None:
                with lock:
                    state['radar'] = target

        camera.listen(camera_callback)
        lidar.listen(lidar_callback)
        radar.listen(radar_callback)

        if not args.no_display:
            cv2.namedWindow("carla Camera", cv2.WINDOW_NORMAL)

        kf = KalmanFilter(dt=0.1)
        initialized = False
        start = last = time.monotonic()

        while args.duration <= 0 or time.monotonic() - start < args.duration:
            now = time.monotonic()
            dt = now - last
            last = now

            # consume each measurement once so stale values are not fused again
            with lock:
                frame = None if state['frame'] is None else state['frame'].copy()
                measurements = [(z, R) for z, R in ((state['lidar'], R_LIDAR), (state['radar'], R_RADAR))
                                if z is not None]
                state['lidar'] = state['radar'] = None

            if not initialized and measurements:
                kf.x[:2] = measurements[0][0]
                initialized = True
            elif initialized:
                kf.predict(dt)
                for z, R in measurements:
                    kf.update(z, R)

            if initialized:
                px, py, vx, vy = kf.x
                status = f"target x={px:5.1f} m  y={py:5.1f} m  v=({vx:4.1f}, {vy:4.1f}) m/s"
            else:
                status = "no target"

            if args.no_display:
                print(status)
            elif frame is not None:
                cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                cv2.imshow("carla Camera", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping sensors and destroying actors...", flush=True)
        for actor in reversed(actors):
            if isinstance(actor, carla.Sensor) and actor.is_listening:
                actor.stop()
            # release vehicles from the Traffic Manager first; destroying an autopilot vehicle
            # can make the TM thread abort with "trying to operate on a destroyed actor"
            elif isinstance(actor, carla.Vehicle) and actor.is_alive:
                actor.set_autopilot(False)
        for actor in reversed(actors):
            if actor.is_alive:
                actor.destroy()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
