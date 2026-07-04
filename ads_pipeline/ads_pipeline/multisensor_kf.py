import carla
import numpy as np
import time
import cv2
import sys
import threading
import math

#connect with carla

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)


available_maps = client.get_available_maps()
print("available Maps")
for map_name in available_maps:
    print(f"{map_name}")

world = client.load_world('Town03')
world.set_weather(carla.WeatherParameters.HardRainNoon)
bp_lib = world.get_blueprint_library()
time.sleep(2.0)

#spawn vehicle

vehicle_bp = bp_lib.find('vehicle.tesla.model3')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle1 = world.spawn_actor(vehicle_bp,spawn_point)
vehicle1.set_autopilot(True)
#lead vehicle

vehicle_bp1 = bp_lib.find('vehicle.tesla.model3')
spawn_point1 = world.get_map().get_spawn_points()[1]
vehicle2 = world.spawn_actor(vehicle_bp1,spawn_point1)
vehicle2.set_autopilot(True)
print(f"Spawned vehicle: {vehicle1.type_id} at {spawn_point1.location}")

camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x','1280')
camera_bp.set_attribute('image_size_y', '720')
camera_bp.set_attribute('fov','90')
camera_tranform  = carla.Transform(carla.Location(x=1.5,z=2.4))
camera = world.spawn_actor(camera_bp, camera_tranform, attach_to=vehicle1)


#Attach LiDAR sensor

lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels','64')
lidar_bp.set_attribute('range','100.0')
lidar_bp.set_attribute('points_per_second','1200000')
lidar_bp.set_attribute('rotation_frequency','20')
lidar_tranform = carla.Transform(carla.Location(x=0.0,z=2.5))
lidar=world.spawn_actor(lidar_bp,lidar_tranform,attach_to=vehicle1)


# Attach RADAR sensor

radar_bp = bp_lib.find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov',str(35))
radar_bp.set_attribute('vertical_fov',str(20))
radar_bp.set_attribute('range',str(20))
radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
radar = world.spawn_actor(radar_bp,radar_transform, attach_to=vehicle1)


latest_frame = None
latest_lidar = None
latest_radar = None
latest_camera = None
frame_lock = threading.Lock()

try:
    cv2.namedWindow("carla Camera", cv2.WINDOW_NORMAL)

    # camera callback: store the newest frame for the main loop to render
    def camera_callback(image):
        global latest_frame, latest_camera
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape(image.height, image.width, 4)[:, :, :3]
        with frame_lock:
            latest_frame = array

        bbox_width_px = 80
        fov_deg = 90

        focal_length = image.width / (2 * np.tan(np.radians(fov_deg)/2.0))

        know_vehicle_width = 1.8

        distance  = (know_vehicle_width * focal_length) / bbox_width_px

        latest_camera = distance     

    camera.listen(camera_callback)
    
    def lidar_callback(point_cloud):
        data = np.frombuffer(point_cloud.raw_data, dtype=np.float32)
        points = data.reshape(-1,4)

        print(f"[LiDAR] Frame {point_cloud.frame}:" f"{points.shape[0]} points,"
              f"range: {np.linalg.norm(points[:, :3], axis=1).max():.1f} m")
    lidar.listen(lidar_callback)  

    def radar_callback(radar_data):
        velocity_range = 7.5 #m/s
        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alti  = math.degrees(detect.altitude)
            #The 0.25 adjusts a bit the distance so the dots can 
            # be properly seen
            fw_vec = carla.Vector3D(x = detect.depth - 0.25)
            carla.Transform(carla.Location(),carla.Rotation(
                pitch=current_rot.pitch + alti,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll
            )).transform(fw_vec)
            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            world.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))
    radar.listen(lambda radar_data: radar_callback(radar_data))


    while True:
        with frame_lock:
            frame = None if latest_frame is None else latest_frame.copy()

        if frame is not None:
            cv2.imshow("carla Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping sensors and destroying actors...")
finally:
    for actor in (camera, lidar,radar, vehicle1, vehicle2):
        if actor is not None:
            actor.destroy()
    cv2.destroyAllWindows()
    sys.exit(0)