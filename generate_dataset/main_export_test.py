import glob
import os
import sys
import time

import cv2
import carla
import random
import pygame
import subprocess


from sync_mode import CarlaSyncMode
from utils.sync_utils import draw_image, get_font, should_quit
from generate_traffic import Traffic
from process_output import ProcessOutput
from export_data import DataExport
from utils.camera_projection import build_projection_matrix
from config_dicts import tags_dict, attributes, categories, visibilities


carla_path = os.path.normpath(os.path.join(os.getcwd(), "../../../Carla/CarlaUE4.exe"))
# process = subprocess.Popen(carla_path)

# time.sleep(15)

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

output_dir = 'data_output'

traffic = True
walkers = True
num_vehicles = 100
num_pedestrians = 100
scene_length = 5
no_scenes = 10
scenes = {}

maps = ["Town10"]

weather = [(carla.WeatherParameters.ClearNoon, int(no_scenes * 0.3)),
           (carla.WeatherParameters.HardRainNoon, int(no_scenes * 0.2)),
           (carla.WeatherParameters.ClearSunset, int(no_scenes * 0.3)),
           (carla.WeatherParameters.WetCloudySunset, int(no_scenes * 0.2))]

w_choices = ([weather[0][0]] * weather[0][1]) + ([weather[1][0]] * weather[1][1]) + ([weather[2][0]] * weather[2][1]) + \
            ([weather[3][0]] * weather[3][1])

map_split = int(no_scenes * 1)
tt_split = int(no_scenes * 0.9)

for i in range(no_scenes):
    scene_name = "scene" + str(i + 1)
    seed = i + 1
    w = w_choices[i]

    if (i+1) > map_split:
        map = maps[1]
    else:
        map = maps[0]

    if (i+1) > tt_split:
        tt = "test"
    else:
        tt = "trainval"

    scenes[scene_name] = (seed, map, w, tt)

print(scenes)


def main():
    scene_start = True
    for scene, info in scenes.items():
        seed = info[0]
        map = info[1]
        w = info[2]
        tt = info[3]

        actor_list = []
        sensor_list = {}
        sensors_active = {}
        traffic_list = []
        pygame.init()

        client = carla.Client('localhost', 2000)
        client.set_timeout(500.0)

        world = client.get_world()
        world.set_weather(w)
        # world = client.load_world(map)

        sample_clock = -10      # Add delay to wait for vehicles to spawn properly

        if tt == 'test':
            scene_start = True

        try:
            random.seed(seed)
            m = world.get_map()
            start_pose = random.choice(m.get_spawn_points())
            random.seed(None)

            blueprint_library = world.get_blueprint_library()

            # --------- Spawn ego vehicle --------- #
            # Set to model 3
            vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]
            vehicle = world.spawn_actor(vehicle_blueprint, start_pose)

            actor_list.append(vehicle)
            vehicle.set_simulate_physics(False)

            # SENSOR CONFIGURATIONS
            # ---- RGB CAM DRONE ---- #
            camera_bp_drone = blueprint_library.find('sensor.camera.rgb')
            camera_bp_drone.set_attribute('image_size_x', str(1920))
            camera_bp_drone.set_attribute('image_size_y', str(1080))
            camera_bp_drone.set_attribute('sensor_tick', str(0.1))
            camera_bp_drone.set_attribute('fov', str(75))

            # ---- RGB CAM CAR ---- #
            camera_bp_car = blueprint_library.find('sensor.camera.rgb')
            camera_bp_car.set_attribute('image_size_x', str(1280))
            camera_bp_car.set_attribute('image_size_y', str(720))
            camera_bp_car.set_attribute('sensor_tick', str(0.1))
            camera_bp_car.set_attribute('fov', str(100))

            # ---- DEPTH CAM DRONE ---- #
            depth_bp_drone = blueprint_library.find('sensor.camera.depth')
            depth_bp_drone.set_attribute('image_size_x', str(1920))
            depth_bp_drone.set_attribute('image_size_y', str(1080))
            depth_bp_drone.set_attribute('sensor_tick', str(0.1))
            depth_bp_drone.set_attribute('fov', str(75))

            # ---- DEPTH CAM CAR ---- #
            depth_bp_car = blueprint_library.find('sensor.camera.depth')
            depth_bp_car.set_attribute('image_size_x', str(1280))
            depth_bp_car.set_attribute('image_size_y', str(720))
            depth_bp_car.set_attribute('sensor_tick', str(0.1))
            depth_bp_car.set_attribute('fov', str(100))

            # ---- SEMANTIC LIDAR ---- #
            lidar_sem = blueprint_library.find('sensor.lidar.ray_cast_semantic')
            lidar_sem.set_attribute('channels', str(64))
            lidar_sem.set_attribute('points_per_second', str(500000))
            lidar_sem.set_attribute('rotation_frequency', str(60))
            lidar_sem.set_attribute('range', str(50))

            # Sensor Positions
            # DRONE
            cam_drone_ego = [2, 0]
            cam_drone_h = {
                "CAM_DRONE_100": 100,
            }
            cam_rot = [-90, 0, 0]

            # CAR CAMS
            cam_front = [1.5, 0, 3.2]
            cam_front_rot = [0, 0, 0]

            # CAR LIDAR
            lidar_pos = [1, 0, 3.2]

            # --------- Spawn sensors --------- #
            # -------- CAR CAMS ------- #
            # CAR FRONT
            camera_front = world.spawn_actor(
                camera_bp_car,
                carla.Transform(carla.Location(x=cam_front[0], y=cam_front[1], z=cam_front[2]),
                                carla.Rotation(cam_front_rot[0], cam_front_rot[1], cam_front_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_front)
            sensor_list["CAM_FRONT"] = "camera"
            sensors_active[camera_front] = "CAM_FRONT"

            # --------- DRONE CAMS ---------- #
            # DRONE CAM 100
            camera_drone_100 = world.spawn_actor(
                camera_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_100"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_100)
            sensor_list["CAM_DRONE_100"] = "camera"
            sensors_active[camera_drone_100] = "CAM_DRONE_100"

            # -------- DEPTH CAMS -------- #
            # CAR FRONT
            camera_dep_front = world.spawn_actor(
                depth_bp_car,
                carla.Transform(carla.Location(x=cam_front[0], y=cam_front[1], z=cam_front[2]),
                                carla.Rotation(cam_front_rot[0], cam_front_rot[1], cam_front_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_front)

            # DRONE DEPTH 100
            camera_dep_100 = world.spawn_actor(
                depth_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_100"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_100)

            # # -------- LIDAR --------- #
            # # CAR LIDAR
            # lidar_sem = world.spawn_actor(
            #     lidar_sem,
            #     carla.Transform(carla.Location(x=lidar_pos[0], y=lidar_pos[1], z=lidar_pos[2]),
            #                     carla.Rotation(0, 0, 0)),
            #     attach_to=vehicle)
            # actor_list.append(lidar_sem)
            # sensor_list["LIDAR_TOP"] = "lidar"
            # sensors_active[lidar_sem] = "LIDAR_TOP"

            # --------- CAMERA ATTRIBUTES CALC --------- #
            # Get the attributes from the drone camera
            image_w_drone = camera_bp_drone.get_attribute("image_size_x").as_int()
            image_h_drone = camera_bp_drone.get_attribute("image_size_y").as_int()
            fov_drone = camera_bp_drone.get_attribute("fov").as_float()

            # Calculate the camera projection matrix to project from 3D -> 2D
            K_drone = build_projection_matrix(image_w_drone, image_h_drone, fov_drone)

            # Get the attributes from the car camera
            image_w_car = camera_bp_car.get_attribute("image_size_x").as_int()
            image_h_car = camera_bp_car.get_attribute("image_size_y").as_int()
            fov_car = camera_bp_car.get_attribute("fov").as_float()

            # Calculate the camera projection matrix to project from 3D -> 2D
            K_car = build_projection_matrix(image_w_car, image_h_car, fov_car)

            drone_cameras = {
                "CAM_DRONE_100": (camera_drone_100, 100),
            }

            car_cameras = {
                "CAM_FRONT": (camera_dep_front, [1.5, 0, 0.4]),
            }

            # ------- GENERATE TRAFFIC --------- #
            generate_traffic = Traffic(world, blueprint_library, client, m.get_spawn_points(), num_vehicles,
                                       num_pedestrians, seed)
            if traffic:
                traffic_list = generate_traffic.spawn_vehicles()
            if walkers:
                walker_actors, walker_controllers = generate_traffic.spawn_pedestrians()

            # Create new scene
            if scene_start:
                data_out = DataExport(scene, sensor_list, sensors_active, seed, attributes, categories, visibilities,
                                      tt)
                data_out.create_maps(maps)
                data_out.create_sensors()
                data_out.create_attributes()
                data_out.create_categories()
                data_out.create_visibility()
                data_out.calibrated_sensors(camera_bp_drone, None, camera_bp_car, cam_drone_ego,
                                            cam_front, cam_drone_h, lidar_pos, image_w_drone, image_h_drone,
                                            None, None, image_w_car, image_h_car)
                scene_start = False
            else:
                data_out.tt = tt
                data_out.scene_name = scene

            # Create new process output class for this scene
            process_output = ProcessOutput(world=world,
                                           vehicle=vehicle,
                                           scene=scene,
                                           data_out=data_out,
                                           sensor_list=sensor_list,
                                           sensors_active=sensors_active,
                                           output_dir=output_dir,
                                           drone_cameras=drone_cameras,
                                           car_cameras=car_cameras,
                                           image_w_drone=image_w_drone,
                                           image_h_drone=image_h_drone,
                                           image_w_drone_4k=None,
                                           image_h_drone_4k=None,
                                           image_w_car=image_w_car,
                                           image_h_car=image_h_car,
                                           K_drone=K_drone,
                                           K_drone_4k=None,
                                           K_car=K_car,
                                           tags_dict=tags_dict,
                                           tt=tt)

            # Create a synchronous mode context.
            with CarlaSyncMode(world, camera_drone_100, camera_front, camera_dep_front, camera_dep_100, fps=10) as sync_mode:
                while True:
                    if should_quit():
                        return

                    vehicle.set_autopilot(True)

                    # Advance the simulation and wait for the data.
                    snapshot, image_drone_100, image_front, image_dep_front, image_dep_100,  = sync_mode.tick(timeout=2.0)

                    sensors_output = {
                        "CAM_FRONT": image_front,
                        "CAM_DRONE_100": image_drone_100,
                        # "LIDAR_TOP": lidar_out,
                    }

                    depth_output = {
                        "CAM_FRONT": image_dep_front,
                        "CAM_DRONE_100": image_dep_100,
                    }

                    if sample_clock == 0:
                        data_out.create_scene()
                        data_out.create_log(map)
                        start_scene = True
                        end_scene = False
                    elif sample_clock == scene_length:
                        start_scene = False
                        end_scene = True
                    else:
                        start_scene = False
                        end_scene = False

                    if sample_clock % 5 == 0 and sample_clock >= 0:
                        is_key_frame = True
                    else:
                        is_key_frame = False

                    if sample_clock >= 0:
                        process_output.process_data(sample_clock=sample_clock,
                                                    sensors_output=sensors_output,
                                                    depth=depth_output,
                                                    is_key_frame=is_key_frame,
                                                    start_scene=start_scene,
                                                    end_scene=end_scene)

                    print(f'scene: {scene}, sample: {sample_clock}')
                    sample_clock += 1

                    if end_scene:
                        # === TESTING SCENE EXPORT === #
                        print("Exporting scene")
                        data_out.create_instances()
                        data_out.export_all()
                        data_out.reset()

                        break

        finally:
            print('\ndestroying actors.')
            for actor in actor_list:
                actor.destroy()

            if traffic:
                for actor in traffic_list:
                    actor.destroy()

                print('\ndestroyed vehicles')

            if walkers:
                for actor in walker_actors:
                    actor.destroy()
                for controller in walker_controllers:
                    controller.stop()
                print('\ndestroyed walkers')

            print('\ndone.')

            time.sleep(3)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')




