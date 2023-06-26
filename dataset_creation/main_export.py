import glob
import os
import sys
import time

import cv2
import carla
import random
import pygame
import subprocess
import psutil
import math

from sync_mode import CarlaSyncMode
from utils.sync_utils import draw_image, get_font, should_quit
from utils.generate_traffic import Traffic
from process_output import ProcessOutput
from export_data import DataExport
from utils.camera_projection import build_projection_matrix
from utils.config_dicts import tags_dict, attributes, categories, visibilities
from utils.import_existing import check_data

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

carla_path = os.path.normpath(os.path.join(os.getcwd(), "../../../Carla/CarlaUE4.exe"))
output_dir = 'data_output/mini_set_2'

output_display = True
traffic = True
walkers = False
num_vehicles = 600
num_pedestrians = 100
scene_length = 200
no_scenes_train = 80
no_scenes_test = 20
scenes = {}

maps = ["Town05"]

weather = [(carla.WeatherParameters.ClearNoon, int(no_scenes_train * 0.3)),
           (carla.WeatherParameters.ClearSunset, int(no_scenes_train * 0.3)),
           (carla.WeatherParameters.CloudyNoon, int(no_scenes_train * 0.2)),
           (carla.WeatherParameters.HardRainNoon, int(no_scenes_train * 0.2))]

w_choices_train = ([weather[0][0]] * weather[0][1]) + ([weather[1][0]] * weather[1][1]) + ([weather[2][0]] * weather[2][1]) + \
    ([weather[3][0]] * weather[3][1])

weather_test = [(carla.WeatherParameters.ClearNoon, int(no_scenes_test * 0.3)),
           (carla.WeatherParameters.ClearSunset, int(no_scenes_test * 0.3)),
           (carla.WeatherParameters.CloudyNoon, int(no_scenes_test * 0.2)),
           (carla.WeatherParameters.HardRainNoon, int(no_scenes_test * 0.2))]

w_choices_test = ([weather_test[0][0]] * weather_test[0][1]) + ([weather_test[1][0]] * weather_test[1][1]) + ([weather_test[2][0]] * weather_test[2][1]) + \
    ([weather_test[3][0]] * weather_test[3][1])

for i in range(no_scenes_train):
    scene_name = "scene" + str(i)
    seed = i + 1

    w = w_choices_train[i]

    if i < no_scenes_train/2:
        w = w_choices_train[i]
    else:
        w = w_choices_train[(i - int(no_scenes_train/2))]

    # w = carla.WeatherParameters.ClearNoon

    # if (i+1) > map_split_train:
    #     map = maps[1]
    # else:
    #     map = maps[0]

    map = maps[0]

    tt = "trainval"
    scenes[scene_name] = (seed, map, w, tt)

# for i in range(no_scenes_test):
#     scene_name = "scene" + str(no_scenes_train + i + 1)
#     seed = no_scenes_train + i + 1
#
#     w = w_choices_test[i]
#
#     # if i < no_scenes_test / 2:
#     #     w = w_choices_test[i]
#     # else:
#     #     w = w_choices_test[(i - int(no_scenes_test / 2))]
#
#     # if (i+1) > map_split_test:
#     #     map = maps[1]
#     # else:
#     #     map = maps[0]
#
#     map = maps[0]
#
#     tt = "test"
#     scenes[scene_name] = (seed, map, w, tt)

# scenes = {"scene1": (0, "Town10", carla.WeatherParameters.ClearNoon, 'trainval'),
#           "scene2": (1, "Town10", carla.WeatherParameters.ClearNoon, 'trainval'),
#           "scene3": (2, "Town10", carla.WeatherParameters.ClearNoon, 'trainval')}

print(scenes)


# Startup CARLA
subprocess.Popen(carla_path)
time.sleep(15)

start_time = time.time()


def main():
    scene_start = True
    for scene, info in scenes.items():
        print(f'\n Starting: {scene}')

        # Assign variables
        seed = info[0]
        map = info[1]
        w = info[2]
        tt = info[3]

        actor_list = []
        sensor_list = {}
        sensors_active = {}
        traffic_list = []

        if tt == 'test':
            scene_start = True

        if output_display:
            pygame.init()
            display = pygame.display.set_mode((1920, 1080), pygame.HWSURFACE | pygame.DOUBLEBUF)
            font = get_font()
            clock = pygame.time.Clock()

        client = carla.Client('localhost', 2000)
        client.set_timeout(500.0)

        if seed == 395:
            world = client.load_world(map)
        else:
            world = client.reload_world()

        time.sleep(5)

        # settings = world.get_settings()
        # settings.tile_stream_distance = 1000
        # world.apply_settings(settings)

        world.set_weather(w)

        sample_clock = -10      # Add delay to wait for vehicles to spawn properly

        try:
            random.seed(seed)
            m = world.get_map()
            start_pose = random.choice(m.get_spawn_points())
            random.seed(None)

            blueprint_library = world.get_blueprint_library()

            # --------- Spawn ego vehicle --------- #
            # Set to model 3
            vehicle_blueprint = world.get_blueprint_library().filter('model3')[0]
            # vehicle_blueprint.set_attribute('role_name', 'hero')
            vehicle = world.spawn_actor(vehicle_blueprint, start_pose)

            actor_list.append(vehicle)
            vehicle.set_simulate_physics(False)
            # vehicle.set_autopilot(True)

            # SENSOR CONFIGURATIONS
            # ---- RGB CAM DRONE ---- #
            camera_bp_drone = blueprint_library.find('sensor.camera.rgb')
            camera_bp_drone.set_attribute('image_size_x', str(1920))
            camera_bp_drone.set_attribute('image_size_y', str(1080))
            camera_bp_drone.set_attribute('sensor_tick', str(0.1))
            camera_bp_drone.set_attribute('fov', str(75))

            # ---- RGB CAM DRONE 4K ---- #
            camera_bp_drone_4k = blueprint_library.find('sensor.camera.rgb')
            camera_bp_drone_4k.set_attribute('image_size_x', str(3840))
            camera_bp_drone_4k.set_attribute('image_size_y', str(2160))
            camera_bp_drone_4k.set_attribute('sensor_tick', str(0.1))
            camera_bp_drone_4k.set_attribute('fov', str(75))

            # ---- DEPTH CAM DRONE ---- #
            depth_bp_drone = blueprint_library.find('sensor.camera.depth')
            depth_bp_drone.set_attribute('image_size_x', str(1920))
            depth_bp_drone.set_attribute('image_size_y', str(1080))
            depth_bp_drone.set_attribute('sensor_tick', str(0.1))
            depth_bp_drone.set_attribute('fov', str(75))

            # ---- DEPTH CAM DRONE 4K ---- #
            depth_bp_drone_4k = blueprint_library.find('sensor.camera.depth')
            depth_bp_drone_4k.set_attribute('image_size_x', str(3840))
            depth_bp_drone_4k.set_attribute('image_size_y', str(2160))
            depth_bp_drone_4k.set_attribute('sensor_tick', str(0.1))
            depth_bp_drone_4k.set_attribute('fov', str(75))

            # ---- RGB CAM CAR ---- #
            camera_bp_car = blueprint_library.find('sensor.camera.rgb')
            camera_bp_car.set_attribute('image_size_x', str(1280))
            camera_bp_car.set_attribute('image_size_y', str(720))
            camera_bp_car.set_attribute('sensor_tick', str(0.1))
            camera_bp_car.set_attribute('fov', str(100))

            # ---- DEPTH CAM CAR ---- #
            depth_bp_car = blueprint_library.find('sensor.camera.depth')
            depth_bp_car.set_attribute('image_size_x', str(1280))
            depth_bp_car.set_attribute('image_size_y', str(720))
            depth_bp_car.set_attribute('sensor_tick', str(0.1))
            depth_bp_car.set_attribute('fov', str(100))

            # ---- SEMANTIC LIDAR ---- #
            lidar_sem = blueprint_library.find('sensor.lidar.ray_cast_semantic')
            lidar_sem.set_attribute('upper_fov', str(10))
            lidar_sem.set_attribute('lower_fov', str(-30))
            lidar_sem.set_attribute('channels', str(32))
            lidar_sem.set_attribute('points_per_second', str(350000))
            lidar_sem.set_attribute('rotation_frequency', str(10))
            lidar_sem.set_attribute('range', str(75))

            # Sensor Positions
            # DRONE
            cam_drone_ego = [2, 0]
            cam_drone_h = {
                "CAM_DRONE_25": 25,
                "CAM_DRONE_50": 50,
                "CAM_DRONE_100": 100,
                "CAM_DRONE_200": 200,
                "CAM_DRONE_400_4K": 400
            }
            cam_rot = [-90, 0, 0]

            # CAR CAMS
            cam_front = [1.5, 0, 3.2]
            cam_front_rot = [0, 0, 0]

            cam_right = [0, 1.5, 3.2]
            cam_right_rot = [0, 90, 0]

            cam_left = [0, -1.5, 3.2]
            cam_left_rot = [0, -90, 0]

            cam_back = [-2, 0, 3.2]
            cam_back_rot = [0, 180, 0]

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
            # DRONE CAM 25
            camera_drone_25 = world.spawn_actor(
                camera_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_25"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_25)
            sensor_list["CAM_DRONE_25"] = "camera"
            sensors_active[camera_drone_25] = "CAM_DRONE_25"

            # DRONE CAM 50
            camera_drone_50 = world.spawn_actor(
                camera_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_50"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_50)
            sensor_list["CAM_DRONE_50"] = "camera"
            sensors_active[camera_drone_50] = "CAM_DRONE_50"

            # DRONE CAM 100
            camera_drone_100 = world.spawn_actor(
                camera_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_100"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_100)
            sensor_list["CAM_DRONE_100"] = "camera"
            sensors_active[camera_drone_100] = "CAM_DRONE_100"

            # DRONE CAM 200
            camera_drone_200 = world.spawn_actor(
                camera_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_200"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_200)
            sensor_list["CAM_DRONE_200"] = "camera"
            sensors_active[camera_drone_200] = "CAM_DRONE_200"

            # DRONE CAM 400
            camera_drone_400_4k = world.spawn_actor(
                camera_bp_drone_4k,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_400_4K"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_drone_400_4k)
            sensor_list["CAM_DRONE_400_4K"] = "camera"
            sensors_active[camera_drone_400_4k] = "CAM_DRONE_400_4K"

            # -------- DEPTH CAMS -------- #
            # CAR FRONT
            camera_dep_front = world.spawn_actor(
                depth_bp_car,
                carla.Transform(carla.Location(x=cam_front[0], y=cam_front[1], z=cam_front[2]),
                                carla.Rotation(cam_front_rot[0], cam_front_rot[1], cam_front_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_front)

            # CAR RIGHT
            camera_dep_right = world.spawn_actor(
                depth_bp_car,
                carla.Transform(carla.Location(x=cam_right[0], y=cam_right[1], z=cam_right[2]),
                                carla.Rotation(cam_right_rot[0], cam_right_rot[1], cam_right_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_right)

            # CAR BACK
            camera_dep_back = world.spawn_actor(
                depth_bp_car,
                carla.Transform(carla.Location(x=cam_back[0], y=cam_back[1], z=cam_back[2]),
                                carla.Rotation(cam_back_rot[0], cam_back_rot[1], cam_back_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_back)

            # CAR LEFT
            camera_dep_left = world.spawn_actor(
                depth_bp_car,
                carla.Transform(carla.Location(x=cam_left[0], y=cam_left[1], z=cam_left[2]),
                                carla.Rotation(cam_left_rot[0], cam_left_rot[1], cam_left_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_left)

            # DRONE DEPTH 25
            camera_dep_25 = world.spawn_actor(
                depth_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_25"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_25)

            # DRONE DEPTH 50
            camera_dep_50 = world.spawn_actor(
                depth_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_50"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_50)

            # DRONE DEPTH 100
            camera_dep_100 = world.spawn_actor(
                depth_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_100"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_100)

            # DRONE DEPTH 200
            camera_dep_200 = world.spawn_actor(
                depth_bp_drone,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_200"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_200)

            # DRONE DEPTH 400
            camera_dep_400_4k = world.spawn_actor(
                depth_bp_drone_4k,
                carla.Transform(carla.Location(x=cam_drone_ego[0], y=cam_drone_ego[1], z=cam_drone_h["CAM_DRONE_400_4K"]),
                                carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
                attach_to=vehicle)
            actor_list.append(camera_dep_400_4k)

            # -------- LIDAR --------- #
            # CAR LIDAR
            lidar_sem = world.spawn_actor(
                lidar_sem,
                carla.Transform(carla.Location(x=lidar_pos[0], y=lidar_pos[1], z=lidar_pos[2]),
                                carla.Rotation(0, -90, 0)),
                attach_to=vehicle)
            actor_list.append(lidar_sem)
            sensor_list["LIDAR_TOP"] = "lidar"
            sensors_active[lidar_sem] = "LIDAR_TOP"

            # --------- CAMERA ATTRIBUTES CALC --------- #
            # Get the attributes from the drone cameras
            image_w_drone = camera_bp_drone.get_attribute("image_size_x").as_int()
            image_h_drone = camera_bp_drone.get_attribute("image_size_y").as_int()
            fov_drone = camera_bp_drone.get_attribute("fov").as_float()

            # Calculate the camera projection matrix to project from 3D -> 2D
            K_drone = build_projection_matrix(image_w_drone, image_h_drone, fov_drone)

            image_w_drone_4k = camera_bp_drone_4k.get_attribute("image_size_x").as_int()
            image_h_drone_4k = camera_bp_drone_4k.get_attribute("image_size_y").as_int()
            fov_drone_4k = camera_bp_drone_4k.get_attribute("fov").as_float()

            # Calculate the camera projection matrix to project from 3D -> 2D
            K_drone_4k = build_projection_matrix(image_w_drone_4k, image_h_drone_4k, fov_drone_4k)

            # Get the attributes from the car camera
            image_w_car = camera_bp_car.get_attribute("image_size_x").as_int()
            image_h_car = camera_bp_car.get_attribute("image_size_y").as_int()
            fov_car = camera_bp_car.get_attribute("fov").as_float()

            # Calculate the camera projection matrix to project from 3D -> 2D
            K_car = build_projection_matrix(image_w_car, image_h_car, fov_car)

            drone_cameras = {
                "CAM_DRONE_25": (camera_drone_25, 25),
                "CAM_DRONE_50": (camera_drone_50, 50),
                "CAM_DRONE_100": (camera_drone_100, 100),
                "CAM_DRONE_200": (camera_drone_200, 200),
                "CAM_DRONE_400_4K": (camera_drone_400_4k, 400)
            }

            car_cameras = {
                "CAM_FRONT": (camera_dep_front, [1.5, 0, 0.4]),
                "CAM_RIGHT": (camera_dep_right, [0, 1.5, 1.4]),
                "CAM_LEFT": (camera_dep_left, [0, -1.5, 1.4]),
                "CAM_BACK": (camera_dep_back, [-2, 0, 1.4])
            }

            # ------- GENERATE TRAFFIC --------- #
            generate_traffic = Traffic(world, blueprint_library, client, m, num_vehicles,
                                       num_pedestrians, seed)
            if traffic:
                traffic_list = generate_traffic.spawn_vehicles()
            # if walkers:
            #     walker_actors, walker_controllers = generate_traffic.spawn_pedestrians()
            print(f'traffic spawned')

            # Create new scene
            if scene_start:
                data_out = DataExport(scene, sensor_list, sensors_active, seed, attributes, categories, visibilities, tt)
                existing = check_data(output_dir, "horus")
                print(f'EXISTING: {existing}')

                if existing == False:
                    data_out.create_maps(maps)
                    data_out.create_sensors()
                    data_out.create_attributes()
                    data_out.create_categories()
                    data_out.create_visibility()
                    data_out.calibrated_sensors(camera_bp_drone, camera_bp_drone_4k, camera_bp_car, cam_drone_ego,
                                                cam_front, cam_drone_h, lidar_pos, image_w_drone, image_h_drone,
                                                image_w_drone_4k, image_h_drone_4k, image_w_car, image_h_car)
                else:
                    sensor_tokens, calibrated_sensor_tokens, attribute_tokens, category_tokens = existing
                    data_out.sensor_tokens = sensor_tokens
                    data_out.calibrated_tokens = calibrated_sensor_tokens
                    data_out.attribute_tokens = attribute_tokens
                    data_out.category_tokens = category_tokens

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
                                           image_w_drone_4k=image_w_drone_4k,
                                           image_h_drone_4k=image_h_drone_4k,
                                           image_w_car=image_w_car,
                                           image_h_car=image_h_car,
                                           K_drone=K_drone,
                                           K_drone_4k=K_drone_4k,
                                           K_car=K_car,
                                           tags_dict=tags_dict,
                                           tt=tt)

            # Create a synchronous mode context.
            with CarlaSyncMode(world, camera_drone_25, camera_drone_50, camera_drone_100, camera_drone_200,
                               camera_drone_400_4k, camera_front, camera_dep_front, camera_dep_right, camera_dep_back,
                               camera_dep_left, camera_dep_25, camera_dep_50, camera_dep_100, camera_dep_200,
                               camera_dep_400_4k, lidar_sem, fps=10) as sync_mode:
                while True:
                    if output_display:
                        if should_quit():
                            return
                        clock.tick()

                    vehicle.set_autopilot(True)

                    # Advance the simulation and wait for the data.
                    snapshot, image_drone_25, image_drone_50, image_drone_100, image_drone_200, image_drone_400_4k, \
                    image_front, image_dep_front, image_dep_right, image_dep_back,\
                    image_dep_left, image_dep_25, image_dep_50, image_dep_100, image_dep_200, image_dep_400_4k, lidar_out = sync_mode.tick(timeout=5)

                    sensors_output = {
                        "CAM_FRONT": image_front,
                        "CAM_DRONE_25": image_drone_25,
                        "CAM_DRONE_50": image_drone_50,
                        "CAM_DRONE_100": image_drone_100,
                        "CAM_DRONE_200": image_drone_200,
                        "CAM_DRONE_400_4K": image_drone_400_4k,
                        "LIDAR_TOP": lidar_out,
                    }

                    depth_output = {
                        "CAM_FRONT": image_dep_front,
                        "CAM_RIGHT": image_dep_right,
                        "CAM_BACK": image_dep_back,
                        "CAM_LEFT": image_dep_left,
                        "CAM_DRONE_25": image_dep_25,
                        "CAM_DRONE_50": image_dep_50,
                        "CAM_DRONE_100": image_dep_100,
                        "CAM_DRONE_200": image_dep_200,
                        "CAM_DRONE_400_4K": image_dep_400_4k,
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

                    if sample_clock % 10 == 0:
                        print(f'{scene}, sample: {sample_clock}')
                    sample_clock += 1
                    if output_display:
                        # Output display
                        fps = round(1.0 / snapshot.timestamp.delta_seconds)
                        # image_dep_back.convert(carla.ColorConverter.LogarithmicDepth)
                        image_dep_front.convert(carla.ColorConverter.LogarithmicDepth)
                        draw_image(display, image_dep_front)

                        # draw_image(display, image_drone_200)

                        display.blit(
                            font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                            (8, 10))
                        display.blit(
                            font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                            (8, 28))
                        pygame.display.flip()

                    if end_scene:

                        # === TESTING SCENE EXPORT === #
                        print("Exporting scene")
                        # print("Creating instances")
                        # data_out.create_instances()
                        print("Exporting scene, annotations, sample data and instances")
                        data_out.export_all()
                        print("Resetting")
                        data_out.reset()
                        print("Scene done")
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
                    # controller.stop()
                    controller.destroy()
                print('\ndestroyed walkers')

            if output_display:
                pygame.quit()

            # cv2.destroyAllWindows()
            time.sleep(5)

    for proc in psutil.process_iter():
        if "CarlaUE4-Win64-Shipping.exe" in proc.name():
            print('terminating')
            proc.terminate()
            break

    end_time = time.time()
    print(f'COMPLETED IN: {end_time - start_time}')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')









