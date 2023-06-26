from datetime import datetime
import glob
import os
import sys
import cv2
import open3d as o3d
import time


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import pygame
import numpy as np

from sync_mode import CarlaSyncMode
from utils.sync_utils import draw_image, draw_lidar, get_font, should_quit
from generate_traffic import Traffic
from process_output import process_data, sweep_data
from process_output import find_objects

from export_data import DataExport

from process_lidar import process_lidar, lidar_callback, semantic_lidar_callback, add_open3d_axis

# Location to save data
save_dir = 'export'

timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
output_dir = 'data_output'

drone_output = True

traffic = True
walkers = True
num_vehicles = 100
num_pedestrians = 100

image_dim = [1920, 1080]

category_list = {
            "human.pedestrian.adult": "Regular pedestrian",
            "vehicle.car": "Any type of car, big or small",
            "vehicle.motorcycle": "Any type of motorcycle",
            "vehicle.bicycle": "Any type of bicycle"
        }


def main():
    actor_list = []
    sensor_list = {}
    sensors_active = {}
    traffic_list = []
    pygame.init()

    display = pygame.display.set_mode((1920, 1080), pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    sample_clock = 0
    random.seed(50)  # 5 = Corner,  10 = T-section, 50 = same as 5 but reversed

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        # Spawn ego vehicle
        vehicle = world.spawn_actor(
            random.choice(blueprint_library.filter('vehicle.*')),
            start_pose)

        actor_list.append(vehicle)
        vehicle.set_simulate_physics(False)

        # Generate traffic
        generate_traffic = Traffic(world, blueprint_library, client, m.get_spawn_points(), num_vehicles, num_pedestrians)
        if traffic:
            traffic_list = generate_traffic.spawn_vehicles()

        if walkers:
            walker_actors, walker_controllers = generate_traffic.spawn_pedestrians()

        # SPAWN SENSORS
        # ---- RGB CAM ---- #
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(1920))
        camera_bp.set_attribute('image_size_y', str(1080))
        camera_bp.set_attribute('sensor_tick', str(0.1))

        # ---- LIDAR ---- #
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('upper_fov', str(30))
        # lidar_bp.set_attribute('lower_fov', str(-25))
        lidar_bp.set_attribute('channels', str(64))
        lidar_bp.set_attribute('points_per_second', str(500000))
        lidar_bp.set_attribute('rotation_frequency', str(60))
        lidar_bp.set_attribute('range', str(50))

        # ---- SEMANTIC LIDAR ---- #
        lidar_sem = blueprint_library.find('sensor.lidar.ray_cast_semantic')
        lidar_sem.set_attribute('channels', str(64))
        lidar_sem.set_attribute('points_per_second', str(500000))
        lidar_sem.set_attribute('rotation_frequency', str(60))
        lidar_sem.set_attribute('range', str(50))

        # camera_rgb = world.spawn_actor(
        #     camera_bp,
        #     carla.Transform(carla.Location(x=1.6, z=1.6), carla.Rotation(pitch=-15)),
        #     attach_to=vehicle)
        # actor_list.append(camera_rgb)
        # sensor_list["CAM_FRONT"] = "camera"
        # sensors_active[camera_rgb] = "CAM_FRONT"

        cam_pos = [2, 0, 100]
        cam_rot = [-90, 0, 0]

        lidar_pos = [1, 0, 2.2]

        camera_drone = world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=cam_pos[0], y=cam_pos[1], z=cam_pos[2]),
                            carla.Rotation(cam_rot[0], cam_rot[1], cam_rot[2])),
            attach_to=vehicle)
        actor_list.append(camera_drone)
        sensor_list["CAM_DRONE"] = "camera"
        sensors_active[camera_drone] = "CAM_DRONE"

        # vehicle_lidar = world.spawn_actor(
        #     lidar_bp,
        #     carla.Transform(carla.Location(x=lidar_pos[0], y=lidar_pos[1], z=lidar_pos[2]), carla.Rotation(0, 0, 0)),
        #     attach_to=vehicle)
        # actor_list.append(vehicle_lidar)
        # sensor_list["LIDAR_TOP"] = "lidar"
        # sensors_active[vehicle_lidar] = "LIDAR_TOP"

        lidar_sem = world.spawn_actor(
            lidar_sem,
            carla.Transform(carla.Location(x=lidar_pos[0], y=lidar_pos[1], z=lidar_pos[2]), carla.Rotation(0, 0, 0)),
            attach_to=vehicle)
        actor_list.append(lidar_sem)
        sensor_list["LIDAR_TOP"] = "lidar"
        sensors_active[lidar_sem] = "LIDAR_TOP"

        # Create new scene
        data_out = DataExport("new_scene", sensor_list, sensors_active, 0)
        data_out.create_sensors()
        # data_out.create_categories()
        data_out.create_attributes()
        data_out.calibrated_sensors(camera_bp, cam_pos, lidar_pos, camera_bp.get_attribute("image_size_x").as_int(),
                                 camera_bp.get_attribute("image_size_y").as_int())

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_drone, lidar_sem, fps=10) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                # Advance the simulation and wait for the data.
                snapshot, image_drone, lidar_sem = sync_mode.tick(timeout=2.0)

                sensors_output = {
                    # camera_rgb: image_rgb,
                    "CAM_DRONE": image_drone,
                    "LIDAR_TOP": lidar_sem,
                    # "LIDAR_SEM": lidar_sem
                }

                if sample_clock == 0:
                    data_out.create_scene()
                    start_scene = True
                    end_scene = False
                elif sample_clock == 20:
                    start_scene = False
                    end_scene = True
                else:
                    start_scene = False
                    end_scene = False

                if sample_clock % 5 == 0:
                    is_key_frame = True
                else:
                    is_key_frame = False

                process_data(data_out=data_out,
                             world=world,
                             vehicle=vehicle,
                             sensor_list=sensor_list,
                             sensors_active=sensors_active,
                             sensors_output=sensors_output,
                             output_dir=output_dir,
                             scene_number="001",
                             is_key_frame=is_key_frame,
                             start_scene=start_scene,
                             end_scene=end_scene)

                sample_clock += 1

                # TODO: Fix scene numbering

                if end_scene:
                    # === TESTING SCENE EXPORT === #
                    print("exporting")
                    # TODO: Add separate loop later in main file to spawn in different location
                    # TODO: Add proper location to log

                    data_out.create_instances()
                    data_out.export_all("Town10")
                    sample_clock = 0


                # # Choose the next waypoint and update the car location.
                # waypoint = random.choice(waypoint.next(1.5))
                # vehicle.set_transform(waypoint.transform)
                vehicle.set_autopilot(True)

                # image_semseg.convert(carla.ColorConverter.CityScapesPalette)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)


                draw_image(display, image_drone)

                display.blit(
                    font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)),
                    (8, 10))
                display.blit(
                    font.render('% 5d FPS (simulated)' % fps, True, (255, 255, 255)),
                    (8, 28))
                pygame.display.flip()

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

        pygame.quit()

        cv2.destroyAllWindows()

        print('\ndone.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')