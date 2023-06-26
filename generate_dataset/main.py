import glob
import os
import sys
import time
from datetime import datetime
import numpy as np
import cv2
import carla
import argparse
import logging
import random
from sensors_spwan import SensorSpawn
import process_output
from generate_traffic import Traffic

# -----------
# SETTINGS
# -----------

# Simulation time settings
timeout = time.time() + 10

# Sensor settings
spawn_vehicle = True

drone_camera = False
veh_camera = True
veh_lidar = False

# BBOX settings
bbox = False
show_bbox = False

# Drone settings
drone_height = 100

# Traffic settings
traffic = False
num_vehicles = 50

# Location to save data
save_dir = 'export'

timestamp = datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
output_dir = save_dir + f'/test_{timestamp}'


# ----------------
# Load CARLA stuff
# ----------------
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

argparser = argparse.ArgumentParser(description=__doc__)
argparser.add_argument(
    '--host',
    metavar='H',
    default='127.0.0.1',
    help='IP of the host server (default: 127.0.0.1)')
argparser.add_argument(
    '-p', '--port',
    metavar='P',
    default=2000,
    type=int,
    help='TCP port to listen to (default: 2000)')
args = argparser.parse_args()

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

client = carla.Client(args.host, args.port)
client.set_timeout(10.0)


# ----------------
# Load world parameters
# ----------------

# Load world or use current
world = client.get_world()
# world = client.load_world('Town10HD_Opt') # Town01, Town03, Town07, Town10HD_Opt

# # Set world settings
settings = world.get_settings()
settings.synchronous_mode = False    # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Load world blueprint library
world_bp = world.get_blueprint_library()

# Set weather manually
# weather = carla.WeatherParameters(
#     cloudyness=80.0,
#     precipitation=30.0,
#     sun_altitude_angle=70.0)

# Set weather predefined
# weather = carla.WeatherParameters.ClearSunset

# Apply selected weather
# world.set_weather(weather)


# ----------------
# Spawn vehicles and sensors
# ----------------
# Determine spawn points
spawn_points = world.get_map().get_spawn_points()
number_of_spawn_points = len(spawn_points)

if spawn_vehicle:
    # Spawn ego vehicle
    vehicle_bp = world_bp.find('vehicle.tesla.model3')
    vehicle_bp.set_attribute('role_name', 'ego')
    vehicle_color = random.choice(vehicle_bp.get_attribute('color').recommended_values)
    vehicle_bp.set_attribute('color', vehicle_color)
    vehicle = None
    if 0 < number_of_spawn_points:
        random.shuffle(spawn_points)
        vehicle_transform = spawn_points[0]
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        print('\nVehicle is spawned')
    else:
        logging.warning('Could not found any spawn points')

    # Spawn sensors
    sensor_spawn = SensorSpawn(carla, world, world_bp, vehicle)

    if veh_camera:
        vehicle_cam, cam_bp = sensor_spawn.spawn_vehicle_cam()
        vehicle_cam.listen(lambda image: process_output.process_image(world, vehicle, vehicle_cam, cam_bp, image, output_dir, '/vehicle_camera', bbox, show_bbox))

    if veh_lidar:
        vehicle_lidar = sensor_spawn.spawn_vehicle_lidar()
        vehicle_lidar.listen(lambda point_cloud: process_output.process_pointcloud(point_cloud, output_dir, '/vehicle_lidar'))

    if drone_camera:
        drone_cam, drone_cam_bp = sensor_spawn.spawn_drone_cam(drone_height)
        drone_cam.listen(lambda image: process_output.process_image(world, vehicle, drone_cam, drone_cam_bp, image, output_dir, '/drone_camera', bbox, show_bbox))

    # --------------
    # Place spectator on ego spawning
    # --------------
    spectator = world.get_spectator()
    # world_snapshot = world.wait_for_tick()
    spectator.set_transform(vehicle.get_transform())

    # --------------
    # Enable autopilot for ego vehicle
    # --------------
    vehicle.set_autopilot(True)
    print('\nVehicle autopilot enabled')

# --------------
# Generate traffic
# --------------
if traffic:
    generate_traffic = Traffic(world, world_bp, client, spawn_points, num_vehicles)
    vehicles_list = generate_traffic.spawn_vehicles()

# Game loop
while time.time() < timeout:
    world.tick()
    # world_snapshot = world.wait_for_tick()

# Destroy vehicles and sensors
if traffic:
    client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

if spawn_vehicle:
    if veh_camera:
        vehicle_cam.stop()
        vehicle_cam.destroy()
    if veh_lidar:
        vehicle_lidar.stop()
        vehicle_lidar.destroy()
    if drone_camera:
        drone_cam.stop()
        drone_cam.destroy()
    vehicle.destroy()

cv2.destroyAllWindows()


print('\nSimulation ended')

