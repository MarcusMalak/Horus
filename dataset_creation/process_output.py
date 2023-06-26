import carla
import numpy as np
import os
import cv2
import time
from utils.draw_bbox import draw_bbox
from typing import Union
from utils.quaternion import get_quaternion_from_euler
from utils.camera_projection import get_camera_intrinsic
import open3d as o3d
from scipy.spatial import distance
from utils.occlusion import filter_occluded
from utils.occlusion import get_visiblity_ego
from PIL import Image
import cv2 as cv


class ProcessOutput:
    def __init__(self,  world, vehicle, scene, data_out, sensor_list, sensors_active, output_dir, drone_cameras,
                 car_cameras, image_w_drone, image_h_drone, image_w_drone_4k, image_h_drone_4k, image_w_car,
                 image_h_car, K_drone, K_drone_4k, K_car, tags_dict, tt):
        self.world = world
        self.vehicle = vehicle
        self.scene_number = scene
        self.data_out = data_out
        self.sensor_list = sensor_list
        self.sensors_active = sensors_active
        self.output_dir = output_dir
        self.drone_cameras = drone_cameras
        self.car_cameras = car_cameras
        self.image_w_drone = image_w_drone
        self.image_h_drone = image_h_drone
        self.image_w_drone_4k = image_w_drone_4k
        self.image_h_drone_4k = image_h_drone_4k
        self.image_w_car = image_w_car
        self.image_h_car = image_h_car
        self.K_drone = K_drone
        self.K_drone_4k = K_drone_4k
        self.K_car = K_car
        self.tags_dict = tags_dict
        self.tt = tt

        self.sensors_output = None
        self.depth = None

        self.parked_ids = {}

    def process_data(self, sample_clock, sensors_output, depth, is_key_frame, start_scene, end_scene):
        self.sensors_output = sensors_output
        self.depth = depth

        sensor_sample_data = []
        objects = []

        # timestamp = int(time.time())
        ego_transform = self.vehicle.get_transform().location
        ego_transform.z = 0  # Set Z coordinate to 0, see nuScenes documentation
        ego_transform = get_loc(ego_transform)

        ego_rotation = self.vehicle.get_transform().rotation
        ego_rotation = fix_rotations(ego_rotation)

        ego_rotation = get_quaternion_from_euler(ego_rotation.pitch, ego_rotation.yaw, ego_rotation.roll)
        ego_data = {
            "translation": ego_transform,
            "rotation": ego_rotation,
        }

        if is_key_frame:
            output = "samples"
        else:
            output = "sweeps"

        for sensor in self.sensors_active:
            channel = self.sensors_active[sensor]  # Sensor name (CAM_FRONT, CAM_DRONE or LIDAR)
            sensor_out = self.sensors_output[channel]  # Image or point cloud
            modality = self.sensor_list[channel]  # Sensor type(camera or Lidar)

            if modality == "camera":
                image = sensors_output[channel]
                image_path = os.path.join(self.output_dir, output, str(channel))
                if not os.path.isdir(image_path):
                    os.makedirs(image_path)

                # Compress and save image
                # qualities = [25, 50, 75, 100]

                img_array = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
                imageRGB = cv.cvtColor(img_array, cv.COLOR_BGR2RGB)
                img_out = Image.fromarray(imageRGB)
                img_out = img_out.convert('RGB')

                # for q in qualities:
                image_filename = os.path.join(image_path, self.scene_number + "_" + str(sample_clock) + ".jpg")
                image_filename = image_filename.replace("\\", "/")
                img_out.save(image_filename, optimize=True, quality=75)

                # image_filename = os.path.join(image_path, scene_number + "_" + str(sample_clock) + "_original_" + ".jpg")
                # image.save_to_disk(image_filename)

                output_path = os.path.join(output, str(channel), self.scene_number + "_" + str(sample_clock) + ".jpg")

                sensor_data_dict = {
                    "sensor": channel,
                    "fileformat": ".jpg",
                    "filename": output_path,
                    "height": sensor_out.height,
                    "width": sensor_out.width,
                    "lidarseg": None
                }
                sensor_sample_data.append(sensor_data_dict)

            elif channel == "LIDAR_TOP":
                # point_cloud = sensors_output[channel]
                # data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4, f4, f4, f4, int, int')))
                #
                # locations = np.empty((0, 5))
                # tags = np.empty((0, 1), dtype=np.uint8)
                # intensity = 0
                # ring = 0
                #
                # for point in data:
                #     locations = np.append(locations, np.array([[-point[0], point[1], point[2], intensity, ring]]),
                #                           axis=0)
                #     tag = self.tags_dict[point[5]]
                #     tags = np.append(tags, np.array([[tag]]), axis=0)

                point_cloud = sensors_output[channel]
                dtype_size = np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4'), ('ring', int), ('tag', int)]).itemsize
                buffer_szie = len(point_cloud.raw_data) // dtype_size * dtype_size
                data = np.frombuffer(point_cloud.raw_data[:buffer_szie], dtype=np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4'), ('ring', int), ('tag', int)]))
                data = data.reshape(-1, 1)

                locations = np.empty((data.shape[0], 5))
                locations[:, 0] = -data['x'].flatten()
                locations[:, 1] = data['y'].flatten()
                locations[:, 2] = data['z'].flatten()
                locations[:, 3] = 0
                locations[:, 4] = 0

                tags = np.array([self.tags_dict[tag[0]] for tag in data['tag']], dtype=np.uint8)

                # Save lidar point cloud to folder
                if is_key_frame:
                    output = "samples"
                else:
                    output = "sweeps"

                lidar_path = os.path.join(self.output_dir, output, str(channel))
                lidar_filename = os.path.join(lidar_path, self.scene_number + "_" + str(sample_clock) + ".pcd.bin")
                lidar_filename = lidar_filename.replace("\\", "/")
                output_path = os.path.join(output, str(channel), self.scene_number + "_" + str(sample_clock) + ".pcd.bin")
                if not os.path.isdir(lidar_path):
                    os.makedirs(lidar_path)

                np.asarray(locations).astype('float32').tofile(lidar_filename)

                # Save lidar annotations
                lidarseg_path = None
                if is_key_frame:
                    lidar_path = os.path.join(self.output_dir, "lidarseg", "horus-" + self.tt)
                    if not os.path.isdir(lidar_path):
                        os.makedirs(lidar_path)
                    lidar_filename = os.path.join(lidar_path, self.scene_number + "_0_" + str(sample_clock) + ".pcd.bin")
                    lidarseg_path = os.path.join("lidarseg", "horus-trainval",
                                                 self.scene_number + "_0_" + str(sample_clock) + ".pcd.bin")
                    np.asarray(tags).astype('uint8').tofile(lidar_filename)

                sensor_data_dict = {
                    "sensor": channel,
                    "fileformat": ".pcd",
                    "filename": output_path,
                    "height": "",
                    "width": "",
                    "lidarseg": lidarseg_path
                }
                sensor_sample_data.append(sensor_data_dict)

        if self.K_drone_4k is not None:
            # Save Depth Output of 400m Drone
            channel = "CAM_DRONE_400_4K"
            d_image = depth[channel]

            image_path = os.path.join(self.output_dir, output, str(channel) + "_DEPTH")
            if not os.path.isdir(image_path):
                os.makedirs(image_path)

            img_array = np.reshape(np.copy(d_image.raw_data), (d_image.height, d_image.width, 4))
            imageRGB = cv.cvtColor(img_array, cv.COLOR_BGR2RGB)
            img_out = Image.fromarray(imageRGB)
            img_out = img_out.convert('RGB')
            image_filename = os.path.join(image_path, self.scene_number + "_" + str(sample_clock) + ".jpg")
            img_out.save(image_filename)

        if is_key_frame:
            instances, instance_locations, instance_rotations, instance_size, instance_car_vis, instance_drone_vis = \
                self.find_objects()
            for i in range(len(instances)):
                object_dict = {
                    "instance": instances[i],
                    "location": instance_locations[i],
                    "rotation": instance_rotations[i],
                    "size": instance_size[i],
                    "car_vis": instance_car_vis[i],
                    "drone_vis": instance_drone_vis[i]
                }
                objects.append(object_dict)

        else:
            objects.append(None)

        self.data_out.process_sample(sample_clock, ego_data, sensor_sample_data, objects, is_key_frame, start_scene,
                                     end_scene)

    def find_objects(self):
        instances = []
        instance_locations = []
        instance_rotations = []
        instance_size = []
        instance_car_vis = []
        instance_drone_vis = []

        all_vehicles = {}
        moving_vehicles = {}
        annotated_vehicles = {}

        all_filter = {}
        # th = 50

        # Find moving vehicles
        for npc in self.world.get_actors().filter('*vehicle*'):
            # dist = npc.get_transform().location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            loc = (int(npc.get_transform().location.x), int(npc.get_transform().location.y))
            type_id = (npc.id, npc.type_id)
            moving_vehicles[(loc, npc)] = type_id
            # moving_vehicles[npc] = type_id

        # Find all cars
        for bb in self.world.get_level_bbs(carla.CityObjectLabel.Car):
            # dist = bb.location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            all_vehicles[bb] = "car"
            loc = (int(bb.location.x), int(bb.location.y))
            all_filter[loc] = bb

        # Find all trucks
        for bb in self.world.get_level_bbs(carla.CityObjectLabel.Truck):
            # dist = bb.location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            all_vehicles[bb] = "truck"
            loc = (int(bb.location.x), int(bb.location.y))
            all_filter[loc] = bb

        # Find all buses
        for bb in self.world.get_level_bbs(carla.CityObjectLabel.Bus):
            # dist = bb.location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            all_vehicles[bb] = "bus"
            loc = (int(bb.location.x), int(bb.location.y))
            all_filter[loc] = bb

        # Find all motorcycles
        for bb in self.world.get_level_bbs(carla.CityObjectLabel.Motorcycle):
            # dist = bb.location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            # Bug in Carla 0.9.14 --> some bicycle bboxes don't have correct dimensions
            bb.extent.x = 1.104723
            bb.extent.y = 0.401290

            all_vehicles[bb] = "motorcycle"
            loc = (int(bb.location.x), int(bb.location.y))
            all_filter[loc] = bb

        # Find all bikes
        for bb in self.world.get_level_bbs(carla.CityObjectLabel.Bicycle):
            # dist = bb.location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            # Bug in Carla 0.9.14 --> some bicycle bboxes don't have correct dimensions
            bb.extent.x = 0.821422
            bb.extent.y = 0.186258
            bb.extent.z = 0.511951

            all_vehicles[bb] = "bicycle"
            loc = (int(bb.location.x), int(bb.location.y))
            all_filter[loc] = bb

        # Determine which of all vehicles are moving and assign correct ID
        for mov_loc, id_mov in moving_vehicles.items():
            for bb_loc, bb_all in all_filter.items():
                if 0 <= distance.euclidean(mov_loc[0], bb_loc) < 1.5:  # For some reason trucks have an offset with the bbox
                    # Label ego vehicle
                    if id_mov[0] == self.vehicle.id:
                        real_id = "ego"
                    else:
                        real_id = all_vehicles[bb_all]

                    type_id_out = (id_mov[0], real_id, 'moving')

                    npc_loc = (mov_loc[1].get_transform().location.x, mov_loc[1].get_transform().location.y)
                    bb_all.location.x = npc_loc[0]
                    bb_all.location.y = npc_loc[1]

                    annotated_vehicles[bb_all] = type_id_out
                else:
                    if bb_all not in annotated_vehicles:
                        if self.parked_ids:
                            if bb_loc not in self.parked_ids.keys():
                                bb_id = self.parked_ids[list(self.parked_ids)[-1]] - 1
                                self.parked_ids[bb_loc] = bb_id
                            else:
                                bb_id = self.parked_ids[bb_loc]
                        else:
                            bb_id = -1
                            self.parked_ids[bb_loc] = bb_id

                        type_id_out = all_vehicles[bb_all]
                        annotated_vehicles[bb_all] = (bb_id, type_id_out, 'parked')

        # Check occlusion and append
        for bb, bb_id in annotated_vehicles.items():
            size = [bb.extent.y * 2, bb.extent.x * 2, bb.extent.z * 2]  # width = y, length = x, height = z

            occ_drone, vis_drone = filter_occluded(self.drone_cameras, bb.location, size, self.depth, self.image_w_drone,
                                                   self.image_h_drone, self.image_w_drone_4k, self.image_h_drone_4k,
                                                   self.K_drone, self.K_drone_4k)
            occ_car, vis_car = get_visiblity_ego(self.car_cameras, bb.location, size, self.depth, self.image_w_car,
                                                 self.image_h_car, self.K_car, self.vehicle)

            if not occ_drone or not occ_car:
                location = get_loc(bb.location)
                rotation = fix_rotations(bb.rotation)
                rotation = get_quaternion_from_euler(rotation.pitch, rotation.yaw, rotation.roll)

                instances.append(bb_id)
                instance_locations.append(location)
                instance_rotations.append(rotation)
                instance_size.append(size)
                instance_car_vis.append(vis_car)
                instance_drone_vis.append(vis_drone)

        # Find Pedestrians
        for npc in self.world.get_actors().filter('*pedestrian*'):
            # dist = npc.get_transform().location.distance(self.vehicle.get_transform().location)
            # if dist < th:
            bb = npc.bounding_box
            location = npc.get_transform().location
            size = [bb.extent.y * 2, bb.extent.x * 2, bb.extent.z * 2]  # width = y, length = x, height = z

            occ_drone, vis_drone = filter_occluded(self.drone_cameras, location, size, self.depth, self.image_w_drone,
                                                   self.image_h_drone, self.image_w_drone_4k, self.image_h_drone_4k,
                                                   self.K_drone, self.K_drone_4k)
            occ_car, vis_car = get_visiblity_ego(self.car_cameras, location, size, self.depth, self.image_w_car,
                                                 self.image_h_car, self.K_car, self.vehicle)
            if not occ_drone or not occ_car:
                instances.append((npc.id, 'pedestrian', 'pedestrian'))
                location = get_loc(location)
                rotation = npc.get_transform().rotation
                rotation = fix_rotations(rotation)
                rotation = get_quaternion_from_euler(rotation.pitch, rotation.yaw, rotation.roll)

                instance_locations.append(location)
                instance_rotations.append(rotation)
                instance_size.append(size)
                instance_car_vis.append(vis_car)
                instance_drone_vis.append(vis_drone)

        return instances, instance_locations, instance_rotations, instance_size, instance_car_vis, instance_drone_vis


def get_loc(location):
    return [-location.x, location.y, location.z]


def fix_rotations(object):
    if object.yaw != -90:
        if (object.yaw < 0) and (object.yaw > -180):
            error = object.yaw + 90
            object.yaw = object.yaw - 2 * error
        if (object.yaw > 0) and (object.yaw < 180):
            error = object.yaw - 90
            object.yaw = object.yaw - 2 * error

    return object


def get_num_points(point_cloud):
    num_points = 0
    channels = point_cloud.channels
    for i in range(channels):
        num_points += point_cloud.get_point_count(i+1)

    return num_points


# def build_projection_matrix(w, h, fov):
#     focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
#     K = np.identity(3)
#     K[0, 0] = K[1, 1] = focal
#     K[0, 2] = w / 2.0
#     K[1, 2] = h / 2.0
#     return K

# def sweep_data(sensors_active, sensor_list, sensors_output, output_dir, scene_number):
#     timestamp = int(time.time())
#     for sensor in sensors_active:
#         channel = sensors_active[sensor]  # Sensor name (CAM_FRONT, CAM_DRONE or LIDAR)
#         modality = sensor_list[channel]  # Sensor type(camera or Lidar)
#
#         if modality == "camera":
#             image = sensors_output[channel]
#             image_path = os.path.join(output_dir, "sweeps", str(channel) + "/")
#             image.save_to_disk(image_path + scene_number + "_" + str(timestamp) + "_" + '%.6d.jpg' % image.frame)