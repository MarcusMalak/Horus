from data_blocks.scene import SceneExport
from data_blocks.sensor import Sensors
from data_blocks.category import Category
from data_blocks.sample_data import SampleData
from data_blocks.sample_annotation import SampleAnnotation
from data_blocks.calibrated_sensor import CalibratedSensor
from data_blocks.ego_pose import EgoPose
from data_blocks.instance import Instance
from data_blocks.attribute import Attribute
from data_blocks.visibility import Visibility
from data_blocks.map import Map
from data_blocks.lidarseg import Lidarseg
from data_blocks.utils.export_json import update_maps
from data_blocks.utils.sample_annot_export import export_json_mod

from data_blocks.utils.generate_token import get_token
from utils.camera_projection import get_camera_intrinsic
from utils.quaternion import get_quaternion_from_euler
from utils.sensor_projection import sensor_2_ego

import random
import os
import time

class DataExport:
    def __init__(self, scene: str, sensor_list: dict, sensors_active: dict, seed: int, attributes: dict,
                 categories: list, visibilities: dict, tt: str):
        self.seed = seed
        # random.seed(self.seed)

        self.scene = None
        self.scene_name = scene
        self.sensor_list = sensor_list
        self.sensors_active = sensors_active
        self.attributes = attributes
        self.categories = categories
        self.visibilities = visibilities
        self.tt = tt

        self.sensor_tokens = {}         # Dict with all sensor tokens (sensor_channel = key, sensor_token = value)
        self.calibrated_tokens = {}    # Calibrated sensor tokens
        self.sensor_sample_data = {}    # Dict with sample data (sensor_token = key, sample_data = value (list)
        self.annotations = {}  # Dict with all instances and corresponding annotation objects
        self.instances = {}     # Dict with instance tokens
        self.attribute_tokens = {}
        self.category_tokens = {}

        self.sample_token = ""
        self.prev_sample_token = ""
        self.next_sample_token = ""
        self.first_sample = ""
        self.last_sample = ""

    def create_scene(self):
        self.scene = SceneExport(self.scene_name)

    def create_maps(self, maps):
        for map in maps:
            token = get_token()
            filename = os.path.join('maps', map + '.jpg')
            filename = filename.replace("\\", "/")
            map = Map(token, filename)
            map.export(self.tt)

    def update_maps(self, maps):
        for map in maps:
            token = get_token()
            filename = os.path.join('maps', map + '.jpg')
            filename = filename.replace("\\", "/")
            map = Map(token, filename)


    def create_log(self, map):
        token = get_token()
        log = self.scene.create_log(token, map)
        self.scene.log_token = token
        log.export(self.tt)

        update_maps("map.json", map, token, self.tt)

        # if map in self.log_tokens:
        #     map_tokens = self.log_tokens[map]
        #     map_tokens.append(token)
        #     self.log_tokens[map] = map_tokens
        # else:
        #     self.log_tokens[map] = [token]

    def create_sensors(self):
        for channel, modality in self.sensor_list.items():
            sensor = Sensors(channel, modality)
            self.sensor_tokens[channel] = sensor.token
            sensor.export(self.tt)

    def create_categories(self):
        for index, name in enumerate(self.categories):
            token = get_token()
            category = Category(token, name, name, index)
            self.category_tokens[name] = token
            category.export(self.tt)

    def create_attributes(self):
        for name, description in self.attributes.items():
            token = get_token()
            attribute = Attribute(token, name, description)
            self.attribute_tokens[name] = token
            attribute.export(self.tt)

    def create_visibility(self):
        for index, level in enumerate(self.visibilities):
            token = str(index)
            description = self.visibilities[level]
            visibility = Visibility(token, level, description)
            visibility.export(self.tt)

    def calibrated_sensors(self, camera_bp_drone, camera_bp_drone_4k, camera_bp_car, cam_drone_pos, cam_front,
                           cam_drone_h, lidar_pos, width_drone, height_drone, width_drone_4k, height_drone_4k,
                           width_car, height_car):
        for i, sensor in enumerate(self.sensors_active.keys()):
            channel = self.sensors_active[sensor]    # Sensor name (CAM_FRONT, CAM_DRONE or LIDAR)
            modality = self.sensor_list[channel]     # Sensor type(camera or Lidar)

            if modality == "camera":
                if channel == "CAM_FRONT":
                    sensor_trans = [cam_front[0], cam_front[1], cam_front[2]]
                    sensor_rotation = [90, -90, 0]      # FRONT
                    camera_intrinsic = get_camera_intrinsic(camera_bp_car, width_car, height_car)
                elif channel == "CAM_DRONE_400_4K":
                    sensor_trans = [cam_drone_pos[0], cam_drone_pos[1], cam_drone_h[channel]]
                    sensor_rotation = [180, -90, 0]   # FRONT
                    camera_intrinsic = get_camera_intrinsic(camera_bp_drone_4k, width_drone_4k, height_drone_4k)
                else:
                    sensor_trans = [cam_drone_pos[0], cam_drone_pos[1], cam_drone_h[channel]]
                    sensor_rotation = [180, -90, 0]     # TOP
                    camera_intrinsic = get_camera_intrinsic(camera_bp_drone, width_drone, height_drone)

                sensor_rotation = get_quaternion_from_euler(sensor_rotation[0], sensor_rotation[1],
                                                            sensor_rotation[2])
            elif channel == "LIDAR_TOP":
                sensor_trans = lidar_pos
                # sensor_rotation = [-180, 0, -180]
                # sensor_rotation = get_quaternion_from_euler(sensor_rotation[0], sensor_rotation[1],
                #                                             sensor_rotation[2])
                # sensor_rotation = [0, 0, 0, 1]
                sensor_rotation = [0.707, 0, 0, -0.707]

            elif channel == "LIDAR_SEM":
                break

            if modality != "camera":
                camera_intrinsic = []

            calibrated_token = get_token()
            self.calibrated_tokens[channel] = calibrated_token

            calibrated_sensor = CalibratedSensor(calibrated_token, self.sensor_tokens[channel], sensor_trans,
                                                 sensor_rotation, camera_intrinsic)

            calibrated_sensor.export(self.tt)

    def process_sample(self, timestamp, ego_data, sensor_sample_data, objects, is_key_frame, start_scene, end_scene):
        if is_key_frame:
            self.sample_token = self.next_sample_token
            if start_scene:
                self.sample_token = get_token()
                self.scene.first_sample_token = self.sample_token

            if not end_scene:
                self.next_sample_token = get_token()
            elif end_scene:
                self.next_sample_token = ""
                self.scene.last_sample_token = self.sample_token

            # Create sample (which contains one or more sample_data from each sensor)
            sample = self.scene.create_sample(self.sample_token, timestamp, self.prev_sample_token, self.next_sample_token)
            self.prev_sample_token = self.sample_token

            self.scene.nbr_samples += 1

            sample.export(self.tt)

        # Make sure sweep gets sample token that is closest to keyframe 1,2,3 are prev and 4 takes next
        if (timestamp + 1) % 5 == 0 and timestamp != 0:
            if self.next_sample_token != "":
                self.sample_token = self.next_sample_token

        # Create and export ego pose
        ego_pose_token = get_token()
        ego_pose = EgoPose(ego_pose_token, timestamp, ego_data["rotation"], ego_data["translation"])
        ego_pose.export(self.tt)

        # Cycle through all sensors in current sample and create sample_data
        for data in sensor_sample_data:
            sensor_token = self.sensor_tokens[data["sensor"]]
            if sensor_token in self.sensor_sample_data:
                sample_data_list = self.sensor_sample_data[sensor_token]
                if end_scene:
                    next_sample_data_token = ""
                else:
                    next_sample_data_token = get_token()

                sample_data = SampleData(token=sample_data_list[-1].next,
                                         sample_token=self.sample_token,
                                         ego_pose_token=ego_pose_token,
                                         calibrated_sensor_token=self.calibrated_tokens[data["sensor"]],
                                         timestamp=timestamp,
                                         fileformat=data["fileformat"],
                                         is_key_frame=is_key_frame,
                                         height=data["height"],
                                         width=data["width"],
                                         filename=data["filename"],
                                         prev=sample_data_list[-1].token,
                                         next=next_sample_data_token)
                self.sensor_sample_data[sensor_token].extend([sample_data])
            else:
                sample_data_token = get_token()
                if end_scene:
                    next_sample_data_token = ""
                else:
                    next_sample_data_token = get_token()

                sample_data = SampleData(token=sample_data_token,
                                         sample_token=self.sample_token,
                                         ego_pose_token=ego_pose_token,
                                         calibrated_sensor_token=self.calibrated_tokens[data["sensor"]],
                                         timestamp=timestamp,
                                         fileformat=data["fileformat"],
                                         is_key_frame=is_key_frame,
                                         height=data["height"],
                                         width=data["width"],
                                         filename=data["filename"],
                                         prev="",
                                         next=next_sample_data_token)
                self.sensor_sample_data[sensor_token] = [sample_data]

            if data["lidarseg"] is not None:
                lidarseg = Lidarseg(sample_data.token, sample_data.token, data["lidarseg"])
                lidarseg.export(self.tt)

        if is_key_frame:
            for object in objects:
                instance = object["instance"]
                location = object["location"]
                rotation = object["rotation"]
                size = object["size"]
                car_vis = object["car_vis"]
                drone_vis = object["drone_vis"]

                num_lidar_points = 0

                if instance[2] == "moving":
                    attribute_token = self.attribute_tokens["vehicle.moving"]
                elif instance[2] == "parked":
                    attribute_token = self.attribute_tokens["vehicle.parked"]
                elif instance[2] == "pedestrian":
                    attribute_token = self.attribute_tokens["pedestrian.moving"]

                if instance in self.annotations:
                    sample_annotation_list = self.annotations[instance]
                    annotation_token = sample_annotation_list[-1].next

                    if end_scene:
                        next_annotation_token = ""
                    else:
                        next_annotation_token = get_token()

                    instance_token = self.instances[instance]
                    car_vis_token = self.choose_vis_token(car_vis)
                    drone_vis_token = self.choose_vis_token(drone_vis)

                    # # If parked vehicle always make new token as there is no way to id parked vehicle in carla
                    # if instance == (-1, 'vehicle.parked'):
                    #     annotation_token = get_token()
                    # else:
                    #     annotation_token = sample_annotation_list[-1].next

                    sample_annotation = SampleAnnotation(token=annotation_token,
                                                         sample_token=self.sample_token,
                                                         instance_token=instance_token,
                                                         attribute_token=attribute_token,
                                                         car_vis=car_vis_token,
                                                         drone_vis=drone_vis_token,
                                                         translation=location,
                                                         size=size,
                                                         rotation=rotation,
                                                         num_lidar_points=num_lidar_points,
                                                         next=next_annotation_token,
                                                         prev=sample_annotation_list[-1].token)
                    self.annotations[instance].extend([sample_annotation])
                else:
                    annotation_token = get_token()
                    instance_token = get_token()
                    car_vis_token = self.choose_vis_token(car_vis)
                    drone_vis_token = self.choose_vis_token(drone_vis)

                    if end_scene:
                        next_annotation_token = ""
                    else:
                        next_annotation_token = get_token()

                    sample_annotation = SampleAnnotation(token=annotation_token,
                                                         sample_token=self.sample_token,
                                                         instance_token=instance_token,
                                                         attribute_token=attribute_token,
                                                         car_vis=car_vis_token,
                                                         drone_vis=drone_vis_token,
                                                         translation=location,
                                                         size=size,
                                                         rotation=rotation,
                                                         num_lidar_points=num_lidar_points,
                                                         next=next_annotation_token,
                                                         prev="")
                    self.annotations[instance] = [sample_annotation]
                    self.instances[instance] = instance_token

    # def create_instances(self):
    #     for instance, annotations in self.annotations.items():
    #         token = self.instances[instance]
    #         category_carla = instance[1]
    #         category = None
    #
    #         if category_carla == 'ego':
    #             category = 'vehicle.ego'
    #         elif category_carla == 'car':
    #             category = 'vehicle.car'
    #         elif category_carla == 'truck':
    #             category = 'vehicle.truck'
    #         elif category_carla == 'bus':
    #             category = 'vehicle.bus.rigid'
    #         elif category_carla == 'motorcycle':
    #             category = 'vehicle.motorcycle'
    #         elif category_carla == 'bicycle':
    #             category = 'vehicle.bicycle'
    #         elif category_carla == 'vehicle.parked':
    #             category = 'vehicle.parked'
    #         elif category_carla == 'pedestrian':
    #             category = 'human.pedestrian.adult'
    #
    #         category_token = self.category_tokens[category]
    #         nbr_annot = len(annotations)
    #         first_annot = annotations[0]
    #         last_annot = annotations[-1]
    #         instance_block = Instance(token, category_token, nbr_annot, first_annot.token, last_annot.token)
    #         instance_block.export(self.tt)

    def create_instance(self, instance, annotations):
        token = self.instances[instance]
        category_carla = instance[1]
        category = None

        if category_carla == 'ego':
            category = 'vehicle.ego'
        elif category_carla == 'car':
            category = 'vehicle.car'
        elif category_carla == 'truck':
            category = 'vehicle.truck'
        elif category_carla == 'bus':
            category = 'vehicle.bus.rigid'
        elif category_carla == 'motorcycle':
            category = 'vehicle.motorcycle'
        elif category_carla == 'bicycle':
            category = 'vehicle.bicycle'
        elif category_carla == 'vehicle.parked':
            category = 'vehicle.parked'
        elif category_carla == 'pedestrian':
            category = 'human.pedestrian.adult'

        category_token = self.category_tokens[category]
        nbr_annot = len(annotations)
        first_annot = annotations[0]
        last_annot = annotations[-1]
        output_dict = {
            "token": token,
            "category_token": category_token,
            "nbr_annotations": nbr_annot,
            "first_annotation_token": first_annot.token,
            "last_annotation_token": last_annot.token
        }

        return output_dict

    def export_all(self):
        start_time = time.time()
        # Export sample_data
        sample_data_list = []
        for sensor, sample_data in self.sensor_sample_data.items():
            for data in sample_data:
                output_dict = {
                    "token": data.token,
                    "sample_token": data.sample_token,
                    "ego_pose_token": data.ego_pose_token,
                    "calibrated_sensor_token": data.calibrated_sensor_token,
                    "timestamp": data.timestamp,
                    "fileformat": data.fileformat,
                    "is_key_frame": data.is_key_frame,
                    "height": data.height,
                    "width": data.width,
                    "filename": data.filename,
                    "prev": data.prev,
                    "next": data.next
                }
                sample_data_list.append(output_dict)
                # data.export(self.tt)
        export_json_mod(sample_data_list, self.tt, "sample_data.json")

        # Export sample_annotation
        export_instance_list = []
        export_annotation_list = []
        count = 0
        for instance, annotation in self.annotations.items():
            count += 1
            num_annot = len(annotation)
            output_dict_inst = self.create_instance(instance, annotation)
            export_instance_list.append(output_dict_inst)
            for i, sample_annotation in enumerate(annotation):
                if i == num_annot - 1:
                    sample_annotation.next = ""
                output_dict_ann = {
                    "token": sample_annotation.token,
                    "sample_token": sample_annotation.sample_token,
                    "instance_token": sample_annotation.instance_token,
                    "visibility_token": sample_annotation.visibility_token,
                    "visibility_token_drone": sample_annotation.visibility_token_drone,
                    "attribute_tokens": sample_annotation.attribute_tokens,
                    "translation": sample_annotation.translation,
                    "size": sample_annotation.size,
                    "rotation": sample_annotation.rotation,
                    "prev": sample_annotation.prev,
                    "next": sample_annotation.next,
                    "num_lidar_pts": sample_annotation.num_lidar_points,
                    "num_radar_pts": sample_annotation.num_radar_points
                }
                export_annotation_list.append(output_dict_ann)

        export_json_mod(export_annotation_list, self.tt, "sample_annotation.json")
        export_json_mod(export_instance_list, self.tt, "instance.json")

        end_time = time.time()
        dt = end_time - start_time
        print(f'\n TIME TAKEN EXPORT: {dt}')

        self.scene.export_scene(self.tt)

    def reset(self):
        self.scene = None
        self.sensor_sample_data = {}
        self.annotations = {}
        self.instances = {}
        self.sample_token = ""
        self.prev_sample_token = ""
        self.next_sample_token = ""
        self.first_sample = ""
        self.last_sample = ""

    @staticmethod
    def get_loc(location):
        return [-location.x, location.y, location.z]

    @staticmethod
    def choose_vis_token(vis):
        if vis == 0:
            token = "0"
        elif vis < 0.4:
            token = "1"
        elif 0.4 < vis < 0.6:
            token = "2"
        elif 0.6 < vis < 0.8:
            token = "3"
        else:
            token = "4"
        return token
