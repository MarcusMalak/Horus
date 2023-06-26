from data_blocks.utils.generate_token import get_token
from typing import List
from data_blocks.utils.export_json import export_json


class SampleAnnotation:
    def __init__(self, token: str, sample_token: str, instance_token: str, attribute_token: str, car_vis: str,
                 drone_vis: str, translation: List[float], size: List[float], rotation: List[float],
                 num_lidar_points: int, next: str, prev: str):

        self.token = token
        self.sample_token = sample_token
        self.instance_token = instance_token
        self.attribute_tokens = [attribute_token]
        self.visibility_token = car_vis
        self.visibility_token_drone = drone_vis
        self.translation = translation
        self.size = size
        self.rotation = rotation
        self.next = next
        self.prev = prev
        self.num_lidar_points = num_lidar_points
        self.num_radar_points = 0

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "sample_token": self.sample_token,
            "instance_token": self.instance_token,
            "visibility_token": self.visibility_token,
            "visibility_token_drone": self.visibility_token_drone,
            "attribute_tokens": self.attribute_tokens,
            "translation": self.translation,
            "size": self.size,
            "rotation": self.rotation,
            "prev": self.prev,
            "next": self.next,
            "num_lidar_pts": self.num_lidar_points,
            "num_radar_pts": self.num_radar_points
        }]

        output_file = "sample_annotation.json"
        export_json(output_file, output_dict, tt)

