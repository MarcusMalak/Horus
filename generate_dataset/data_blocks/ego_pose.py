from data_blocks.utils.generate_token import get_token
from typing import List
from data_blocks.utils.export_json import export_json


class EgoPose:
    def __init__(self, token: str, timestamp: int, rotation: List[float], translation: List[float]):
        self.token = token
        self.timestamp = timestamp
        self.rotation = rotation
        self.translation = translation

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "timestamp": self.timestamp,
            "rotation": self.rotation,
            "translation": self.translation
        }]

        output_file = "ego_pose.json"
        export_json(output_file, output_dict, tt)
