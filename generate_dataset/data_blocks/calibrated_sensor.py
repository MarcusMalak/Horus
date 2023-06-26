from data_blocks.utils.generate_token import get_token
from typing import List
from data_blocks.utils.export_json import export_json


class CalibratedSensor:
    def __init__(self, calibrated_token: str, sensor_token: str, translation: List[float], rotation: List[float], camera_intrinsic):
        self.token = calibrated_token
        self.sensor_token = sensor_token
        self.translation = translation
        self.rotation = rotation
        self.camera_intrinsic = camera_intrinsic

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "sensor_token": self.sensor_token,
            "translation": self.translation,
            "rotation": self.rotation,
            "camera_intrinsic": self.camera_intrinsic
        }]

        output_file = "calibrated_sensor.json"
        export_json(output_file, output_dict, tt)
