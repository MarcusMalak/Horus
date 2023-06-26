from data_blocks.utils.generate_token import get_token
from typing import List
from data_blocks.utils.export_json import export_json


class SampleData:
    def __init__(self, token: str, sample_token: str, ego_pose_token: str, calibrated_sensor_token: str, timestamp: int,
                 fileformat: str, is_key_frame: bool, height: int, width: int, filename: str, prev: str, next: str):
        self.token = token
        self.sample_token = sample_token
        self.ego_pose_token = ego_pose_token
        self.calibrated_sensor_token = calibrated_sensor_token
        self.timestamp = timestamp
        self.fileformat = fileformat
        self.is_key_frame = is_key_frame
        self.height = height
        self.width = width
        self.filename = filename
        self.prev = prev
        self.next = next

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "sample_token": self.sample_token,
            "ego_pose_token": self.ego_pose_token,
            "calibrated_sensor_token": self.calibrated_sensor_token,
            "timestamp": self.timestamp,
            "fileformat": self.fileformat,
            "is_key_frame": self.is_key_frame,
            "height": self.height,
            "width": self.width,
            "filename": self.filename,
            "prev": self.prev,
            "next": self.next
        }]

        output_file = "sample_data.json"
        export_json(output_file, output_dict, tt)


