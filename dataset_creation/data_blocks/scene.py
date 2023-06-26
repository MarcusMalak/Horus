from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json
from datetime import datetime

output_dir = "data_output"


class SceneExport:
    def __init__(self, name: str):
        self.token = get_token()
        self.log_token = ""
        self.nbr_samples = 0
        self.first_sample_token = ""
        self.last_sample_token = ""
        self.name = name
        self.description = ""
        self.instances = []

    def create_log(self, token: str, location: str):
        log = self.Log(token, location)
        return log

    def create_sample(self, token: str, timestamp: int, prev: str, next: str):
        new_sample = self.Sample(token, timestamp, prev, next, self.token)
        return new_sample

    def export_scene(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "log_token": self.log_token,
            "nbr_samples": self.nbr_samples,
            "first_sample_token": self.first_sample_token,
            "last_sample_token": self.last_sample_token,
            "name": self.name,
            "description": self.description
        }]

        output_file = "scene.json"
        export_json(output_file, output_dict, tt)

    class Log:
        def __init__(self, token, location):
            self.token = token
            self.logfile = ""
            self.vehicle = "drone + vehicle"
            self.date_captured = datetime.today().strftime('%Y-%m-%d')
            self.location = location

        def export(self, tt):
            # Data to write
            output_dict = [{
                "token": self.token,
                "logfile": self.logfile,
                "vehicle": self.vehicle,
                "date_captured": self.date_captured,
                "location": self.location
            }]

            output_file = "log.json"
            export_json(output_file, output_dict, tt)

    class Sample:
        def __init__(self, token: str, timestamp: int, prev: str, next: str, scene_token: str):
            self.token = token
            self.timestamp = timestamp
            self.prev = prev
            self.next = next
            self.scene_token = scene_token

        def export(self, tt):
            # Data to write
            output_dict = [{
                "token": self.token,
                "timestamp": self.timestamp,
                "prev": self.prev,
                "next": self.next,
                "scene_token": self.scene_token
            }]

            output_file = "sample.json"
            export_json(output_file, output_dict, tt)