from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Sensors:
    def __init__(self, channel: str, modality: str):
        self.token = get_token()
        self.channel = channel
        self.modality = modality

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "channel": self.channel,
            "modality": self.modality
        }]

        output_file = "sensor.json"
        export_json(output_file, output_dict, tt)
