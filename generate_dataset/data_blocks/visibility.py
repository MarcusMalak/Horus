from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Visibility:
    def __init__(self, token: str, level: str, description: str):
        self.token = token
        self.level = level
        self.description = description

    def export(self, tt):
        # Data to write
        output_dict = [{
            "description": self.description,
            "token": self.token,
            "level": self.level
        }]

        output_file = "visibility.json"
        export_json(output_file, output_dict, tt)
