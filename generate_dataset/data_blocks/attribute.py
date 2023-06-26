from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Attribute:
    def __init__(self, token, name, description):
        self.token = token
        self.name = name
        self.description = description

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "name": self.name,
            "description": self.description
        }]

        output_file = "attribute.json"
        export_json(output_file, output_dict, tt)
