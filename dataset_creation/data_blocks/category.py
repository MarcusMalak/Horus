from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Category:
    def __init__(self, token: str, name: str, description: str, index: int):
        self.token = token
        self.name = name
        self.description = description
        self.index = index

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "name": self.name,
            "description": self.description,
            "index": self.index
        }]

        output_file = "category.json"
        export_json(output_file, output_dict, tt)
