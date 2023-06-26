from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Lidarseg:
    def __init__(self, token, sample_data, filename):
        self.token = token
        self.sample_data_token = sample_data
        self.filename = filename

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "sample_data_token": self.sample_data_token,
            "filename": self.filename
        }]

        output_file = "lidarseg.json"
        export_json(output_file, output_dict, tt)
