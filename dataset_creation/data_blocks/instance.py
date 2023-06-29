from data_blocks.utils.generate_token import get_token
from data_blocks.utils.export_json import export_json


class Instance:
    def __init__(self, token: str, category_token: str, nbr_annotations: int, first_annotation: str, last_annotation: str):
        self.token = token
        self.category_token = category_token
        self.nbr_annotations = nbr_annotations
        self.first_annotation_token = first_annotation
        self.last_annotation_token = last_annotation

    def export(self, tt):
        # Data to write
        output_dict = [{
            "token": self.token,
            "category_token": self.category_token,
            "nbr_annotations": self.nbr_annotations,
            "first_annotation_token": self.first_annotation_token,
            "last_annotation_token": self.last_annotation_token
        }]

        output_file = "instance.json"
        export_json(output_file, output_dict, tt)


