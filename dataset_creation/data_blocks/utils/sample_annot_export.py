import ujson
import os
from os import path

data_output = r'D:\horus'
# data_output = 'data_output/mini_set_2'


def export_json_mod(output_dict, traintest, output_file):
    output_dir = os.path.join(data_output, "horus-" + traintest)
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    output_path = os.path.join(output_dir, output_file)

    if path.exists(output_path):
        with open(output_path) as file:
            file_data = ujson.load(file)
            file_data.extend(output_dict)
        with open(output_path, 'w') as new_file:
            ujson.dump(file_data, new_file, indent=4)
    else:
        # Serialize json
        json_output = ujson.dumps(output_dict, indent=4)
        # Write to .json file
        with open(output_path, "w") as file:
            file.write(json_output)
