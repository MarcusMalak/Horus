import ujson
import os

data_output = r'D:\horus\horus-trainval'
json_file = 'sample_data.json'


def edit_paths(json_file):
    sample_data_dir = os.path.join(data_output, json_file)
    with open(sample_data_dir, 'r') as file:
        file_data = ujson.load(file)
        for i, d in enumerate(file_data):
            filename = d["filename"]
            d["filename"] = filename.replace('\/', '/')
            print(d["filename"])
            # print(i)

    with open(sample_data_dir, 'w') as file:
        ujson.dump(file_data, file, indent=0, escape_forward_slashes=False)


edit_paths(json_file)