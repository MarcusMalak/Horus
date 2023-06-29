import ujson
import os
from os import path

# data_output = r'D:\Carla Export'
data_output = r"D:\horus"


def import_logs():
    log_dir = os.path.join(data_output, "horus-test", 'log.json')
    log_tokens = []

    with open(log_dir) as file:
        file_data = ujson.load(file)
        for d in file_data:
            if d['location'] == "Town05":
                log_tokens.append(d['token'])

    return log_tokens


def update_maps(log_tokens):
    map_dir = os.path.join(data_output, "horus-test", 'map.json')

    with open(map_dir, 'r') as file:
        data = ujson.load(file)
        data[1]['log_tokens'].extend(log_tokens)

    with open(map_dir, 'w') as file:
        ujson.dump(data, file, indent=4)


update_maps(import_logs())
