import os
import ujson


def check_data(output_dir, name):
    json_folder = os.path.join(output_dir, name + "-trainval")

    # If folder exists
    if os.path.exists(json_folder):
        return import_data(json_folder)
    else:
        return False


def import_data(json_folder):
    sensor_tokens = {}
    calibrated_sensor_tokens = {}
    attribute_tokens = {}
    category_tokens = {}

    # Sensor tokens
    sensor_file = os.path.join(json_folder, "sensor.json")
    with open(sensor_file, 'r') as file:
        file_data = ujson.load(file)
        for d in file_data:
            sensor_tokens[d["channel"]] = d["token"]

    # Calibrated sensor tokens
    calibrated_sensor_file = os.path.join(json_folder, "calibrated_sensor.json")
    with open(calibrated_sensor_file, 'r') as file:
        file_data = ujson.load(file)
        inv_sensor = {v: k for k, v in sensor_tokens.items()}
        for d in file_data:
            calibrated_sensor_tokens[inv_sensor[d["sensor_token"]]] = d["token"]

    # Attribute tokens
    attribute_file = os.path.join(json_folder, "attribute.json")
    with open(attribute_file, 'r') as file:
        file_data = ujson.load(file)
        for d in file_data:
            attribute_tokens[d["name"]] = d["token"]

    # Category tokens
    category_file = os.path.join(json_folder, "category.json")
    with open(category_file, 'r') as file:
        file_data = ujson.load(file)
        for d in file_data:
            category_tokens[d["name"]] = d["token"]

    return sensor_tokens, calibrated_sensor_tokens, attribute_tokens, category_tokens