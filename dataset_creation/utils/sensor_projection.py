import numpy as np


def sensor_2_ego(sensor_trans, vehicle):
    world_2_vehicle = np.array(vehicle.get_transform().get_inverse_matrix())
    sensor_trans = np.array([sensor_trans.x, sensor_trans.y, sensor_trans.z, 1])
    sensor_vehicle = np.dot(world_2_vehicle, sensor_trans)
    # sensor_vehicle = [sensor_vehicle[1], -sensor_vehicle[2], sensor_vehicle[0]]
    sensor_vehicle = [sensor_vehicle[0], sensor_vehicle[1], sensor_vehicle[2]]

    return sensor_vehicle
