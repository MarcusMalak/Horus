import os
import numpy as np

from nuscenes.nuscenes import NuScenes

current_dir = os.getcwd()

dataroot = os.path.abspath('/media/marcus/Backup disk/horus')
nusc = NuScenes(version='horus-trainval', dataroot=dataroot, verbose=True)

th = 30

def filter_range(ego_loc_arr, object):
    location = object['translation']
    ann_loc_arr = np.array((location[0], location[1]))
    dist = np.linalg.norm(ego_loc_arr - ann_loc_arr)
    if dist < th:
        return True
    else:
        return False


def find_samples(nusc_annos):
    for sample_token in nusc_annos.keys():
        my_sample = nusc.get('sample', sample_token)
        ego_pose = nusc.get('sample_data', my_sample['data']['LIDAR_TOP'])['ego_pose_token']
        ego_loc = nusc.get('ego_pose', ego_pose)['translation']
        ego_loc_arr = np.array((ego_loc[0], ego_loc[1]))

        nusc_annos[sample_token] = [object for object in nusc_annos[sample_token] if filter_range(ego_loc_arr, object)]

    return nusc_annos