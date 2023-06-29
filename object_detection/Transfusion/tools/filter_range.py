import os
import numpy as np

from nuscenes.nuscenes import NuScenes
import pickle

current_dir = os.getcwd()

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_10")
# nusc = NuScenes(version='mini_set_10-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_3_working")
# nusc = NuScenes(version='mini_set_3-trainval', dataroot=dataroot, verbose=True)

dataroot = os.path.abspath(r'/media/marcus/Backup disk/horus')
nusc = NuScenes(version='horus-trainval', dataroot=dataroot, verbose=True)

th_min = 40
th_max = 50

th_values = [(0, 10), (10, 20), (20, 30), (30, 40), (40, 50), (50, 60), (60, 70)]
# th_values = [(40, 50)]

# pkl_file = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_3_working\mini_set_3_infos_val.pkl")
pkl_file = r"/media/marcus/Backup disk/horus/horus_infos_val_org.pkl"


def filter_vis(sample_token, i):
    my_sample = nusc.get('sample', sample_token)
    ego_pose = nusc.get('sample_data', my_sample['data']['LIDAR_TOP'])['ego_pose_token']
    ego_loc = nusc.get('ego_pose', ego_pose)['translation']
    ann = my_sample['anns'][i]
    ann_record = nusc.get('sample_annotation', ann)
    ann_loc = ann_record['translation']
    ego_loc_arr = np.array((ego_loc[0], ego_loc[1]))
    ann_loc_arr = np.array((ann_loc[0], ann_loc[1]))

    dist = np.linalg.norm(ego_loc_arr - ann_loc_arr)
    if dist < 75:
        if th_min < dist < th_max:
            return True
        else:
            return False
    else:
        return False


for value in th_values:
    th_min = value[0]
    th_max = value[1]

    print(f'MAKING RANGE {th_min, th_max}')

    with open(pkl_file, 'rb') as file:
        data = pickle.load(file)

        for sample in data['infos']:
            sample['gt_boxes'] = [instance for i, instance in enumerate(sample['gt_boxes']) if filter_vis(sample['token'], i)]
            sample['gt_names'] = [instance for i, instance in enumerate(sample['gt_names']) if filter_vis(sample['token'], i)]
            sample['gt_velocity'] = [instance for i, instance in enumerate(sample['gt_velocity']) if filter_vis(sample['token'], i)]
            sample['num_lidar_pts'] = [instance for i, instance in enumerate(sample['num_lidar_pts']) if filter_vis(sample['token'], i)]
            sample['num_radar_pts'] = [instance for i, instance in enumerate(sample['num_radar_pts']) if filter_vis(sample['token'], i)]
            sample['valid_flag'] = [instance for i, instance in enumerate(sample['valid_flag']) if filter_vis(sample['token'], i)]

    output = 'horus_infos_val_range_' + str(th_min) + '-' + str(th_max) + '.pkl'

    with open(output, 'wb') as file:
        pickle.dump(data, file)

    print(f'DONE')
