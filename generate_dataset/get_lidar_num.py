import os
import ujson

from nuscenes.nuscenes import NuScenes
from nuscenes.utils.data_classes import LidarPointCloud
from nuscenes.utils.geometry_utils import points_in_box

current_dir = os.getcwd()

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_3")
# nusc = NuScenes(version='mini_set_3-trainval', dataroot=dataroot, verbose=True)

dataroot = os.path.abspath(r"D:\horus")
nusc = NuScenes(version='horus-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.join(current_dir, 'data_output', 'mini_set_1')
# nusc = NuScenes(version='mini_set_1-trainval', dataroot=dataroot, verbose=True)

counter = 0
count_ped = 0
points_out = {}

for i in range(len(nusc.sample)-8000):
    print(f'sample: {i+8000}')
    my_sample = nusc.sample[i+8000]

    sample_annots = my_sample['anns']
    if my_sample['data']['LIDAR_TOP']:
        sample_data = my_sample['data']['LIDAR_TOP']
        _, boxes, _ = nusc.get_sample_data(sample_data)

        for i in range(len(sample_annots)):
            my_ann = sample_annots[i]

            _, boxes, _ = nusc.get_sample_data(sample_data, selected_anntokens=[my_ann])

            lidar_token = my_sample['data']['LIDAR_TOP']
            lidar_sample_data = nusc.get('sample_data', lidar_token)
            pcl_path = os.path.join(dataroot, lidar_sample_data['filename'])
            pc = LidarPointCloud.from_file(pcl_path).points.T[:, :3].T

            if boxes:
                p_in_box = points_in_box(boxes[0], pc)
                num_points = p_in_box.sum()
                if num_points:
                    points_out[my_ann] = int(num_points)
                    counter += 1
                else:
                    points_out[my_ann] = 0
            else:
                points_out[my_ann] = 0
    else:
        continue

print(f'\n EXPORTING JSON')
# Update .json
output_file = os.path.join(dataroot, "horus-trainval", "sample_annotation.json")

with open(output_file, 'r') as file:
    data = ujson.load(file)
    for i, d in enumerate(data):
        print(f'ann: {i}')
        if d['token'] in points_out:
            d['num_lidar_pts'] = points_out[d['token']]

    # for token, num_points in points_out.items():
    #     for d in data:
    #         if d['token'] == token:
    #             d['num_lidar_pts'] = num_points

with open(output_file, 'w') as file:
    ujson.dump(data, file, indent=4)
