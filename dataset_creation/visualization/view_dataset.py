import os
from nuscenes.nuscenes import NuScenes

current_dir = os.getcwd()

# dataroot = os.path.join(current_dir, r'D:\horus')
# nusc = NuScenes(version='horus-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\nuscenes")
# nusc = NuScenes(version='v1.0-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.join(current_dir, r'C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_3')
# nusc = NuScenes(version='mini_set_3-trainval', dataroot=dataroot, verbose=True)

dataroot = os.path.join(current_dir, 'data_output', 'mini_set_2_backup')
nusc = NuScenes(version='mini_set_2-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_10")
# nusc = NuScenes(version='mini_set_10-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\dataset_creation\generate_dataset\data_output\mini_set_2")
# nusc = NuScenes(version='mini_set_2-trainval', dataroot=dataroot, verbose=True)

# dataroot = os.path.abspath(r"C:\Users\malak\Documents\MasterThesis\MasterThesis\mmdetection3d\data\mini_set_3_working")
# nusc = NuScenes(version='mini_set_3-trainval', dataroot=dataroot, verbose=True)

nusc.list_categories()

# List scenes
# nusc.list_scenes()
my_scene = nusc.scene[0]

# my_sample = nusc.sample[0]

# anns = my_sample['anns']
# for ann in anns:
#     ann_record = nusc.get('sample_annotation', ann)
#     if int(ann_record['visibility_token']) > 0:
#         print(ann)

# print(my_sample)

# # RENDER SEQUENCE
# my_scene_token = nusc.field2token('scene', 'name', 'scene1')[0]

# Render 1 cam
# nusc.render_scene_channel(my_scene_token, channel="CAM_DRONE_25", imsize=(1920, 1080), out_path='drone_25.avi')
# nusc.render_scene_channel(my_scene_token, channel="CAM_DRONE_50", imsize=(1920, 1080), out_path='drone_50.avi')
# nusc.render_scene_channel(my_scene_token, channel="CAM_DRONE_100", imsize=(1920, 1080), out_path='drone_100.avi')
# nusc.render_scene_channel(my_scene_token, channel="CAM_DRONE_200", imsize=(1920, 1080), out_path='drone_200.avi')

# nusc.render_scene(my_scene_token, imsize=(1920, 1080), out_path='drone_all.avi')

# nusc.render_scene_channel_lidarseg(my_scene_token, channel="CAM_DRONE_100", render_mode="video", with_anns=True,
#                                    freq=2, imsize=(1920, 1080), out_folder='vis_output')



# # RENDER CAM
# # sensor = 'CAM_FRONT'
# # sensor = 'CAM_RIGHT'
# # sensor = 'CAM_LEFT'
# # sensor = 'CAM_DRONE_400_4K'
# sensor = 'CAM_DRONE_100'
# # sensor = 'LIDAR_TOP'
#
# my_sample = nusc.sample[25]
# cam_drone_data = nusc.get('sample_data', my_sample['data'][sensor])
# # nusc.render_sample_data(cam_drone_data['token'], underlay_map=False, show_lidarseg=False)
# nusc.render_sample_data(cam_drone_data['token'], underlay_map=False, show_lidarseg=False)
#

# for i in range(100):
#     my_sample = nusc.sample[i]
#     cam_drone_data = nusc.get('sample_data', my_sample['data'][sensor])
#     nusc.render_sample_data(cam_drone_data['token'])


# # RENDER VISIBILITY
# anntoken = "jdd430itg98pe6qkzcbtsll7nnhw9qhq"
# visibility_token = nusc.get('sample_annotation', anntoken)['visibility_token']
# nusc.render_annotation(anntoken)


# my_sample = nusc.sample[7]
# print(my_sample)
# cam_drone_data = nusc.get('sample_data', my_sample['data'][sensor])
# nusc.render_sample_data(cam_drone_data['token'])

# print(my_sample)
#
# my_annotation_token = my_sample['anns'][0]
# my_annotation_metadata = nusc.get('sample_annotation', my_annotation_token)
# print(my_annotation_metadata)

# # # RENDER LIDAR
# my_sample = nusc.sample[328]
# nusc.render_pointcloud_in_image(my_sample['token'], pointsensor_channel='LIDAR_TOP')

# for i in range(100):
#     my_sample = nusc.sample[i]
#     nusc.render_pointcloud_in_image(my_sample['token'], pointsensor_channel='LIDAR_TOP')


# # RENDER LIDARSEG
# my_sample = nusc.sample[0]
# sample_data_token = my_sample['data']['LIDAR_TOP']
# nusc.render_sample_data(sample_data_token,
#                         with_anns=True,
#                         show_lidarseg=True)

# RENDER LIDAR in image
my_sample = nusc.sample[0]
nusc.render_pointcloud_in_image(my_sample['token'],
                                pointsensor_channel='LIDAR_TOP',
                                camera_channel='CAM_DRONE_100',
                                render_intensity=False,
                                show_lidarseg=True,
                                filter_lidarseg_labels=[0, 28, 29, 2, 24, 26, 30, 17, 25, 11, 27],
                                show_lidarseg_legend=True)


