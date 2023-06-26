import os

from matplotlib import image as mpimg
from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion
from nuscenes.utils.data_classes import Box
from nuscenes.utils.geometry_utils import box_in_image, BoxVisibility
import numpy as np
import shutil
import random
import yaml

from PIL import Image, ImageDraw
import matplotlib.pyplot as plt
os.chdir('..')
current_dir = os.getcwd()

# dataroot = os.path.join(current_dir, 'datasets', 'mini_set_3_working')
# dataroot = r'C:\Users\malak\Documents\MasterThesis\MasterThesis\datasets\mini_set_3_working'

dataroot = r'/tudelft.net/staff-umbrella/horus/mmdetection3d/data/horus'
nusc = NuScenes(version='horus-trainval', dataroot=dataroot, verbose=True)

# classes = ['human.pedestrian.adult', 'vehicle.car', 'vehicle.truck', 'vehicle.bus.rigid', 'vehicle.motorcycle',
#            'vehicle.bicycle', 'vehicle.parked']

classes = {'human.pedestrian.adult': "pedestrian",
           'vehicle.car': "car",
           'vehicle.truck': "truck",
           'vehicle.bus.rigid': "bus",
           'vehicle.motorcycle': "motorcycle",
           'vehicle.bicycle': "bicycle",
           'vehicle.parked': "car"}

# camera_names = ['CAM_DRONE_400_4K']
# camera_names = ['CAM_DRONE_25', 'CAM_DRONE_50', 'CAM_DRONE_100', 'CAM_DRONE_200']
camera_names = ['CAM_DRONE_200']

labels = {'pedestrian': 0,
          'car': 1,
          'truck': 2,
          'bus': 3,
          'motorcycle': 4,
          'bicycle': 5}

dataset_name = 'Horus'

total_no_of_samples = len(nusc.sample)
# total_no_of_samples = 3

i = 0
detected_items = []
orig_detected_items = []
obs = []
file = []

print('Total number of samples')
print(len(nusc.sample))


def get_sample_data(nusc_object, sample_data_token, box_vis_level=BoxVisibility.ANY, selected_anntokens=None):
    """
    Returns the data path as well as all annotations related to that sample_data(single image).
    Note that the boxes are transformed into the current sensor's coordinate frame.
    :param sample_data_token: <str>. Sample_data token(image token).
    :param box_vis_level: <BoxVisibility>. If sample_data is an image, this sets required visibility for boxes.
    :param selected_anntokens: [<str>]. If provided only return the selected annotation.
    :return: (data_path <str>, boxes [<Box>], camera_intrinsic <np.array: 3, 3>)
    """

    # Retrieve sensor & pose records
    sd_record = nusc_object.get('sample_data', sample_data_token)
    cs_record = nusc_object.get('calibrated_sensor', sd_record['calibrated_sensor_token'])
    sensor_record = nusc_object.get('sensor', cs_record['sensor_token'])
    pose_record = nusc_object.get('ego_pose', sd_record['ego_pose_token'])

    sample_record = nusc_object.get('sample',sd_record['sample_token'])
    data_path = nusc_object.get_sample_data_path(sample_data_token)

    if sensor_record['modality'] == 'camera':
        cam_intrinsic = np.array(cs_record['camera_intrinsic'])
        imsize = (sd_record['width'], sd_record['height'])
    else:
        cam_intrinsic = None
        imsize = None

    # Retrieve all sample annotations and map to sensor coordinate system.
    if selected_anntokens is not None:
        boxes = list(map(nusc_object.get_box, selected_anntokens))
    else:
        boxes = nusc_object.get_boxes(sample_data_token)
        selected_anntokens = sample_record['anns']

    # Make list of Box objects including coord system transforms.
    box_list = []
    ann_list = []
    for box, ann in zip(boxes, selected_anntokens):
        # Move box to ego vehicle coord system
        box.translate(-np.array(pose_record['translation']))
        box.rotate(Quaternion(pose_record['rotation']).inverse)

        #  Move box to sensor coord system
        box.translate(-np.array(cs_record['translation']))
        box.rotate(Quaternion(cs_record['rotation']).inverse)

        if sensor_record['modality'] == 'camera' and not \
                box_in_image(box, cam_intrinsic, imsize, vis_level=box_vis_level):
            continue

        box_list.append(box)
        ann_list.append(ann)

    return data_path, box_list, ann_list, cam_intrinsic     # single image info


def threeD_2_twoD(boxsy, intrinsic):  # input is a single annotation box
    '''
    given annotation boxes and intrinsic camera matrix
    outputs the 2d bounding box coordinates as a list (all annotations for a particular sample image)
    '''
    corners = boxsy.corners()
    x = corners[0, :]
    y = corners[1, :]
    z = corners[2, :]
    x_y_z = np.array((x, y, z))
    orthographic = np.dot(intrinsic, x_y_z)
    perspective_x = orthographic[0] / orthographic[2]
    perspective_y = orthographic[1] / orthographic[2]
    perspective_z = orthographic[2] / orthographic[2]

    min_x = np.min(perspective_x)
    max_x = np.max(perspective_x)
    min_y = np.min(perspective_y)
    max_y = np.max(perspective_y)

    return min_x, max_x, min_y, max_y


def all_3d_to_2d(boxes, anns, intrinsic):  # input 3d boxes, annotation key lists, intrinsic matrix (one image)
    x_min = []
    x_max = []
    y_min = []
    y_max = []
    width = []
    height = []
    objects_detected = []
    orig_objects_detected = []

    for j in range(len(boxes)):  # iterate through boxes
        box = boxes[j]

        if box.name in classes:  # if the box.name is in the classes we want to detect
            orig_objects_detected.append(classes[box.name])

            # visibility = nusc.get('sample_annotation', '%s' % anns[j])['visibility_token']  # give annotation key
            # visibility = int(visibility)

            # if visibility > 1:  # more than 40% visible in the panoramic view of the the cameras

            center = box.center  # get boxe's center

            center = np.dot(intrinsic, center)
            center_point = center / (center[2])  # convert center point into image plane

            # print(f'center_point: {center_point}')

            if center_point[0] < 0 or center_point[0] > 1920 or center_point[1] < 0 or center_point[1] > 1080:
                # if center of bounding box is outside of the image, do not annotate
                pass

            else:
                min_x, max_x, min_y, max_y = threeD_2_twoD(box, intrinsic)  # converts box into image plane
                w = max_x - min_x
                h = max_y - min_y

                x_min.append(min_x)
                x_max.append(max_x)
                y_min.append(min_y)
                y_max.append(max_y)
                width.append(w)
                height.append(h)

                objects_detected.append(classes[box.name])

                # x_loc = int(round((min_x + max_x) / 2))
                # y_loc = int(round((min_y + max_y) / 2))
                # w_object = int(round(w))
                # h_object = int(round(h))

                # print(f'bbox: {(x_loc, y_loc), (w_object, h_object)}')

                # w = box.wlh[0]
                # l = box.wlh[1]
                #
                # min_x = center_point[0] - 0.5*w
                # max_x = center_point[0] + 0.5*w
                # min_y = center_point[1] - 0.5*l
                # max_y = center_point[1] + 0.5*l
                #
                # x_min.append(min_x)
                # x_max.append(max_x)
                # y_min.append(min_y)
                # y_max.append(max_y)
                # width.append(w)
                # height.append(l)
                #
                # print(f'bbox: {(center_point[0], center_point[1]), (w, l)}')

                # if box.name in pedestrians:
                #     objects_detected.append("pedestrian")
                # elif box.name == "vehicle.car":
                #     objects_detected.append("car")
                # else:
                #     objects_detected.append("cyclist")
        else:
            pass

    return x_min, x_max, y_min, y_max, width, height, objects_detected, orig_objects_detected  # for a single image


def extract_bounding_box(i, camera_name):  # give a single sample number and camera name
    """
    input sample number i, camera name
    outputs min x, max x, min y max y, width and height of bounding box in image coordinates
    2d bounding box
    options for camera name : CAM_FRONT, CAM_FRONT_RIGHT, CAM_FRONT_LEFT, CAM_BACK, CAM_BACK_RIGHT,CAM_BACK_LEFT
    """
    # nusc.sample[i]  # one image

    camera_token = nusc.sample[i]['data']['%s' % camera_name]  # one camera, get the camera token
    path, boxes, anns, intrinsic_matrix = get_sample_data(nusc, '%s' % camera_token)  # gets data for one image
    x_min, x_max, y_min, y_max, width, height, objects_detected, orig_objects_detected = all_3d_to_2d(boxes, anns,
                                                                                                      intrinsic_matrix)

    return x_min, x_max, y_min, y_max, width, height, path, boxes, intrinsic_matrix, objects_detected, orig_objects_detected


def write_to_text(x_min, x_max, y_min, y_max, width, height, path, boxes, objects_detected, dirName_im, dirName_lab, camera):
    im_w = 1920
    im_h = 1080

    dots = []

    for i, object in enumerate(objects_detected):
        x_loc = int(round((x_min[i] + x_max[i])/2))
        y_loc = int(round((y_min[i] + y_max[i])/2))
        w_object = int(round(width[i]))*1.2
        h_object = int(round(height[i]))*1.2

        # dots.append([x_loc, y_loc])

        # Normalize between 0 and 1
        x_loc /= im_w
        y_loc /= im_h
        w_object /= im_w
        h_object /= im_h

        label = labels[object]

        if (0 < x_loc < 1920) and (0 < y_loc < 1080):
            # Copy image
            image_name = os.path.basename(path)
            image_base = image_name.split(".")[0]
            image_out = image_base + "_" + camera + ".jpg"
            image_copy_path = os.path.join(dirName_im, image_out)
            shutil.copyfile(path, image_copy_path)

            # Save .txt annotation
            txt_name = os.path.basename(path).split(".")[0] + "_" + camera
            filename = "%s.txt" % txt_name
            output_file = os.path.join(dirName_lab, filename)
            with open(output_file, 'a') as f:
                f.write('{} {} {} {} {}\n'.format(label, x_loc, y_loc, w_object, h_object))

    # # Draw dots
    # dots = np.array(dots)
    # image = mpimg.imread(path)
    # plt.imshow(image)
    # plt.scatter(dots[:, 0], dots[:, 1], marker="x", color="red", s=200)
    # plt.show()
    #

def create_annotation_directories(camera):
    dirName_train_im = os.path.join(dataroot, 'YOLO', camera, 'train', 'images')
    dirName_train_lab = os.path.join(dataroot, 'YOLO', camera, 'train', 'labels')
    dirName_val_im = os.path.join(dataroot, 'YOLO', camera, 'val', 'images')
    dirName_val_lab = os.path.join(dataroot, 'YOLO', camera, 'val', 'labels')
    dirName_test_im = os.path.join(dataroot, 'YOLO', camera, 'test', 'images')
    dirName_test_lab = os.path.join(dataroot, 'YOLO', camera, 'test', 'labels')

    dirName_data = os.path.join(dataroot, 'YOLO', camera, 'data')

    if not os.path.exists(dirName_train_im):
        os.makedirs(dirName_train_im)
    if not os.path.exists(dirName_train_lab):
        os.makedirs(dirName_train_lab)

    if not os.path.exists(dirName_val_im):
        os.makedirs(dirName_val_im)
    if not os.path.exists(dirName_val_lab):
        os.makedirs(dirName_val_lab)

    if not os.path.exists(dirName_test_im):
        os.makedirs(dirName_test_im)
    if not os.path.exists(dirName_test_lab):
        os.makedirs(dirName_test_lab)

    if not os.path.exists(dirName_data):
        os.makedirs(dirName_data)

    return dirName_train_im, dirName_train_lab, dirName_val_im, dirName_val_lab, dirName_test_im, dirName_test_lab, dirName_data


# Make train, val and test split
# all_samples = np.arange(0, total_no_of_samples-1)
# random.shuffle(all_samples)
# train_set = all_samples[:int(0.8*total_no_of_samples)]
# val_set = all_samples[int(0.8*total_no_of_samples):int(0.9*total_no_of_samples)]
# test_set = all_samples[int(0.9*total_no_of_samples):]

train_set = np.array(list(range(0, 41 * 270)) + list(range(41 * 301, 41 * 454)))
val_set = np.array(list(range(41*271, 41 * 300)) + list(range(41 * 454, 41 * 470)))
# val_set = np.arange(41*270, 41*285)
# test_set = np.arange(41*285, 41*300)

# train_set = np.arange(0, 41*8)
# val_set = np.arange(41*8, 41*9)
# test_set = np.arange(41*9, 41*10)

for camera in camera_names:
    dirName_train_im, dirName_train_lab, dirName_val_im, dirName_val_lab, dirName_test_im, dirName_test_lab, dirName_data = create_annotation_directories(camera)

    # Create train set
    for i, sample_number in enumerate(train_set):
        print(f'{camera} train: {i}')
        x_min, x_max, y_min, y_max, width, height, path, boxes, intrinsic_matrix, objects_detected, orig_objects_detected = extract_bounding_box(
            sample_number, '%s' % camera)
        write_to_text(x_min, x_max, y_min, y_max, width, height, path, boxes, objects_detected, dirName_train_im, dirName_train_lab, camera)

    # Create val set
    for i, sample_number in enumerate(val_set):
        print(f'{camera} val: {i}')
        x_min, x_max, y_min, y_max, width, height, path, boxes, intrinsic_matrix, objects_detected, orig_objects_detected = extract_bounding_box(
            sample_number, '%s' % camera)
        write_to_text(x_min, x_max, y_min, y_max, width, height, path, boxes, objects_detected, dirName_val_im, dirName_val_lab, camera)

    # # Create test set
    # for i, sample_number in enumerate(test_set):
    #     print(f'{camera} test: {i}')
    #     x_min, x_max, y_min, y_max, width, height, path, boxes, intrinsic_matrix, objects_detected, orig_objects_detected = extract_bounding_box(
    #         sample_number, '%s' % camera)
    #     write_to_text(x_min, x_max, y_min, y_max, width, height, path, boxes, objects_detected, dirName_test_im, dirName_test_lab, camera)

    # Write data .yaml file
    data_dict = {'train': os.path.join(dataroot, 'YOLO', camera, 'train'),
                 'val': os.path.join(dataroot, 'YOLO', camera, 'val'),
                 # 'test': os.path.join(dataroot, 'YOLO', camera, 'test'),
                 'nc': 6,
                 'names': ["pedestrian", "car", "truck", "bus", "motorcycle", "bicycle"]}

    data_file = os.path.join(dirName_data, dataset_name + '_' + camera + '.yaml')

    with open(data_file, 'w') as data_out:
        yaml.dump(data_dict, data_out, sort_keys=False)
