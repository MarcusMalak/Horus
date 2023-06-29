import numpy as np
import math


def filter_occluded(drone_cameras, loc, size, depth, img_w, img_h, img_w_4k, img_h_4k, K_drone, K_drone_4k):
    # Get corners of bbox in image
    corners = get_corners(loc, size)  # top_left, top_right, bottom_right, bottom_left

    occ_drone = True
    vis_drone = 0

    for channel, camera in reversed(drone_cameras.items()):
        if channel == "CAM_DRONE_400_4K":
            width_img = img_w_4k
            height_img = img_h_4k
            K = K_drone_4k
        else:
            width_img = img_w
            height_img = img_h
            K = K_drone

        depth_cam = depth[channel]
        cam = camera[0]
        height = camera[1]
        corners_projected = []  # top_left, top_right, bottom_right, bottom_left

        # Get the world to camera matrix
        w2c = np.array(cam.get_transform().get_inverse_matrix())

        for point in corners:
            # Format the input coordinate (loc is a carla.Position object)
            point = np.array([point[0], point[1], point[2], 1])

            # transform to camera coordinates
            point_camera = np.dot(w2c, point)
            point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

            # now project 3D->2D using the camera matrix
            point_img = np.dot(K, point_camera)
            # normalize
            point_img[0] /= point_img[2]
            point_img[1] /= point_img[2]

            object_pos = point_img[0:2]
            corners_projected.append(object_pos)    # top_left --> bottom_left --> bottom_right --> top_right

        # Convert depth to rgb values
        d_raw = process_image_depth(depth_cam)

        # Get bbox in depth image
        bbox_depth, in_image = get_bbox_depth(None, corners_projected, d_raw, width_img, height_img)

        if bbox_depth is not None:
            occluded, vis_drone = get_visibility_bbox(bbox_depth, height, True)
            if not occluded:
                occ_drone = False
            elif in_image:
                occ_drone = True
        if vis_drone is None:
            vis_drone = 0

    return occ_drone, vis_drone


def get_visiblity_ego(car_cameras, loc, size, depth, img_w, img_h, K_car, vehicle):
    occ_car = True
    vis_car = 0

    # Get forward vector
    forward_vec = vehicle.get_transform().get_forward_vector()
    forward_vec = np.array((forward_vec.x, forward_vec.y, forward_vec.z))
    left_vec = np.array((forward_vec[1], -forward_vec[0], forward_vec[2]))
    back_vec = np.array((left_vec[1], -left_vec[0], left_vec[2]))
    right_vec = np.array((back_vec[1], -back_vec[0], back_vec[2]))

    vectors = {
        "CAM_FRONT": forward_vec,
        "CAM_RIGHT": right_vec,
        "CAM_BACK": back_vec,
        "CAM_LEFT": left_vec
    }
    for channel, camera in car_cameras.items():
        corners_ego = get_corners_ego(loc, size, channel)
        depth_cam = depth[channel]
        cam = camera[0]
        cam_trans = camera[1]

        corners_projected = np.zeros((2, 8))

        # Get the world to camera matrix
        w2c = np.array(cam.get_transform().get_inverse_matrix())

        # Get ray from one corner to check if object in front of cam
        ray = loc - vehicle.get_transform().location
        ray = np.array((ray.x, ray.y, ray.z))

        vector = vectors[channel]

        if vector.dot(ray) > 1:
            # Get distance to cam
            # d2cam_real = np.sqrt((ego_transform.x + cam_trans[0] - loc.x) ** 2 + (ego_transform.y + cam_trans[1] - loc.y) ** 2)
            d2cam_depth = None
            invalid = False
            for i in range(corners_ego.shape[1]):
                # Format the input coordinate (loc is a carla.Position object)
                point = np.array([corners_ego[0][i], corners_ego[1][i], corners_ego[2][i], 1])

                # transform to camera coordinates
                point_camera = np.dot(w2c, point)
                point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

                distance = abs(point_camera[2])
                if d2cam_depth is None:
                    d2cam_depth = distance
                elif d2cam_depth > distance:
                    d2cam_depth = distance

                if abs(point_camera[1]) > 5:
                    invalid = True
                    break

                # now project 3D->2D using the camera matrix
                point_img = np.dot(K_car, point_camera)
                # normalize
                point_img[0] /= point_img[2]
                point_img[1] /= point_img[2]

                object_pos = point_img[0:2]
                corners_projected[:, i] = object_pos

            corner_edges = find_edges(corners_projected)

            if (len(corners_projected) != 0) and (invalid is False):
                # Convert depth to rgb values
                d_raw = process_image_depth(depth_cam)

                # Get bbox in depth image
                bbox_depth, in_image = get_bbox_depth(corner_edges, None, d_raw, img_w, img_h)

                if bbox_depth is not None:
                    occluded, vis_car = get_visibility_bbox(bbox_depth, d2cam_depth, False)
                    if not occluded:
                        occ_car = False
                        vis_car = vis_car + 0.3
                        if vis_car > 1:
                            vis_car = 1
                    else:
                        vis_car = 0

    return occ_car, vis_car


def get_corners(loc, size):
    top_left = [loc.x + 0.5 * size[1], loc.y + 0.5 * size[0], loc.z]
    top_right = [loc.x - 0.5 * size[1], loc.y + 0.5 * size[0], loc.z]
    bottom_left = [loc.x + 0.5 * size[1], loc.y - 0.5 * size[0], loc.z]
    bottom_right = [loc.x - 0.5*size[1], loc.y - 0.5*size[0], loc.z]

    corners = [top_left, top_right, bottom_right, bottom_left]
    return corners


def get_corners_ego(loc, size, channel):
    corners = np.zeros((3, 8))

    if channel == "CAM_FRONT" or channel == "CAM_BACK":
        w = size[0]
        l = size[1]
    else:
        w = size[1]
        l = size[0]

    corners[:, 0] = [loc.x - 0.5 * w, loc.y + 0.5 * l, loc.z - 0.5 * size[2]]   # front_bot_left
    corners[:, 1] = [loc.x + 0.5 * w, loc.y + 0.5 * l, loc.z - 0.5 * size[2]]   # front_bot_right
    corners[:, 2] = [loc.x + 0.5 * w, loc.y - 0.5 * l, loc.z - 0.5 * size[2]]   # back_bot_right
    corners[:, 3] = [loc.x - 0.5 * w, loc.y - 0.5 * l, loc.z - 0.5 * size[2]]   # back_bot_left

    corners[:, 4] = [loc.x - 0.5 * w, loc.y + 0.5 * l, loc.z + 0.5 * size[2]]   # front_top_left
    corners[:, 5] = [loc.x + 0.5 * w, loc.y + 0.5 * l, loc.z + 0.5 * size[2]]   # front_top_right
    corners[:, 6] = [loc.x + 0.5 * w, loc.y - 0.5 * l, loc.z + 0.5 * size[2]]   # back_top_right
    corners[:, 7] = [loc.x - 0.5 * w, loc.y - 0.5 * l, loc.z + 0.5 * size[2]]   # back_top_left

    return corners


def find_edges(corners):
    x = corners[0, :]
    y = corners[1, :]
    min_x = np.min(x)
    max_x = np.max(x)
    min_y = np.min(y)
    max_y = np.max(y)

    return [min_x, max_x, min_y, max_y]


def process_image_depth(image):
    """Convert image to numpy array"""
    # Get raw image in 8bit format
    raw_image = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    # Reshape image to RGBA
    raw_image = np.reshape(raw_image, (image.height, image.width, 4))
    # Taking only RGB
    d_raw = raw_image[:, :, :3]

    return d_raw


def get_bbox_depth(min_max, corners, depth_img, img_w, img_h):
    # input corners = top_left --> bottom_left --> bottom_right --> top_right
    # Image origin top left corner
    in_image = False

    if min_max is None:
        x = []
        y = []

        x.append(int(corners[0][0]))  # top_left
        x.append(int(corners[3][0]))  # top_right
        y.append(int(corners[1][1]))  # top_left
        y.append(int(corners[0][1]))  # bottom_left

        x_min = min(x)
        x_max = max(x)
        y_min = min(y)
        y_max = max(y)

    else:
        x_min = int(min_max[0])
        x_max = int(min_max[1])
        y_min = int(min_max[2])
        y_max = int(min_max[3])

    if ((img_w > x_min > 0) or (0 < x_max < img_w)) and ((img_h > y_min > 0) or (0 < y_max < img_h)):
        if x_min < 0:
            x_min = 0
        if y_min < 0:
            y_min = 0
        if x_max > img_w:
            x_max = img_w - 1
        if y_max > img_h:
            y_max = img_h - 1

        in_image = True
        bbox_raw = depth_img[y_min:y_max+1, x_min:x_max+1]
        bbox_depth = np.zeros((bbox_raw.shape[0], bbox_raw.shape[1]))
        for y in range(bbox_raw.shape[0]):
            for x in range(bbox_raw.shape[1]):
                bbox_depth[y][x] = ((bbox_raw[y][x][2] + bbox_raw[y][x][1] * 256 + bbox_raw[y][x][0] * 256 * 256) / (
                            256 * 256 * 256 - 1)) * 1000

    else:
        bbox_depth = None

    return bbox_depth, in_image


def get_visibility_bbox(bbox_depth, d2cam, drone):
    occluded = True

    if drone:
        th = d2cam - 3

        non_occluded = np.sum(bbox_depth > th)
        total_values = bbox_depth.size
        visibility = non_occluded/total_values

        if visibility > 0.1:
            occluded = False

    else:
        th_min = abs(d2cam - 0.5)
        th_max = abs(d2cam + 2)

        th_min_med = abs(d2cam - 3)
        th_max_med = abs(d2cam + 3)

        median = np.median(bbox_depth)
        if median < th_min_med or median > th_max_med:
            visibility = 0
        else:
            non_occ = np.sum(np.logical_and(bbox_depth > th_min, bbox_depth < th_max))

            total_values = bbox_depth.size
            visibility = (non_occ / total_values)

            if visibility > 0.1:
                occluded = False

    return occluded, visibility


def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K



# def get_depth_d2c(ego_transform, ego_rotation, object_loc, d2cam_real, cam_trans, channel):
#     fi_angle = 90-abs(ego_rotation.yaw)     # Angle of rotation relative to horizontal axis
#     # print(f'\nego_rot: {ego_rotation.yaw}')
#     # print(f'fi_angle: {fi_angle}')
#     dx = abs(ego_transform.x + cam_trans[0] - object_loc.x)     # dx between ego and object
#     dy = abs(ego_transform.y + cam_trans[1] - object_loc.y)
#     # print(f'ego_x_y: {ego_transform.x, ego_transform.y}')
#     # print(f'object_x_y: {object_loc.x, object_loc.y}')
#     # print(f'd2cam_real: {d2cam_real}')
#     th_angle = math.acos(dx/d2cam_real)     # th +- fi = angle between object and camera axis
#     # print(f'th_angle: {math.degrees(th_angle)}')
#
#     print(f'ego_rto_before: {ego_rotation.yaw}')
#
#     if channel == "CAM_FRONT":
#         ego_rot = ego_rotation.yaw
#     elif channel == "CAM_RIGHT":
#         ego_rot = ego_rotation.yaw - 90
#     elif channel == "CAM_BACK":
#         ego_rot = ego_rotation.yaw + 180
#     elif channel == "CAM_LEFT":
#         ego_rot = ego_rotation.yaw + 90
#
#     print(f'ego_rotated: {ego_rot}')
#
#     if ego_rot > 180:
#         ego_rot = ego_rot - 360
#     elif ego_rot < -180:
#         ego_rot = ego_rot + 360
#
#     print(f'ego_rot_fixes: {ego_rot}')
#
#     if -90 <= ego_rot <= 0:
#         if object_loc.x < ego_transform.x:
#             th_angle = math.acos(dx/d2cam_real)
#             if object_loc.y > ego_transform.y:
#                 angle_obj = fi_angle - math.degrees(th_angle)
#             else:
#                 angle_obj = fi_angle + math.degrees(th_angle)
#         elif object_loc.x > ego_transform.x:
#             th_angle = math.asin(dx / d2cam_real)
#             angle_obj = 90 - math.degrees(th_angle) - fi_angle
#
#     elif -180 <= ego_rot < -90:
#         if object_loc.x < ego_transform.x:
#             th_angle = math.asin(dx / d2cam_real)
#             angle_obj = 90 - math.degrees(th_angle) - fi_angle
#         elif object_loc.x > ego_transform.x:
#             th_angle = math.acos(dx / d2cam_real)
#             if object_loc.y > ego_transform.y:
#                 angle_obj = fi_angle + math.degrees(th_angle)
#             else:
#                 angle_obj = fi_angle - math.degrees(th_angle)
#
#     elif 90 <= ego_rot <= 180:
#         if object_loc.x > ego_transform.x:
#             th_angle = math.acos(dx / d2cam_real)
#             if object_loc.y < ego_transform.y:
#                 angle_obj = fi_angle - math.degrees(th_angle)
#             else:
#                 angle_obj = fi_angle + math.degrees(th_angle)
#         elif object_loc.x < ego_transform.x:
#             th_angle = math.asin(dx / d2cam_real)
#             angle_obj = 90 - math.degrees(th_angle) - fi_angle
#
#     elif 0 < ego_rot < 90:
#         if object_loc.x < ego_transform.x:
#             th_angle = math.acos(dx / d2cam_real)
#             if object_loc.y < ego_transform.y:
#                 angle_obj = fi_angle - math.degrees(th_angle)
#             else:
#                 angle_obj = fi_angle + math.degrees(th_angle)
#         elif object_loc.x > ego_transform.x:
#             th_angle = math.asin(dx / d2cam_real)
#             angle_obj = 90 - math.degrees(th_angle) - fi_angle
#
#     d2cam_depth = math.sin(math.radians(abs(angle_obj))) * d2cam_real
#
#     return d2cam_depth

# def filter_new(annotated_vehicles, car_cameras, segmented, img_w, img_h, K_car, vehicle, ego_transform):
#     vis_car = 0
#     # Sort vehicles
#     bboxes = {}
#     bboxes_sorted = {}
#     bboxes_out = {}
#     for bb, bb_id in annotated_vehicles.items():
#         d2cam_real = np.sqrt((ego_transform.x - bb.location.x) ** 2 + (ego_transform.y - bb.location.y) ** 2)
#         bboxes[d2cam_real] = (bb, bb_id)
#     for dist in sorted(bboxes):
#         bboxes_sorted[dist] = bboxes[dist]
#
#     # Get forward vector
#     forward_vec = vehicle.get_transform().get_forward_vector()
#     forward_vec = np.array((forward_vec.x, forward_vec.y, forward_vec.z))
#     left_vec = np.array((forward_vec[1], -forward_vec[0], forward_vec[2]))
#     back_vec = np.array((left_vec[1], -left_vec[0], left_vec[2]))
#     right_vec = np.array((back_vec[1], -back_vec[0], back_vec[2]))
#
#     vectors = {
#         "CAM_FRONT": forward_vec,
#         "CAM_RIGHT": right_vec,
#         "CAM_BACK": back_vec,
#         "CAM_LEFT": left_vec
#     }
#
#     for channel, camera in car_cameras.items():
#         instance_ids = []
#         cam = camera[0]
#         vector = vectors[channel]
#
#         for _, bbox in bboxes_sorted.items():
#             occ_car = True
#             loc = bbox[0].location
#             size = [bbox[0].extent.y * 2, bbox[0].extent.x * 2, bbox[0].extent.z * 2]  # width = y, length = x, height = z
#
#             print(f'\nobject_loc: {loc.x, loc.y}')
#             # Get bbox corners in ego frame
#             corners_ego = get_corners_ego(loc, size)
#             corners_projected = np.zeros((2, 8))
#
#             # Get the world to camera matrix
#             w2c = np.array(cam.get_transform().get_inverse_matrix())
#
#             # Get ray from one corner to check if object in front of cam
#             ray = bbox[0].location - vehicle.get_transform().location
#             ray = np.array((ray.x, ray.y, ray.z))
#
#             if vector.dot(ray) > 1:
#                 for i in range(corners_ego.shape[1]):
#                     # Format the input coordinate (loc is a carla.Position object)
#                     point = np.array([corners_ego[0][i], corners_ego[1][i], corners_ego[2][i], 1])
#
#                     # transform to camera coordinates
#                     point_camera = np.dot(w2c, point)
#                     point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
#
#                     # now project 3D->2D using the camera matrix
#                     point_img = np.dot(K_car, point_camera)
#                     # normalize
#                     point_img[0] /= point_img[2]
#                     point_img[1] /= point_img[2]
#
#                     object_pos = point_img[0:2]
#                     corners_projected[:, i] = object_pos
#
#                 if len(corners_projected) != 0:
#                     corner_edges = find_edges(corners_projected)
#                     print(f'corner_edges: {corner_edges}')
#                     seg_image = segmented[channel]
#                     seg_raw = process_image_seg(seg_image)
#                     # Get bbox in segmented image
#                     bbox_seg, in_image = get_bbox_seg(corner_edges, seg_raw, img_w, img_h)
#
#                     if bbox_seg is not None:
#                         occluded, vis_car = get_vis_seg(bbox_seg, bbox[1], instance_ids)
#                         if not occluded:
#                             occ_car = False
#
#             bboxes_out[bbox] = occ_car
#         print(f'saved_ids: {instance_ids}')
#
#     return bboxes_out, vis_car
#
#
# def process_image_seg(image):
#     """Convert image to numpy array"""
#     # Get raw image in 8bit format
#     raw_image = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
#     # Reshape image to RGBA
#     raw_image = np.reshape(raw_image, (image.height, image.width, 4))
#     # Taking only RGB
#     seg_raw = raw_image[:, :, :3]
#
#     return seg_raw
#
#
# def get_vis_seg(bbox_seg, bb_id, instance_ids):
#     occluded = True
#
#     # Assign id to vehicle TODO: Add pedestrians later!
#     seg_tag = bb_id[1].split('.')[0]
#     if seg_tag == 'vehicle':
#         seg_id = 10
#     else:
#         seg_id = -1
#
#     # Slice just the tags and count number of tags in bbox
#     bbox_sem_tags = bbox_seg[:, :, 2]
#     non_occ_tags = np.sum(bbox_sem_tags == seg_id)
#     print(f'non_occ_tags: {non_occ_tags}')
#
#     # If bbox contains tag, check if object not occluded behind similar object
#     if non_occ_tags > 0:
#         bbox_sem_ids = list(zip(*bbox_seg[:, :, :2]))
#         bbox_sem_ids = [(tuple(arr.tolist())) for tup in bbox_sem_ids for arr in tup]     # TODO: Could be optimized?
#
#         if len(instance_ids) != 0:
#             # Count IDS that are not in saved IDs
#             non_occ_ids = len(set(bbox_sem_ids) - set(instance_ids))
#             print(f'instance_ids: {instance_ids}')
#             print(f'non_occ_ids: {non_occ_ids}')
#
#             if non_occ_ids < non_occ_tags:
#                 non_occ_total = non_occ_ids
#             else:
#                 non_occ_total = non_occ_tags
#                 # Find ids and add to list
#                 for i in range(bbox_sem_tags.shape[0]):
#                     j = np.random.randint(bbox_sem_tags.shape[0])
#                     random_loc = [bbox_sem_tags[j][j]]
#                     if random_loc == seg_id:
#                         break
#                 bbox_sem_ids = bbox_seg[:, :, :2]
#                 id_to_save = (bbox_sem_ids[j][j][0], bbox_sem_ids[j][j][1])
#                 instance_ids.append(id_to_save)
#             non_occ_total = non_occ_tags    # TODO: REMOVE LINE!
#         else:
#             non_occ_total = non_occ_tags
#             for i in range(bbox_sem_tags.shape[0]):
#                 j = np.random.randint(bbox_sem_tags.shape[0])
#                 random_loc = [bbox_sem_tags[j][j]]
#                 if random_loc == seg_id:
#                     break
#             bbox_sem_ids = bbox_seg[:, :, :2]
#             id_to_save = (bbox_sem_ids[j][j][0], bbox_sem_ids[j][j][1])
#             instance_ids.append(id_to_save)
#     else:
#         non_occ_total = non_occ_tags
#
#     # Calculate visibility
#     total_values = bbox_seg[:, :, 0].size
#     visibility = non_occ_total / total_values
#
#     print(f'visibility: {visibility}')
#
#     if visibility > 0.05:
#         occluded = False
#
#     return occluded, visibility
#
#
# def get_bbox_seg(min_max, seg_img, img_w, img_h):
#     # input corners = top_left --> bottom_left --> bottom_right --> top_right
#     # Image origin top left corner
#     in_image = False
#
#     x_min = int(min_max[0])
#     x_max = int(min_max[1])
#     y_min = int(min_max[2])
#     y_max = int(min_max[3])
#
#     if ((img_w > x_min > 0) or (0 < x_max < img_w)) and ((img_h > y_min > 0) or (0 < y_max < img_h)):
#         if x_min < 0:
#             x_min = 0
#         if y_min < 0:
#             y_min = 0
#         if x_max > img_w:
#             x_max = img_w - 1
#         if y_max > img_h:
#             y_max = img_h - 1
#
#         in_image = True
#         bbox_seg = seg_img[y_min:y_max+1, x_min:x_max+1]
#     else:
#         bbox_seg = None
#
#
#     # print(f'bbox_seg SEMANTIC ID: {bbox_seg[:, :, 0]}')
#     # print(f'bbox_seg G_VALUE: {bbox_seg[:, :, 1]}')
#     # print(f'bbox_seg B_VALUE: {bbox_seg[:, :, 2]}')
#
#     return bbox_seg, in_image

