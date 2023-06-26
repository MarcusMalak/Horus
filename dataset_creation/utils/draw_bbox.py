import numpy as np
import cv2


def draw_bbox(world, vehicle, camera, camera_bp, image, twoD):
    # Get the world to camera matrix
    world_2_cam = np.array(camera.get_transform().get_inverse_matrix())

    # Get the attributes from the camera
    image_w = camera_bp.get_attribute("image_size_x").as_int()
    image_h = camera_bp.get_attribute("image_size_y").as_int()
    fov = camera_bp.get_attribute("fov").as_float()

    # Calculate the camera projection matrix to project from 3D -> 2D
    K = build_projection_matrix(image_w, image_h, fov)

    edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

    for npc in world.get_actors().filter('*vehicle*'):

        # Filter ego vehicle
        if npc.id != vehicle.id:
            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(vehicle.get_transform().location)

            # print(npc.bounding_box.extent)

            # Filter for the vehicles within 50m
            if dist < 150:
                # Calculate the dot product between the forward vector
                # of the vehicle and the vector between the vehicle
                # and the other vehicle. We threshold this dot product
                # to limit to drawing bounding boxes IN FRONT OF THE CAMERA

                # forward_vec = vehicle.get_transform().get_forward_vector()
                # backward_vec = vehicle.get_transform().get_backward_vector()
                ray = npc.get_transform().location - vehicle.get_transform().location

                # if forward_vec.dot(ray) > 1:

                p1 = get_image_point(bb.location, K, world_2_cam)
                verts = [v for v in bb.get_world_vertices(npc.get_transform())]

                if twoD:
                    x_min, x_max, y_min, y_max = convert_2d(verts, K, world_2_cam)
                    cv2.line(image, (int(x_min), int(y_min)), (int(x_max), int(y_min)), (0, 0, 255, 255), 1)
                    cv2.line(image, (int(x_min), int(y_max)), (int(x_max), int(y_max)), (0, 0, 255, 255), 1)
                    cv2.line(image, (int(x_min), int(y_min)), (int(x_min), int(y_max)), (0, 0, 255, 255), 1)
                    cv2.line(image, (int(x_max), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255, 255), 1)

                else:
                    for edge in edges:
                        p1 = get_image_point(verts[edge[0]], K, world_2_cam)
                        p2 = get_image_point(verts[edge[1]], K, world_2_cam)
                        cv2.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255, 0, 0, 255), 1)

    # for npc in world.get_actors().filter('*walker*'):
    #
    #     bb = npc.bounding_box
    #     dist = npc.get_transform().location.distance(vehicle.get_transform().location)
    #
    #     if dist < 50 and npc.bounding_box.extent.x > 0:
    #         p1 = get_image_point(bb.location, K, world_2_cam)
    #         verts = [v for v in bb.get_world_vertices(npc.get_transform())]
    #
    #         if twoD:
    #             x_min, x_max, y_min, y_max = convert_2d(verts, K, world_2_cam)
    #             cv2.line(image, (int(x_min), int(y_min)), (int(x_max), int(y_min)), (255, 0, 255, 255), 1)
    #             cv2.line(image, (int(x_min), int(y_max)), (int(x_max), int(y_max)), (255, 0, 255, 255), 1)
    #             cv2.line(image, (int(x_min), int(y_min)), (int(x_min), int(y_max)), (255, 0, 255, 255), 1)
    #             cv2.line(image, (int(x_max), int(y_min)), (int(x_max), int(y_max)), (255, 0, 255, 255), 1)
    #
    #         else:
    #             for edge in edges:
    #                 p1 = get_image_point(verts[edge[0]], K, world_2_cam)
    #                 p2 = get_image_point(verts[edge[1]], K, world_2_cam)
    #                 cv2.line(image, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255, 0, 255, 255), 1)
    return image


def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K


def get_image_point(loc, K, w2c):
    # Calculate 2D projection of 3D coordinate

    # Format the input coordinate (loc is a carla.Position object)
    point = np.array([loc.x, loc.y, loc.z, 1])
    # transform to camera coordinates
    point_camera = np.dot(w2c, point)

    # New we must change from UE4's coordinate system to a "standard"
    # (x, y ,z) -> (y, -z, x)
    # and we remove the fourth component also
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # now project 3D->2D using the camera matrix
    point_img = np.dot(K, point_camera)
    # normalize
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]


def convert_2d(verts, K, world_2_camera):
    x_max = -10000
    x_min = 10000
    y_max = -10000
    y_min = 10000

    for vert in verts:
        p = get_image_point(vert, K, world_2_camera)
        # Find the rightmost vertex
        if p[0] > x_max:
            x_max = p[0]
        # Find the leftmost vertex
        if p[0] < x_min:
            x_min = p[0]
        # Find the highest vertex
        if p[1] > y_max:
            y_max = p[1]
        # Find the lowest  vertex
        if p[1] < y_min:
            y_min = p[1]

    return x_min, x_max, y_min, y_max

