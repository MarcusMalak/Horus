import numpy as np


def get_camera_intrinsic(camera_bp, width, height):
    # Build the K projection matrix:
    # K = [[Fx,  0, image_w/2],
    #      [ 0, Fy, image_h/2],
    #      [ 0,  0,         1]]

    fov = camera_bp.get_attribute("fov").as_float()
    focal = width / (2.0 * np.tan(fov * np.pi / 360.0))

    # In this case Fx and Fy are the same since the pixel aspect
    # ratio is 1
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = width / 2.0
    K[1, 2] = height / 2.0

    K_out = [[K[0, 0], K[0, 1], K[0, 2]], [K[1, 0], K[1, 1], K[1, 2]], [K[2, 0], K[2, 1], K[2, 2]]]

    return K_out

def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K