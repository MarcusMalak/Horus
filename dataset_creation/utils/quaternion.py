from pyquaternion import Quaternion


def get_quaternion_from_euler(pitch, yaw, roll):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """

    q1 = Quaternion(axis=[0, 1, 0], degrees=pitch)
    q2 = Quaternion(axis=[0, 0, 1], degrees=yaw)
    q3 = Quaternion(axis=[1, 0, 0], degrees=roll)

    quat = q1 * q2 * q3

    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    return [qw, qx, qy, qz]

