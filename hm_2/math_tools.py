import numpy as np
from scipy.spatial.transform import Rotation


def rot_x(theta: float):
    return np.array([
        [ 1,             0,              0],
        [ 0, np.cos(theta), -np.sin(theta)],
        [ 0, np.sin(theta),  np.cos(theta)],
    ])


def rot_y(theta: float):
    return np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [ 0            , 1, 0            ],
        [-np.sin(theta), 0, np.cos(theta)],
    ])


def rot_z(theta: float):
    return np.array([
        [ np.cos(theta), -np.sin(theta), 0],
        [ np.sin(theta), np.cos(theta) , 0],
        [ 0            , 0             , 1],
    ])


def xyz_to_pose(x: float, y: float, z: float, phi: float, theta: float, psi: float):
    pose = np.eye(4)
    rot = rot_z(psi) @ rot_y(theta) @ rot_x(phi)
    pose[:3, :3] = rot
    pose[0, 3] = x
    pose[1, 3] = y
    pose[2, 3] = z
    return pose
