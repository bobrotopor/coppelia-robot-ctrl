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


class Point(object):
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


def get_flat_circle_params(a: Point, b: Point, c: Point) -> tuple[float]:

    if a.z != b.z or a.z != c.z:
        raise Exception('z-coords must be equal for the circle !')
    z = a.z

    k1 = a.x**2 - b.x**2 + a.y**2 - b.y**2
    k2 = a.x ** 2 - c.x ** 2 + a.y ** 2 - c.y ** 2
    k3 = (a.x - b.x) / (a.x - c.x)

    center_y = 0.5 * (k1 - k2 * k3) / (a.y - b.y - k3 * (a.y - c.y))
    center_x = 0.5 * (k2 - 2 * center_y * (a.y - c.y)) / (a.x - c.x)

    r = np.sqrt((a.x - center_x)**2 + (a.y - center_y)**2)

    return (center_x, center_y, z, r)


def get_circle_points_arr(circle_params: tuple, num: int) -> list[Point]:

    center_x = circle_params[0]
    center_y = circle_params[1]
    z = circle_params[2]
    r = circle_params[3]

    tf_2d = np.array([
        [1, 0, center_x],
        [0, 1, center_y],
        [0, 0, 1],
    ])

    phi_arr = np.linspace(start=0, stop=2*np.pi, num=num)

    circle_points = []
    for phi in phi_arr:
        p = tf_2d @ np.array([r * np.cos(phi), r * np.sin(phi), 1])
        x = p[0]
        y = p[1]
        circle_points.append(Point(x, y, z))

    return circle_points