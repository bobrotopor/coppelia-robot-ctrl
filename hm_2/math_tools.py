"""Модуль математики."""

import numpy as np


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


def tf_from_orientation(x: float, y: float, z: float, phi_x: float, theta_y: float, psi_z: float):
    """Получить матрицу перехода по ориентации твердого тела в пространсвте - 6 координат."""
    tf = np.eye(4)
    rot = rot_z(psi_z) @ rot_y(theta_y) @ rot_x(phi_x)
    tf[:3, :3] = rot
    tf[0, 3] = x
    tf[1, 3] = y
    tf[2, 3] = z
    return tf


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


def points_from_circle_params(circle_params: tuple, num: int, start_from: float = 0) -> list[Point]:
    """Получить """
    center_x = circle_params[0]
    center_y = circle_params[1]
    z = circle_params[2]
    r = circle_params[3]

    tf_2d = np.array([
        [1, 0, center_x],
        [0, 1, center_y],
        [0, 0, 1],
    ])

    phi_step = 2*np.pi / num
    circle_points = []
    for idx in range(num):
        phi = idx * phi_step + start_from
        p = tf_2d @ np.array([r * np.cos(phi), r * np.sin(phi), 1])
        x = p[0]
        y = p[1]
        circle_points.append(Point(x, y, z))

    return circle_points


def points_from_line_segment(p_start: Point, p_end: Point, step: float):
    """Получить массив точек по отрезку прямой в пространстве."""
    # TODO: важно сделать так, что бы разбиение билось с длиной отрезка!

    # направляющий вектор
    vec = np.zeros(3)
    vec[0] = p_end.x - p_start.x
    vec[1] = p_end.y - p_start.y
    vec[2] = p_end.z - p_start.z
    # нормируем направляющий вектор
    vec *= 1 / np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

