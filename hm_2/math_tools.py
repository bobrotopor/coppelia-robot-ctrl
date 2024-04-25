"""Модуль математики."""

import numpy as np
from numpy.typing import NDArray


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


def get_flat_circle_params(a: NDArray, b: NDArray, c: NDArray) -> tuple[float]:

    if len(a) !=3 or len(b) !=3 or len(c) !=3:
        raise Exception('need 3 dim coords !')

    if a[2] != b[2] or a[2] != c[2]:
        raise Exception('z-coords must be equal for the circle !')

    z = a[2]
    x_a = a[0]
    x_b = b[0]
    x_c = c[0]
    y_a = a[1]
    y_b = b[1]
    y_c = c[1]

    k1 = x_a**2 - x_b**2 + y_a**2 - y_b**2          # k1 = a.x**2 - b.x**2 + a.y**2 - b.y**2
    k2 = x_a ** 2 - x_c ** 2 + y_a ** 2 - y_c ** 2  # k2 = a.x ** 2 - c.x ** 2 + a.y ** 2 - c.y ** 2
    k3 = (x_a - x_b) / (x_a - x_c)                  # k3 = (a.x - b.x) / (a.x - c.x)

    center_y = 0.5 * (k1 - k2 * k3) / (y_a - y_b - k3 * (y_a - y_c))
    center_x = 0.5 * (k2 - 2 * center_y * (y_a - y_c)) / (x_a - x_c)

    r = np.sqrt((x_a - center_x)**2 + (y_a - center_y)**2)

    return (center_x, center_y, z, r)


def points_from_circle_params(circle_params: tuple, num: int, start_from: float = 0) -> list[NDArray]:
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
        circle_points.append(np.array([x, y, z]))

    return circle_points


def points_from_line(p_start: NDArray, p_end: NDArray, step: float) -> list[NDArray]:
    """Получить массив точек по отрезку прямой в пространстве."""
    # TODO: важно сделать так, что бы разбиение билось с длиной отрезка!
    if len(p_start) !=3 or len(p_end) !=3:
        raise Exception('need 3 dim coords !')

    # создаем и нормируем направляющий вектор
    dir_vec = p_end - p_start
    len_dir_vec = np.sqrt(sum(dir_vec**2))
    dir_vec *= 1 / len_dir_vec

    num_points = int(np.ceil(len_dir_vec / step))
    real_step = len_dir_vec / num_points

    point = p_start
    line_points = []
    line_points.append(point)
    for point_idx in range(num_points):
        point = point + real_step * dir_vec
        line_points.append(point)

    print(f'last point {line_points[-1]}')
    return line_points

