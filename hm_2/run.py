"""Запуск моделирования манипулятора CRP-RA27-80."""

import time

from hm_2.manipulator import Manipulator
from hm_2.client_tools import init_client_id, get_coord_ids
import numpy as np
from hm_2.math_tools import get_flat_circle_params, points_from_circle_params, tf_from_orientation, Point

PI = np.pi
DEG = PI / 180

DH_PARAMS = (
    {'a': 0.140, 'alpha': PI/2,  'd': 0.4485, 'theta': PI/2},
    {'a': 0.640, 'alpha': 0,     'd': 0,      'theta': PI/2},
    {'a': 0.160, 'alpha': PI/2,  'd': 0,      'theta': 0},
    {'a': 0,     'alpha': PI/2,  'd': 1.078,  'theta': PI},
    {'a': 0.101, 'alpha': 0,     'd': 0,      'theta': 0},
)


if __name__ == '__main__':

    client_id = init_client_id()
    coord_ids = get_coord_ids(client_id)
    crp_ra = Manipulator(client_id=client_id, coord_ids=coord_ids, dh_params=DH_PARAMS)

    a = Point(0.0,  1.6, 0.8)
    b = Point(0.6,  0.9, 0.8)
    c = Point(-0.6, 0.9,0.8)

    circle_params = get_flat_circle_params(a, b, c)
    points = points_from_circle_params(
        circle_params=circle_params,
        num=80,
        start_from=90 * DEG,
    )

    crp_ra_max_len = 0.4485 + 0.640 + 1.078 + 0.101
    for p in points:
        if np.sqrt(p.x**2 + p.y**2 + p.z**2) > crp_ra_max_len:
            raise Exception('Одна из точек окружности находится вне рабочей зоны манипулятора!')

    coords_history = []
    for p in points:
        target_tf = tf_from_orientation(p.x, p.y, p.z, 0, 0, 0)
        coords_history.append(crp_ra.solve_ik(target_tf))

    for coords in coords_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.07)
