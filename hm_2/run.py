"""Запуск моделирования манипулятора CRP-RA27-80."""

import time

from hm_2.manipulator import Manipulator
from hm_2.client_tools import init_client_id, get_coord_ids
import numpy as np
from hm_2.math_tools import get_flat_circle_params, points_from_circle_params, tf_from_orientation
from numpy.typing import NDArray


PI = np.pi
DEG = PI / 180

DH_PARAMS = (
    {'a': 0, 'alpha': PI/2,  'd': 0.1519, 'theta': 0},
    {'a': -0.24365, 'alpha': 0,     'd': 0,      'theta': 0},
    {'a': -0.21325, 'alpha': 0,  'd': 0,      'theta': 0},
    {'a': 0,     'alpha': PI/2,  'd': 0.11235,  'theta': 0},
    {'a': 0, 'alpha': -PI/2,     'd': 0.0819,      'theta': 0},
)
R_MAX = 0.5
R_MIN = 0.00


def is_points_reachable(points: list[NDArray]) -> bool:
    """Достижимы ли манипулятором заданные точки."""

    for p in points:
        p_radius = np.sqrt(sum(p**2))
        if p_radius < R_MIN:
            raise Exception(
                'Одна из точек траектории имеет радиус-вектор меньший '
                'чем минимальный радиус достижимости!'
            )
        if p_radius > R_MAX:
            raise Exception(
                'Одна из точек траектории имеет радиус-вектор больший '
                'чем максимальный радиус достижимости!'
            )
    return True


if __name__ == '__main__':

    client_id = init_client_id()
    coord_ids = get_coord_ids(client_id)
    crp_ra = Manipulator(client_id=client_id, coord_ids=coord_ids, dh_params=DH_PARAMS)

    a = np.array([0.0,  1.6, 0.8])
    b = np.array([0.2,  1.3, 0.8])
    c = np.array([-0.2, 1.3, 0.8])

    circle_params = get_flat_circle_params(a, b, c)
    points = points_from_circle_params(
        circle_params=circle_params,
        num=80,
        start_from=90 * DEG,
    )

    is_points_reachable(points)

    # расчет положениий
    coords_history = []
    for p in points:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, 0, 0)
        coords_history.append(crp_ra.solve_ik(target_tf))

    # анимация
    for coords in coords_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.07)
