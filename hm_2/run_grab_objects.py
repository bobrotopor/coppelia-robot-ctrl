"""Запуск моделирования манипулятора CRP-RA27-80."""

import time

from hm_2.manipulator import Manipulator
from hm_2.client_tools import *
import numpy as np
from hm_2.math_tools import get_flat_circle_params, points_from_circle_params, points_from_line_segment, \
    tf_from_orientation
from numpy.typing import NDArray


PI = np.pi
DEG = PI / 180

DH_PARAMS = (
    {'a': 0.140, 'alpha': PI/2,  'd': 0.4485, 'theta': PI/2},
    {'a': 0.640, 'alpha': 0,     'd': 0,      'theta': PI/2},
    {'a': 0.160, 'alpha': PI/2,  'd': 0,      'theta': 0},
    {'a': 0,     'alpha': PI/2,  'd': 1.078,  'theta': PI},
    {'a': 0.101, 'alpha': 0,     'd': 0,      'theta': 0},
)
R_MAX = 1.865
R_MIN = 0.395


def is_points_reachable(points: list[NDArray], points_seq_name: str = '') -> bool:
    """Достижимы ли манипулятором заданные точки."""

    for p in points:
        p_radius = np.sqrt(sum(p**2))
        if p_radius < R_MIN:
            raise Exception(
                f'{points_seq_name }'
                'Одна из точек траектории имеет радиус-вектор меньший '
                'чем минимальный радиус достижимости!'
            )
        if p_radius > R_MAX:
            raise Exception(
                f'{points_seq_name}'
                'Одна из точек траектории имеет радиус-вектор больший '
                'чем максимальный радиус достижимости!'
            )
    return True


if __name__ == '__main__':

    client_id = init_client_id()
    coord_ids = get_coord_ids(client_id)
    cube_id = get_object_id(client_id, 'cube')

    crp_ra = Manipulator(
        client_id=client_id,
        coord_ids=coord_ids,
        dh_params=DH_PARAMS,
        BFGS_accuracy=1e-6,
    )

    init_cube_pose = get_object_coords(client_id, cube_id)
    step = 4e-2
    z = 0.1
    a = np.array([ init_cube_pose[0],   init_cube_pose[1], z])
    b = np.array([-init_cube_pose[0],   init_cube_pose[1], z])
    c = np.array([-init_cube_pose[0],  -init_cube_pose[1], z])

    circle_params = get_flat_circle_params(a, b, c)
    circle_points = points_from_circle_params(
        circle_params=circle_params,
        num=120,
        start_from=90 * DEG,
    )
    is_points_reachable(circle_points)

    p_home = crp_ra.calc_clamp_xyz()
    p_start_circle = a

    line_points = points_from_line_segment(p_start=p_home, p_end=p_start_circle, step=step)
    is_points_reachable(line_points)
    # print(line_points)
    points_seq = line_points + circle_points

    # расчет положениий
    coords_history = []
    for p in points_seq:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, DEG*90, 0)
        coords_history.append(crp_ra.solve_ik(target_tf))

    # анимация
    for coords in coords_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.07)
