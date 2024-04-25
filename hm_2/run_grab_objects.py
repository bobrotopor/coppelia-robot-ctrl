"""Запуск моделирования манипулятора CRP-RA27-80."""

import time

from hm_2.manipulator import Manipulator
from hm_2.client_tools import *
import numpy as np
from hm_2.math_tools import get_flat_circle_params, points_from_circle_params, points_from_line, \
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


def get_traj_to_grab_object(start_pose, obj_pose, obj_height, step):
    step_j = step
    stop_z_coeff = 3
    move_j_end = np.array([obj_pose[0], obj_pose[1], stop_z_coeff * obj_height])
    move_j = points_from_line(p_start=start_pose, p_end=move_j_end, step=step_j)


    move_l_to_obj = points_from_line(
        p_start=move_j[-1],
        p_end=np.array([move_j[-1][0], move_j[-1][1], cube_len]),
        step=step,
    )

    move_l_with_obj = points_from_line(
        p_start=np.array([move_j[-1][0], move_j[-1][1], cube_len]),
        p_end=move_j[-1],
        step=step,
    )


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
    step = 1e-2
    z = 0.1
    z_with_obj = z * 5

    cube_len = 0.1

    p_home = crp_ra.calc_clamp_xyz()

    l_to_cube = points_from_line(
        p_start=p_home,
        p_end=np.array([init_cube_pose[0], init_cube_pose[1], init_cube_pose[2] + cube_len/2]),
        step=step,
    )
    last_pose = l_to_cube[-1]

    l_up_with_cube = points_from_line(
        p_start=last_pose,
        p_end=np.array([last_pose[0], last_pose[1], last_pose[2] + z_with_obj]),
        step=step,
    )
    is_points_reachable(l_to_cube, 'l_to_cube')
    is_points_reachable(l_up_with_cube, 'l_up_with_cube')


    points_seq = l_to_cube + l_up_with_cube

    # расчет положениий
    coords_history = []
    for p in points_seq:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, DEG*90, 0)
        coords_history.append(crp_ra.solve_ik(target_tf))

    # анимация
    for coords in coords_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.05)


