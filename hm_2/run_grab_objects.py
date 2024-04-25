"""Запуск моделирования манипулятора CRP-RA27-80."""

import time

from hm_2.manipulator import Manipulator
from hm_2.client_tools import *
import numpy as np
from hm_2.math_tools import *
from numpy.typing import NDArray
import vacuum_clamp as vacuum

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


def get_target_tf(pose):
    target_tf = tf_from_orientation(
        pose[0], pose[1], pose[2], 0, DEG * 90, 0
    )
    return target_tf


def move_to_object_traj(start_pose, obj_pose, obj_height, step) -> list[NDArray]:
    # TODO: КОСТЫЛЬ!
    obj_pose[0] -= 0.1
    obj_pose[2] += obj_height

    step_j = step
    stop_z_coeff = 10
    move_j_end = np.array([obj_pose[0], obj_pose[1], stop_z_coeff * obj_height])
    move_j = points_from_line(p_start=start_pose, p_end=move_j_end, step=step_j)

    move_l_to_obj = points_from_line(
        p_start=move_j[-1],
        p_end=np.array([move_j[-1][0], move_j[-1][1], obj_pose[2]]),
        step=step,
    )

    is_points_reachable(move_j, 'move_j')
    is_points_reachable(move_l_to_obj, 'move_l_to_obj')

    return move_j + move_l_to_obj


if __name__ == '__main__':

    client_id = init_client_id()
    coord_ids = get_coord_ids(client_id)
    cube_id = get_object_id(client_id, 'cube')

    crp_ra = Manipulator(
        client_id=client_id,
        coord_ids=coord_ids,
        dh_params=DH_PARAMS,
        BFGS_accuracy=1e-7,
    )

    init_cube_pose = get_object_coords(client_id, cube_id)
    print(f'Начальная позиция кубика {init_cube_pose}')
    step = 2e-2
    cube_height = 0.1

    p_home = crp_ra.calc_clamp_xyz()
    to_object_traj = move_to_object_traj(
        start_pose=p_home, obj_pose=init_cube_pose, obj_height=cube_height, step=step)

    move_l_up = points_from_line(
        p_start=to_object_traj[-1],
        p_end=np.array([to_object_traj[-1][0], to_object_traj[-1][1], to_object_traj[-1][2] + cube_height*4]),
        step=step,
    )

    move_l_xy = points_from_line(
        p_start=move_l_up[-1],
        p_end=np.array([-move_l_up[-1][0], move_l_up[-1][1], move_l_up[-1][2]]),
        step=step,
    )

    move_l_home = points_from_line(
        p_start=move_l_xy[-1],
        p_end=p_home,
        step=step,
    )

    move_l_with_obj = move_l_up + move_l_xy

    # расчет положениий движения к объекту
    to_object_traj_q_history = []
    for p in to_object_traj:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, DEG*90, 0)
        to_object_traj_q_history.append(crp_ra.solve_ik(target_tf))

    # расчет положениий движения c захваченым объектом
    move_with_obj_q_history = []
    for p in move_l_with_obj:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, DEG*90, 0)
        move_with_obj_q_history.append(crp_ra.solve_ik(target_tf))

    # расчет положениий движения в точку старта
    move_home_q_history = []
    for p in move_l_home:
        target_tf = tf_from_orientation(p[0], p[1], p[2], 0, DEG*90, 0)
        move_home_q_history.append(crp_ra.solve_ik(target_tf))

    for coords in to_object_traj_q_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.05)

    vacuum.grab_object(client_id, 'flange', 'cube', True)

    for coords in move_with_obj_q_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.05)

    vacuum.grab_object(client_id, 'flange', 'cube', False)

    for coords in move_home_q_history:
        crp_ra.move_by_coords(coords)
        time.sleep(0.05)
