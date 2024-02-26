"""Запуск моделирования манипулятора CRP-RA27-80."""

from hm_2.manipulator import Manipulator
from hm_2.client_tools import init_client_id, get_coord_ids
import numpy as np


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

    #crp_manipulator.move_to_target((2.57, 1, -1, 0, 0))

    # for idx in range(crp_ra.coord_num):
    #     print(f'А{idx}:\n')
    #     print(crp_ra.trans_from_coord_num(idx))

    print(crp_ra.clamp_tf)
    print(crp_ra.clamp_tf @ np.array([0, 0, 0, 1]))
