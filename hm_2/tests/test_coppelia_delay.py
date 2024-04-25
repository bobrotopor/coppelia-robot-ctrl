"""Тестируем скорость срабатывания коппелии.

Выяснил, что коппелия учитывает динамику, ПОЭТОМУ чем меньше перемещение,
тем быстрее оно будет осуществлено.

Звучит как бред, ибо массы не заданы!
"""

import time
import numpy as np

from hm_2.manipulator import Manipulator


def testing_delay(manip: Manipulator, delay_sec: float):
    print('initial coords: (0, 0, 0, 0, 0)')
    print(manip.clamp_tf)
    vec = manip.clamp_tf @ np.array([0, 0, 0, 1])
    print(f'x={vec[0]}\ny={vec[1]}\nz={vec[2]}\n')

    print('move to coords: (1.57, 0, 0, 0, 0)')
    manip.move_by_coords((1.57, 0, 0, 0, 0))
    time.sleep(delay_sec)

    manip.calc_curr_coords()
    manip.calc_clamp_tf()
    print(manip.clamp_tf)
    vec = manip.clamp_tf @ np.array([0, 0, 0, 1])
    print(f'x={vec[0]}\ny={vec[1]}\nz={vec[2]}\n')

    print('move to coords: (1.57, 0, 1.57, 0, 0)')
    manip.move_by_coords((1.57, 0, 1.57, 0, 0))
    time.sleep(delay_sec)

    manip.calc_curr_coords()
    manip.calc_clamp_tf()
    print(manip.clamp_tf)
    vec = manip.clamp_tf @ np.array([0, 0, 0, 1])
    print(f'x={vec[0]}\ny={vec[1]}\nz={vec[2]}\n')
