"""Класс манипулятора."""

import numpy as np
from numpy.typing import NDArray
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation


from hm_2.client_tools import get_curr_coords, move
from math import sin, cos


class Manipulator:
    """Манипулятор."""

    def __init__(self, dh_params: tuple[dict], client_id, coord_ids):
        self.dh_params = dh_params
        self.client_id = client_id
        self.coord_ids = coord_ids
        self.coord_num = len(dh_params)

        # текущие значения обобщённых координат
        self.curr_coords = None
        self.upd_curr_coords()
        # матрица перехода от текущего положения СК схвата до мировой СК
        self.clamp_tf = self.calc_clamp_tf(self.curr_coords)


    def upd_curr_coords(self) -> NDArray:
        """Обновить текущие значения обобщённых координат из CoppeliaSim."""
        self.curr_coords = get_curr_coords(client_id=self.client_id, coord_ids=self.coord_ids)
        return self.curr_coords

    def calc_clamp_tf(self, curr_coords: NDArray) -> NDArray:
        """Обновить текущее значение матрицы перехода от СК схвата до мировой СК."""
        clamp_tf = np.eye(4)
        for idx in range(self.coord_num):
            tf_i = self.trans_from_coord_num(curr_coords=curr_coords, coord_num=idx)
            clamp_tf = clamp_tf.dot(tf_i)
        return clamp_tf

    def calc_clamp_xyz(self) -> NDArray:
        self.upd_curr_coords()
        clamp_tf = self.calc_clamp_tf(self.curr_coords)
        return clamp_tf[:3, 3]

    def move_by_coords(self, target_coords: tuple | list):
        """Переместить манипулятор в CoppeliaSim в соответсвтии с
        заданными значениями обобщенных координат.

        :param target_coords: [рад], целевой кортеж углов поворота обобщенных координат."""
        move(client_id=self.client_id, coord_ids=self.coord_ids, q=target_coords)

    def trans_from_coord_num(self, curr_coords: NDArray, coord_num: int):
        """Получить матрицу перехода T_i от i до i+1 СК."""

        a = self.dh_params[coord_num]['a']
        alpha = self.dh_params[coord_num]['alpha']
        d = self.dh_params[coord_num]['d']
        theta = self.dh_params[coord_num]['theta'] + curr_coords[coord_num]

        return np.array([
            [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
            [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def calc_pose_err(self, target_tf) -> NDArray:
        # Extract position and rotation from current_pose
        curr_trans = self.clamp_tf[:3, 3]
        curr_rot = Rotation.from_matrix(self.clamp_tf[:3, :3])

        # Extract position and rotation from target_pose
        target_trans = target_tf[:3, 3]
        target_rot = Rotation.from_matrix(target_tf[:3, :3])

        # Compute position error
        pose_err = target_trans - curr_trans

        # Compute rotation error using Euler angles
        rot_err = target_rot.inv() * curr_rot
        rot_err = rot_err.as_euler('xyz')

        # Combine position and rotation error into a single vector
        error = np.hstack((pose_err, rot_err))
        return error

    def solve_ik(self, target_tf):
        """Решение ОЗК."""
        # error function
        self.upd_curr_coords()
        def get_normed_err(curr_coords: NDArray):
            self.clamp_tf = self.calc_clamp_tf(curr_coords)
            return np.linalg.norm(self.calc_pose_err(target_tf))

        # Use BFGS method to minimize the err function
        result = minimize(get_normed_err, self.curr_coords, method='BFGS', options={'eps': 10e-7})
        return result.x
