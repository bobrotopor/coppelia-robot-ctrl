"""Класс манипулятора."""

import numpy as np
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

        self.curr_coords = None         # текущие значения обобщённых координат
        self.clamp_tf = None            # матрица перехода от СК схвата до мировой СК

        self.upd_curr_coords()
        self.upd_clamp_pose()

    def upd_curr_coords(self):
        """Обновить текущие значения обобщённых координат из CoppeliaSim."""
        self.curr_coords = get_curr_coords(client_id=self.client_id, coord_ids=self.coord_ids)

    def upd_clamp_pose(self):
        """Обновить текущее значение матрицы перехода от СК схвата до мировой СК."""
        self.clamp_tf = np.eye(4)
        for idx in range(self.coord_num):
            tf_i = self.trans_from_coord_num(idx)
            self.clamp_tf = self.clamp_tf.dot(tf_i)

    def move_to_target(self, target: tuple):
        """Установить координаты манипулятора в CoppeliaSim по заданным значениям."""
        move(client_id=self.client_id, coord_ids=self.coord_ids, q=target)

    def trans_from_coord_num(self, coord_num: int):
        """Получить матрицу перехода T_i от i до i+1 СК."""

        a = self.dh_params[coord_num]['a']
        alpha = self.dh_params[coord_num]['alpha']
        d = self.dh_params[coord_num]['d']
        #theta = self.dh_params[coord_num]['theta']
        theta = self.dh_params[coord_num]['theta'] + self.curr_coords[coord_num]

        return np.array([
            [cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
            [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def calc_pose_err(self, target_tf):
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


    #
    # def forward_kinematics(self, joint_coord):
    #     DH = self.dh_params_arr.copy()
    #     self.joints_val = np.array(joint_coord)
    #     DH_Joints = []
    #     for i in range(len(DH)):
    #         temp = {}
    #         x = DH[i]
    #         temp['theta'] = x['theta'] + self.joints_val[i]
    #         temp['alpha'] = x['alpha']
    #         temp['d'] = x['d']
    #         temp['a'] = x['a']
    #         DH_Joints.append(temp)
    #
    #     # transformation_matrices = [comp_trans_matrix(**params) for params in DH_Joints]
    #     transformation_matrices = [comp_trans_matrix(DH) for DH in DH_Joints]
    #     # Multiply all transformation matrices together
    #     final = np.eye(4)
    #     for T in transformation_matrices:
    #         final = final.dot(T)
    #
    #     return final
    #
    # def solveIK(self, target_mat):
    #     # error function
    #     def err_cb(joint_angles):
    #         curPose = self.forward_kinematics(joint_angles)
    #         return np.linalg.norm(error_pos(curPose, target_mat))
    #
    #     # Use BFGS method to minimize the err function
    #     result = minimize(err_cb, self.joints_val, method='BFGS', options={'eps': 10e-7})
    #     return result.x
    #
