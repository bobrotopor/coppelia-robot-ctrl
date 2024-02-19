from hm_2.client_tools import get_curr_coords, move


class Manipulator:

    def __init__(self, dh_params, client_id, coord_ids):
        self.dh_params = dh_params
        self.client_id = client_id
        self.coord_ids = coord_ids

    def upd_curr_coords(self):
        self.coord_ids = get_curr_coords(client_id=self.client_id, coord_ids=self.coord_ids)

    def move_to_target(self, target: tuple):
        """Установить координаты манипулятора в CoppeliaSim по заданным значениям."""
        move(client_id=self.client_id, coord_ids=self.coord_ids, q=target)

    # # Define function of FK
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

#
# def get_transform(dh_params: dict, ):
#     dh_params
#     d, a, thetta, alpha
#     return np.array([
#         [c(thetta), -c(alpha)*s(thetta), s(alpha)*s(thetta), a*c(thetta)],
#         [s(thetta), c(alpha)*c(thetta), -s(alpha)*c(thetta), a*s(thetta)],
#         [0, s(alpha), c(alpha), d],
#         [0, 0, 0, 1]
#     ])
#
