import sys
import sim.sim as sim
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


class Manipulator:
    # Initialize robot arm by DH and joints' coordinates
    def __init__(self, dh_params, sim_joints):
        self.dh_params_arr = []
        self.joints_val = np.array([])
        self.sim_joints = sim_joints
        for j in joints_DH:
            self.dh_params_arr.append(j)
            self.joints_val = np.append(self.joints_val, 0)

    # Define function of FK
    def forward_kinematics(self, joint_coord):
        DH = self.dh_params_arr.copy()
        self.joints_val = np.array(joint_coord)
        DH_Joints = []
        for i in range(len(DH)):
            temp = {}
            x = DH[i]
            temp['theta'] = x['theta'] + self.joints_val[i]
            temp['alpha'] = x['alpha']
            temp['d'] = x['d']
            temp['a'] = x['a']
            DH_Joints.append(temp)

        # transformation_matrices = [comp_trans_matrix(**params) for params in DH_Joints]
        transformation_matrices = [comp_trans_matrix(DH) for DH in DH_Joints]
        # Multiply all transformation matrices together
        final = np.eye(4)
        for T in transformation_matrices:
            final = final.dot(T)

        return final

    def solveIK(self, target_mat):
        # error function
        def err_cb(joint_angles):
            curPose = self.forward_kinematics(joint_angles)
            return np.linalg.norm(error_pos(curPose, target_mat))

        # Use BFGS method to minimize the err function
        result = minimize(err_cb, self.joints_val, method='BFGS', options={'eps': 10e-7})
        return result.x

    def get_q_ids(client_id):
        """Получить список id обобщённых координат манипулятора."""
        q_ids = np.array([0, 0, 0, 0, 0])
        err, q_ids[0] = sim.simxGetObjectHandle(client_id, 'j_1', sim.simx_opmode_oneshot_wait)
        err, q_ids[1] = sim.simxGetObjectHandle(client_id, 'j_2', sim.simx_opmode_oneshot_wait)
        err, q_ids[2] = sim.simxGetObjectHandle(client_id, 'j_3', sim.simx_opmode_oneshot_wait)
        err, q_ids[3] = sim.simxGetObjectHandle(client_id, 'j_4', sim.simx_opmode_oneshot_wait)
        err, q_ids[4] = sim.simxGetObjectHandle(client_id, 'j_5', sim.simx_opmode_oneshot_wait)
        return q_ids

    def get_q(client_id, j_ids):
        """Получить список текущих значений обобщённых координат манипулятора."""
        q = np.zeros(5)
        error, q[0] = sim.simxGetJointPosition(client_id, j_ids[0], sim.simx_opmode_oneshot_wait)
        error, q[1] = sim.simxGetJointPosition(client_id, j_ids[1], sim.simx_opmode_oneshot_wait)
        error, q[2] = sim.simxGetJointPosition(client_id, j_ids[2], sim.simx_opmode_oneshot_wait)
        error, q[3] = sim.simxGetJointPosition(client_id, j_ids[3], sim.simx_opmode_oneshot_wait)
        error, q[4] = sim.simxGetJointPosition(client_id, j_ids[4], sim.simx_opmode_oneshot_wait)
        return q

    def move(client_id, q_ids, q):
        error = sim.simxSetJointTargetPosition(client_id, q_ids[0], q[0], sim.simx_opmode_streaming)
        error = sim.simxSetJointTargetPosition(client_id, q_ids[1], q[1], sim.simx_opmode_streaming)
        error = sim.simxSetJointTargetPosition(client_id, q_ids[2], q[2], sim.simx_opmode_streaming)
        error = sim.simxSetJointTargetPosition(client_id, q_ids[3], q[3], sim.simx_opmode_streaming)
        error = sim.simxSetJointTargetPosition(client_id, q_ids[4], q[4], sim.simx_opmode_streaming)
