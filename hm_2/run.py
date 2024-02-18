import sys
import sim.sim as sim
import numpy as np

def init_id():
    sim.simxFinish(-1)  # just in case, close all opened connections
    client_id = sim.simxStart(
        '127.0.0.1',
        19999,
        True,
        True,
        5000,
        5,
    )

    if client_id != -1:  # check if client connection successful
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')

    return client_id


def get_joint_ids(client_id):
    """Получить список id звеньев."""
    j_id = np.array([0, 0, 0, 0, 0])
    err, j_id[0] = sim.simxGetObjectHandle(client_id, 'j_1', sim.simx_opmode_oneshot_wait)
    err, j_id[1] = sim.simxGetObjectHandle(client_id, 'j_2', sim.simx_opmode_oneshot_wait)
    err, j_id[2] = sim.simxGetObjectHandle(client_id, 'j_3', sim.simx_opmode_oneshot_wait)
    err, j_id[3] = sim.simxGetObjectHandle(client_id, 'j_4', sim.simx_opmode_oneshot_wait)
    err, j_id[4] = sim.simxGetObjectHandle(client_id, 'j_5', sim.simx_opmode_oneshot_wait)
    return j_id


def get_joint_poses(client_id, j_ids):
    """Получить список значений обобщённых координат."""
    q = np.zeros(5)
    error, q[0] = sim.simxGetJointPosition(client_id, j_ids[0], sim.simx_opmode_oneshot_wait)
    error, q[1] = sim.simxGetJointPosition(client_id, j_ids[1], sim.simx_opmode_oneshot_wait)
    error, q[2] = sim.simxGetJointPosition(client_id, j_ids[2], sim.simx_opmode_oneshot_wait)
    error, q[3] = sim.simxGetJointPosition(client_id, j_ids[3], sim.simx_opmode_oneshot_wait)
    error, q[4] = sim.simxGetJointPosition(client_id, j_ids[4], sim.simx_opmode_oneshot_wait)
    return q


def move(client_id, j_ids, q):
    error = sim.simxSetJointTargetPosition(client_id, j_ids[0], q[0], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j_ids[1], q[1], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j_ids[2], q[2], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j_ids[3], q[3], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j_ids[4], q[4], sim.simx_opmode_streaming)



if __name__ == '__main__':

    id = init_id()
    print(f'id = {id}')
    joints = get_joint_ids(id)


    poses_1 = get_joint_poses(client_id=id, j_ids=joints)

    q = [1, 1, 1.57, 1.57, 1]
    move(id, joints, q)
    poses_2 = get_joint_poses(client_id=id, j_ids=joints)

    pass