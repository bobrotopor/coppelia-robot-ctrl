import sys
import sim.sim as sim
import numpy as np

from numpy.typing import NDArray


def init_client_id():
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


def get_coord_ids(client_id) -> list:
    """Получить список id обобщённых координат манипулятора."""
    coord_ids = [0, 0, 0, 0, 0]
    err, coord_ids[0] = sim.simxGetObjectHandle(client_id, 'j_1', sim.simx_opmode_oneshot_wait)
    err, coord_ids[1] = sim.simxGetObjectHandle(client_id, 'j_2', sim.simx_opmode_oneshot_wait)
    err, coord_ids[2] = sim.simxGetObjectHandle(client_id, 'j_3', sim.simx_opmode_oneshot_wait)
    err, coord_ids[3] = sim.simxGetObjectHandle(client_id, 'j_4', sim.simx_opmode_oneshot_wait)
    err, coord_ids[4] = sim.simxGetObjectHandle(client_id, 'j_5', sim.simx_opmode_oneshot_wait)
    return coord_ids


def get_curr_coords(client_id, coord_ids) -> NDArray:
    """Получить список текущих значений обобщённых координат манипулятора."""
    coords = np.zeros(5)
    error, coords[0] = sim.simxGetJointPosition(client_id, coord_ids[0], sim.simx_opmode_oneshot_wait)
    error, coords[1] = sim.simxGetJointPosition(client_id, coord_ids[1], sim.simx_opmode_oneshot_wait)
    error, coords[2] = sim.simxGetJointPosition(client_id, coord_ids[2], sim.simx_opmode_oneshot_wait)
    error, coords[3] = sim.simxGetJointPosition(client_id, coord_ids[3], sim.simx_opmode_oneshot_wait)
    error, coords[4] = sim.simxGetJointPosition(client_id, coord_ids[4], sim.simx_opmode_oneshot_wait)
    return coords


def move(client_id, coord_ids, q) -> None:
    error = sim.simxSetJointTargetPosition(client_id, coord_ids[0], q[0], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, coord_ids[1], q[1], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, coord_ids[2], q[2], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, coord_ids[3], q[3], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, coord_ids[4], q[4], sim.simx_opmode_streaming)
