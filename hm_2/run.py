import sys
import sim.sim as sim
import numpy as np


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



def get_transform(dh_params: dict, ):
    dh_params
    d, a, thetta, alpha
    return np.array([
        [c(thetta), -c(alpha)*s(thetta), s(alpha)*s(thetta), a*c(thetta)],
        [s(thetta), c(alpha)*c(thetta), -s(alpha)*c(thetta), a*s(thetta)],
        [0, s(alpha), c(alpha), d],
        [0, 0, 0, 1]
    ])



if __name__ == '__main__':

    id = init_client_id()
    print(f'id = {id}')
    joints = get_q_ids(id)


    poses_1 = get_q(client_id=id, j_ids=joints)

    q = [0, 0, 0, 0, 0]
    move(id, joints, q)
    poses_2 = get_q(client_id=id, j_ids=joints)

    pass