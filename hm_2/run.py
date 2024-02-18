import sys
import sim.sim as sim


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


if __name__ == '__main__':

    id = init_id()
    print(f'id = {id}')
