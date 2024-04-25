import sim.sim as sim


def grab_object(client_id, master_obj_name, slave_obj_name, state: bool = False):
    [err, flange] = sim.simxGetObjectHandle(client_id, master_obj_name, sim.simx_opmode_blocking)
    print(flange)
    if err:
        print('master_obj: simxGetObjectHandle error')

    [err, cube] = sim.simxGetObjectHandle(client_id, slave_obj_name, sim.simx_opmode_blocking)
    if err:
        print('slave_obj: simxGetObjectHandle error')

    if state is True:
        err = sim.simxSetObjectIntParameter(client_id, cube, 3003, 1, sim.simx_opmode_blocking)
        if err:
            print('cube set param error')
        err = sim.simxSetObjectParent(client_id, cube, flange, True, sim.simx_opmode_blocking)
        if err:
            print('cube set parent error')
    else:
        err = sim.simxSetObjectIntParameter(client_id, cube, 3003, 0, sim.simx_opmode_blocking)

        if err:
            print('cube set param error')

        err = sim.simxSetObjectParent(client_id, cube, -1, False, sim.simx_opmode_blocking)

        if err:
            print('cube set parent error')