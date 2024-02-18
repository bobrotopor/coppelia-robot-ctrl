import sys
import sim as sim
import numpy as np


def initID():
    sim.simxFinish(-1)  # just in case, close all opened connections
    client_id = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    if client_id != -1:  # check if client connection successful
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')

    return client_id


def initJ(client_id):
    j = np.array([0, 0, 0, 0, 0])
    error, j[0] = sim.simxGetObjectHandle(client_id, 'joint_1', sim.simx_opmode_oneshot_wait)
    error, j[1] = sim.simxGetObjectHandle(client_id, 'joint_2', sim.simx_opmode_oneshot_wait)
    error, j[2] = sim.simxGetObjectHandle(client_id, 'joint_3', sim.simx_opmode_oneshot_wait)
    error, j[3] = sim.simxGetObjectHandle(client_id, 'joint_4', sim.simx_opmode_oneshot_wait)
    error, j[4] = sim.simxGetObjectHandle(client_id, 'joint_5', sim.simx_opmode_oneshot_wait)
    return j

client_id = initID()
j = initJ(client_id)


def getJointPos():
    q = np.zeros(5)
    error, q[0] = sim.simxGetJointPosition(client_id, j[0], sim.simx_opmode_oneshot_wait)
    error, q[1] = sim.simxGetJointPosition(client_id, j[1], sim.simx_opmode_oneshot_wait)
    error, q[2] = sim.simxGetJointPosition(client_id, j[2], sim.simx_opmode_oneshot_wait)
    error, q[3] = sim.simxGetJointPosition(client_id, j[3], sim.simx_opmode_oneshot_wait)
    error, q[4] = sim.simxGetJointPosition(client_id, j[4], sim.simx_opmode_oneshot_wait)
    return q


def OZK(q0, tar):
    def f(q):
        return np.linalg.norm(tar - T(q)[5][0: 3, 3])

    res = sp.optimize.minimize(fun=f, x0=q0, method='BFGS', options={'eps': 1.e-8}).x
    return res[0: 5]


def move(q):
    error = sim.simxSetJointTargetPosition(client_id, j[0], q[0], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j[1], q[1], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j[2], q[2], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j[3], q[3], sim.simx_opmode_streaming)
    error = sim.simxSetJointTargetPosition(client_id, j[4], q[4], sim.simx_opmode_streaming)


def goHome():
    q = [0, 0, 0, 0, 0]
    move(q)
    cur_pos = T(q)[5][0: 3, 3]
    return cur_pos


def traceL(t1, t2):
    line = t2 - t1
    s = 0.01
    count = ceil(np.linalg.norm(line) / s)

    if count > 0:
        step = line / count
        return step, count
    else:
        return 0, 0


def moveL(cur_pos, tar):
    q = getJointPos()

    step, count = traceL(cur_pos, tar)
    for i in range(1, count + 1):
        q = OZK(q, cur_pos + step * i)
        move(q)

    return cur_pos + step * count


def calc(p1, p2, p3):
    C = np.zeros(3)
    C[2] = p1[2]
    C[1] = 0.5 * ((p1[0] - p2[0]) * (p1[0] * p1[0] - p3[0] * p3[0] + p1[1] * p1[1] - p3[1] * p3[1]) -
                (p1[0] - p3[0]) * (p1[0] * p1[0] - p2[0] * p2[0] + p1[1] * p1[1] - p2[1] * p2[1])) / \
         ((p1[0] - p2[0]) * (p1[1] - p3[1]) - (p1[0] - p3[0]) * (p1[1] - p2[1]))
    C[0] = (0.5 * (p1[0] * p1[0] - p2[0] * p2[0] + p1[1] * p1[1] - p2[1] * p2[1]) - C[1] * (p1[1] - p2[1])) / \
         (p1[0] - p2[0])
    r = (p1[0]-C[0])*(p1[0]-C[0]) + (p1[1]-C[1])*(p1[1]-C[1])
    return C, r


def traceC(r):
    line = 2 * pi * r
    s = 0.01
    count = ceil(line / s)
    return count


def check(C, r, phi):
    cor = True
    r_min = .495 * .495
    r_max = 1.865 * 1.865

    while phi < 2 * pi:
        R = pow(C[0] + r * c(phi), 2) + pow(C[1] + r * s(phi),2)
        if R < r_min or R > r_max:
            cor = False
            break
        phi += phi

    return cor


def go(cur_pos):
    z = 0.4485
    p1 = np.array([0, .6, z])
    p2 = np.array([.550, 1, z])
    p3 = np.array([0, 1.7, z])

    C, r = calc(p1, p2, p3)
    count = traceC(r)
    phi = 2 * pi / count

    if check(C, r, phi):
        tar = np.array([C[0] + r, C[1], z])
        moveL(cur_pos, tar)
        q = getJointPos()

        while True:
            for i in range(1, count):
                tar[0] = C[0] + r * c(phi * i)
                tar[1] = C[1] + r * s(phi * i)
                q = OZK(q, tar)
                move(q)
                i += 1
    else:
        print(f'error of reachability')


def main():
    cur_pos = goHome()
    go(cur_pos)


main()