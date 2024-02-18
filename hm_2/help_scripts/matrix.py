from math import sin as s, cos as c, pi as pi, ceil as ceil
import numpy as np

def Mat_A(d, a, thetta, alpha):
    return np.array([[c(thetta), -c(alpha)*s(thetta), s(alpha)*s(thetta), a*c(thetta)],
                     [s(thetta), c(alpha)*c(thetta), -s(alpha)*c(thetta), a*s(thetta)],
                     [0, s(alpha), c(alpha), d],
                     [0, 0, 0, 1]])

def A1(q):
    return Mat_A(0.4485, 0.140, pi/2 + q, pi/2)

def A2(q):
    return Mat_A(0, 0.640, pi/2 + q, 0)

def A3(q):
    return Mat_A(0, 0.160, q, pi/2)

def A4(q):
    return Mat_A(1.078, 0, pi + q, pi/2)

def A5(q):
    return Mat_A(0, 0.101, q, 0)


def T(q):
    T0 = np.array([[1., .0, .0, .0],
                   [.0, 1., .0, .0],
                   [.0, .0, 1., .0],
                   [.0, .0, .0, 1.]])
    T1 = A1(q[0])
    T2 = T1.dot(A2(q[1]))
    T3 = T2.dot(A3(q[2]))
    T4 = T3.dot(A4(q[3]))
    T = T4.dot(A5(q[4]))

    return [T0, T1, T2, T3, T4, T]