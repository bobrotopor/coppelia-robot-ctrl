import sys
import sim as sim
import time
import numpy as np
from math import *
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

deg = pi / 180

# d = [89, 0, 0, 109, 94, 82]
# alpha = np.array([90,0,0,90,-90,0])*deg
# a = [0, -42, -392, 0, 0, 0]

joints_DH = [{'a':  0,      'alpha':    90*deg,     'd':    89,     'theta':    0},
             {'a':  -425,   'alpha':    0,          'd':    0,      'theta':    -90*deg},
             {'a':  -392,   'alpha':    0,          'd':    0,      'theta':    0},
             {'a':  0,      'alpha':    90*deg,     'd':    109,    'theta':    -90*deg},
             {'a':  0,      'alpha':    -90*deg,    'd':    94,     'theta':    0},
             {'a':  0,      'alpha':    0,          'd':    82,     'theta':    0}]

def Rx(theta):
    return np.matrix([[ 1,             0,              0],
                      [ 0, np.cos(theta), -np.sin(theta)],
                      [ 0, np.sin(theta),  np.cos(theta)]])
  
def Ry(theta):
    return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                      [ 0            , 1, 0            ],
                      [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
    return np.matrix([[ np.cos(theta), -np.sin(theta), 0],
                      [ np.sin(theta), np.cos(theta) , 0],
                      [ 0            , 0             , 1]])

def XYZ_toPose(x, y, z, phi, theta, psi):
    pose = np.eye(4)
    P = np.matrix([x, y, z])
    R = Rz(psi) * Ry(theta) * Rx(phi)
    pose[:3, :3] = R
    pose[0, 3] = x
    pose[1, 3] = y
    pose[2, 3] = z
    return pose

def comp_trans_matrix(DH_dict):
    theta = DH_dict['theta']
    alpha = DH_dict['alpha']
    d = DH_dict['d']
    a = DH_dict['a']
    # Compute individual elements of transformation matrix
    T = np.array([[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta)],
                  [sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta)],
                  [     0,          sin(alpha),              cos(alpha),            d     ],
                  [     0,               0,                      0,                 1     ]])
    return T

def error_pos(current_pose, desired_pose):
    # Extract position and rotation from current_pose
    current_position = current_pose[:3, 3]
    current_rotation = R.from_matrix(current_pose[:3, :3])

    # Extract position and rotation from desired_pose
    desired_position = desired_pose[:3, 3]
    desired_rotation = R.from_matrix(desired_pose[:3, :3])

    # Compute position error
    position_error = desired_position - current_position

    # Compute rotation error using Euler angles
    rotation_error = desired_rotation.inv() * current_rotation
    rotation_error = rotation_error.as_euler('xyz')

    # Combine position and rotation error into a single vector
    error = np.hstack((position_error, rotation_error))
    return error

class Manipulator:
    # Initialize robot arm by DH and joints' coordinates
    def __init__(self, joints_DH, sim_joints):
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
        result = minimize(err_cb, self.joints_val, method='BFGS', options={'eps':10e-7})
        return result.x


def Test(clientID, Q):
    err1, j1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    err2, j2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    err3, j3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    err4, j4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    err5, j5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    err6, j6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)

    sim.simxSetJointTargetPosition(clientID, j1, Q[0], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j2, Q[1], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j3, Q[2], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j4, Q[3], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j5, Q[4], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j6, Q[5], sim.simx_opmode_streaming)

def ManipMove(clientID, robot, Q):
    (j1,j2,j3,j4,j5,j6) = robot.sim_joints
    sim.simxSetJointTargetPosition(clientID, j1, Q[0], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j2, Q[1], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j3, Q[2], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j4, Q[3], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j5, Q[4], sim.simx_opmode_streaming)
    sim.simxSetJointTargetPosition(clientID, j6, Q[5], sim.simx_opmode_streaming)

class Point:
    def __init__(self,x,y,z):
        self.x=x
        self.y=y
        self.z=z
    def getData(self):
        return (self.x,self.y,self.z)

class Circle:
    def __init__(self,xc,yc,zc,r):
        self.xc = xc
        self.yc = yc
        self.zc = zc
        self.r = r 

def find_circle(points): #y=const
   A = Point(*points[0])
   B = Point(*points[1])
   C = Point(*points[2])

   k1 = (B.z-A.z)/(B.x-A.x)
   k2 = (C.z-B.z)/(C.x-B.x)
   xc = (k1*k2*(A.z-C.z)+k2*(A.x+B.x)-k1*(B.x+C.x))/(2*(k2-k1))
   yc = A.y
   zc = -(xc-(A.x+B.x)/2)/k1+(A.z+B.z)/2
   r = sqrt((A.x-xc)**2+(A.z-zc)**2)
   return Circle(xc,yc,zc,r)

def generate_poses(Circ,step_count):
    delta = 2*pi / step_count
    curr_point = Point(0,0,0)
    curr_point.y = Circ.yc
    pos_arr = []

    for i in range(step_count):
        curr_point.x = Circ.xc+Circ.r*cos(delta*i)
        curr_point.z = Circ.zc+Circ.r*sin(delta*i)
        pos_arr.append(curr_point.getData())
    
    return [pos + (90*deg,0*deg,0*deg) for pos in pos_arr]
    #[90deg,0,0] - Euler angles

def UR5_initial(clientID):
    err1, j1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    err2, j2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    err3, j3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    err4, j4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    err5, j5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    err6, j6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)
    return (j1,j2,j3,j4,j5,j6)

def main():
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

    print(clientID)

    if clientID != -1:  # check if client connection successful
        print('Connected to remote API server')
    else:
        print('Connection not successful')
        sys.exit('Could not connect')

    UR5_j = UR5_initial(clientID);
    robot = Manipulator(joints_DH, UR5_j)

    A = Point(-150, -250, 600)
    B = Point(0, -250, 750)
    C = Point(150, -250, 600)
    points = (A.getData(), B.getData(), C.getData())
    if (A.y != B.y) or (A.y != C.y) or (C.y != B.y):
        print('Not allowed position (some points are not on the frontal plane)')
        sys.exit()
    d1 = sqrt((A.x - B.x)**2 + (A.z - B.z)**2)
    d2 = sqrt((A.x - C.x) ** 2 + (A.z - C.z) ** 2)
    d3 = sqrt((B.x - C.x) ** 2 + (B.z - C.z) ** 2)
    if (d1 + d2) == d3 or (d1 + d3) == d2 or (d2 + d3) == d1:
        print('Points are located on the same line')
        sys.exit()
    Circ = find_circle(points)
    posesXYZ = generate_poses(Circ,40) # 50 - step-count
    
    Q_preset=[]
    for pos in posesXYZ:
        print(pos)
        target_mat = XYZ_toPose(*pos)
        Q = robot.solveIK(target_mat)
        Q_preset.append(Q)
        print('+', end='')

    print(robot.forward_kinematics((30*deg,30*deg,30*deg,30*deg,30*deg,30*deg)))

    while(1):
        for Q in Q_preset:
            ManipMove(clientID,robot,Q)
            time.sleep(0.03)

main()
