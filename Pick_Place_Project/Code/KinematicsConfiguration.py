from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math

pi = math.pi

class chasis_params:
    r = 0.0475
    l = 0.47/2
    w = 0.3/2


class kinematic_configuration:

    Tb0 = np.array([
        [1,0,0,0.1662],
        [0,1,0,0],
        [0,0,1,0.0026],
        [0,0,0,1]
    ])

    M0e = np.array([
        [1,0,0,0.033],
        [0,1,0,0],
        [0,0,1,0.6546],
        [0,0,0,1]
    ])


    Blist = np.array([
        [0,0,1,0,0.033,0],
        [0,-1,0,-0.5076,0,0],
        [0,-1,0,-0.3526,0,0],
        [0,-1,0,-0.2176,0,0],
        [0,0,1,0,0,0]
    ]).T

    @staticmethod  
    def Tse(pi, x, y,theta):
        Tsb = kinematic_configuration.configTomatrix(pi,x,y)
        Ts0 = np.matmul(Tsb, kinematic_configuration.Tb0)
        T0e = mr.FKinBody(kinematic_configuration.M0e, kinematic_configuration.Blist, theta)
        return np.matmul(Ts0, T0e)

    @staticmethod  
    def configTomatrix(pi, x, y):
        return np.array([
            [math.cos(pi), -1*math.sin(pi), 0, x     ],
            [math.sin(pi),    math.cos(pi), 0, y     ],
            [0,               0,            1, 0.0963],
            [0,               0,            0, 1]
        ])

  


class Cube_Configuration:
    Tsc_start = np.array([
        [1,0,0,1],
        [0,1,0,0],
        [0,0,1,0.025],
        [0,0,0,1]
    ])


    Tsc_goal = np.array([
        [0,1,0,0],
        [-1,0,0,-1],
        [0,0,1,0.025],
        [0,0,0,1]
    ])

class Simulation_Parameters:
    TimeStep = 0.01
    joint_Speed_limit = 10
    wheel_Speed_limit = 30
    joint_limits = [[-1,1],[-1.9,1.2],[-1.9,1.5],[-2.89,2.89],[-2.89,2.89]]

class Trajectory_parameters:


    Trajectory1_time = 10
    Trajectory2_time = 3
    Trajectory4_time = 3
    Trajectory5_time = 10
    Trajectory6_time = 3
    Trajectory8_time = 3

    Trajecotry_step = 0.01
    K_Param = 1
    Trajecotry_resolution = Trajecotry_step/K_Param
    Grap_Time = 0.625

    max_lin_vel = 0.1
    max_ang_vel = 0.1

    #Example Tce_grasp
    wy = [0,3*pi/4,0]  # rotation about Y-axis by 135 degree
    Ry = mr.MatrixExp3(mr.VecToso3(wy)) # rotation matrix
    P = [0,0,0.0]      #align position to cube frame
    Tce_grasp= mr.RpToTrans(Ry, P)

    #Example Tce_standoff
    wy = [0,3*pi/4,0]  # rotation about Y-axis by 135 degree
    Ry = mr.MatrixExp3(mr.VecToso3(wy)) # rotation matrix
    P = [0,0,0.1]      #align position 0.1m above cube frame
    Tce_standoff= mr.RpToTrans(Ry, P)

    @staticmethod
    def euler_angles_from_rotation_matrix(R):
        '''
        Converts rotation matrix to euler angles
        Input :
            R : Rotation matrix
        
        Return :
            alpha, beta and gamma
        '''

        def isclose(x, y, rtol=1.e-5, atol=1.e-8):
            return abs(x-y) <= atol + rtol * abs(y)

        phi = 0.0
        if isclose(R[2,0],-1.0):
            theta = math.pi/2.0
            psi = math.atan2(R[0,1],R[0,2])
        elif isclose(R[2,0],1.0):
            theta = -math.pi/2.0
            psi = math.atan2(-R[0,1],-R[0,2])
        else:
            theta = -math.asin(R[2,0])
            cos_theta = math.cos(theta)
            psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
            phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
        return psi, theta, phi

    @staticmethod
    def get_duration(T_start, T_end):
        T_dist = np.dot(mr.TransInv(T_start),T_end)
        dist = math.hypot(T_dist[0,-1], T_dist[1,-1])
        ang = max(Trajectory_parameters.euler_angles_from_rotation_matrix(T_dist[:-1,:-1]))
        return max(dist/Trajectory_parameters.max_lin_vel, ang/Trajectory_parameters.max_ang_vel)

    @staticmethod
    def estimateTrajectoryTimes(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff):

        Tsc_standoff_above_cube = np.matmul(Tsc_initial, Tce_standoff)
        Tsc_at_cube = np.matmul(Tsc_initial, Tce_grasp)
        Tsc_standoff_above_goal = np.matmul(Tsc_final, Tce_standoff)
        Tsc_at_goal =  np.matmul(Tsc_final, Tce_grasp)

        Trajectory_parameters.Trajectory1_time = Trajectory_parameters.get_duration(Tse_initial,Tsc_standoff_above_cube)
        Trajectory_parameters.Trajectory2_time = Trajectory_parameters.get_duration(Tsc_standoff_above_cube,Tsc_at_cube)
        Trajectory_parameters.Trajectory4_time = Trajectory_parameters.get_duration(Tsc_at_cube,Tsc_standoff_above_cube)
        Trajectory_parameters.Trajectory5_time = Trajectory_parameters.get_duration(Tsc_standoff_above_cube,Tsc_standoff_above_goal)
        Trajectory_parameters.Trajectory6_time = Trajectory_parameters.get_duration(Tsc_standoff_above_goal,Tsc_at_goal)
        Trajectory_parameters.Trajectory8_time = Trajectory_parameters.get_duration(Tsc_at_goal,Tsc_standoff_above_goal)



class FeedbackControl_Parameters:
    kp_matrix = 0 * np.identity(6)
    ki_matrix = 0 * np.identity(6)
    delta_t = 0.01/Trajectory_parameters.K_Param

    y = 1/(chasis_params.l+chasis_params.w)
    F6 = (chasis_params.r/4)* np.array([
        [0]*4,
        [0]*4,
        [-1*y, y, y, -1*y],
        [1,1,1,1],
        [-1,1,-1,1],
        [0,0,0,0]
    ])