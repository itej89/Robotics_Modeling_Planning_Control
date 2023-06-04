from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math

pi = math.pi

#Chasis parameters
class chasis_params:
    r = 0.0475
    l = 0.47/2
    w = 0.3/2

#Given Robot Configuration paramters
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

  

#Cube configuration
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


#Simulation Parameters
class Simulation_Parameters:
    TimeStep = 0.01
    joint_Speed_limit = 10
    wheel_Speed_limit = 30
    joint_limits = [[-2.932,2.932],[-1.117,1.553],[-2.620,2.530],[-1.78,1.78],[-2.89,2.89]]


#Trajectory Parameters
class Trajectory_parameters:

    Trajectory1_time = 10
    Trajectory2_time = 3
    Trajectory4_time = 3
    Trajectory5_time = 10
    Trajectory6_time = 3
    Trajectory8_time = 3

    Trajecotry_step = Simulation_Parameters.TimeStep
    K_Param = 1
    Trajecotry_resolution = Trajecotry_step/K_Param
    Grap_Time = 0.625


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


#Feedback control parameters
class FeedbackControl_Parameters:
    kp_matrix = 0 * np.identity(6)
    ki_matrix = 0 * np.identity(6)
    delta_t = Simulation_Parameters.TimeStep/Trajectory_parameters.K_Param

    y = 1/(chasis_params.l+chasis_params.w)
    F6 = (chasis_params.r/4)* np.array([
        [0]*4,
        [0]*4,
        [-1*y, y, y, -1*y],
        [1,1,1,1],
        [-1,1,-1,1],
        [0,0,0,0]
    ])