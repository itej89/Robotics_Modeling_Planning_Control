from contextlib import suppress
from dataclasses import replace
from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 

from KinematicsConfiguration import *

pi = math.pi



def MecanumOdometry(deltatheta, chasis_config):

    Fvar = 1/(chasis_params.l+chasis_params.w)

    F = (chasis_params.r/4)*np.array([
        [-1*Fvar,  Fvar,    Fvar,    -1*Fvar],
        [ 1,       1,       1,        1],
        [-1,       1,      -1,        1]
    ]) 

    Vb = np.matmul(F, deltatheta)
    # print("Vb : ",Vb)
    Vb6 = np.concatenate((np.array([0,0]), Vb, np.array([0])), axis=0)
    # print("Vb6 : ",Vb6)
    Tbbdash = mr.MatrixExp6(mr.VecTose3(Vb6))
    config_vec = mr.se3ToVec(Tbbdash)
    (Wz, Vx, Vy) = config_vec[2:5]

    Phi_k = chasis_config[0]

    qbDelta = [0,0,0] 
    if Wz == 0:
        qbDelta = [Wz, Vx, Vy]
    else:
        qbDelta = [Wz, ((Vx*math.sin(Wz)+Vy*(math.cos(Wz)-1))/Wz), ((Vy*math.sin(Wz)+Vx*(1-math.cos(Wz)))/Wz)]

    T = np.array([
        [1, 0,                0],
        [0, math.cos(Phi_k), -1*math.sin(Phi_k)],
        [0, math.sin(Phi_k),    math.cos(Phi_k)]
    ])

    qDelta = np.matmul(T, np.array(qbDelta))

    new_chasis_config = chasis_config + qDelta

    return new_chasis_config



def NextState(current_configuration, control_vector, time_step, joint_Speed_limit, wheel_Speed_limit):
    
    control_vector[0:5]  = [joint_Speed_limit if i > joint_Speed_limit else i for i in control_vector[0:5]]
    control_vector[0:5]  = [joint_Speed_limit if i < -1*joint_Speed_limit else i for i in control_vector[0:5]]
    control_vector[5:9]  = [wheel_Speed_limit if i > wheel_Speed_limit else i for i in control_vector[5:9]]
    control_vector[5:9]  = [wheel_Speed_limit if i < -1* wheel_Speed_limit else i for i in control_vector[5:9]]


    chasis_configuration = current_configuration[0:3]
    arm_Configuration = current_configuration[3:8]
    wheel_angles = current_configuration[8:12]


    joint_Speeds = control_vector[0:5]
    wheel_Speeds = control_vector[5:9]

    updated_arm_joint_angles = []
    for (theta, thetadot) in itertools.zip_longest(arm_Configuration, joint_Speeds):
        updated_arm_joint_angles.append(theta+(thetadot*time_step))

    updated_wheel_angles = []
    for (theta, thetadot) in itertools.zip_longest(wheel_angles, wheel_Speeds):
        updated_wheel_angles.append(theta+(thetadot*time_step))

    wheels_deltatheta = []
    for (theta_new, theta_old) in itertools.zip_longest(updated_wheel_angles, wheel_angles):
        wheels_deltatheta.append(theta_new-theta_old)

    updated_chasis_Config = MecanumOdometry(wheels_deltatheta, chasis_configuration)

    return updated_chasis_Config.tolist()+updated_arm_joint_angles+updated_wheel_angles





def loopkinematics(robot_configuration, control_vector, TimeStep, TotalTime, joint_Speed_limit, wheel_Speed_limit):

    count = int(TotalTime/TimeStep)

    with open("Odometry_test.csv", 'w') as f:  
        str = np.array2string(np.array(robot_configuration), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "")
        f.write(str+",0"+"\n")
        for i in range(count):
            robot_configuration = NextState(robot_configuration, control_vector, TimeStep, joint_Speed_limit, wheel_Speed_limit)
            str = np.array2string(np.array(robot_configuration), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str+",0"+"\n")



if __name__ == "__main__":
    initial_config = [0,0,0,0,0,0,0,0,0,0,0,0]
    arm_theta_dot = [0,0,0,0,0]
    chasis_u = [-10,10,10,-10]
    joint_speeds = arm_theta_dot+chasis_u
    loopkinematics(initial_config,joint_speeds,Simulation_Parameters.TimeStep,1,Simulation_Parameters.joint_Speed_limit,Simulation_Parameters.wheel_Speed_limit)