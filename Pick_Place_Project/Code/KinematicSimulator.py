import os
import sys
from pathlib import Path

from contextlib import suppress
from dataclasses import replace
from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 

from KinematicsConfiguration import *

pi = math.pi


#Function to estimate Mecanum robot odometry
def MecanumOdometry(deltatheta, chasis_config):
    """Performs Odometry calculation of a 4 wheel Mecanum omnidirectional robot
        :param deltatheta: delta theta andles
        :param chasis_config : A 3-array describing the robot chasis configuration in {S} frame - (phi, x, y)
    :return: 
        new_chasis_config : returns updated chasis configuration

    Example Input:
            deltatheta : [-0.09999999999999964, 0.09999999999999964, 0.09999999999999964, -0.09999999999999964]
            chasis_config : [1.1967228857709284, 0.0, 0.0]
        Output:
            new_chasis_config : [1.20906024 0.         0.        ]
    """
    
    #Create F matrix
    Fvar = 1/(chasis_params.l+chasis_params.w)
    F = (chasis_params.r/4)*np.array([
        [-1*Fvar,  Fvar,    Fvar,    -1*Fvar],
        [ 1,       1,       1,        1],
        [-1,       1,      -1,        1]
    ]) 

    #Get body Twist
    Vb = np.matmul(F, deltatheta)
    Vb6 = np.concatenate((np.array([0,0]), Vb, np.array([0])), axis=0)
    
    #Get Body configuration
    Tbbdash = mr.MatrixExp6(mr.VecTose3(Vb6))
    config_vec = mr.se3ToVec(Tbbdash)
    (Wz, Vx, Vy) = config_vec[2:5]


    qbDelta = [0,0,0] 
    if Wz == 0:
        qbDelta = [Wz, Vx, Vy]
    else:
        qbDelta = [Wz, ((Vx*math.sin(Wz)+Vy*(math.cos(Wz)-1))/Wz), ((Vy*math.sin(Wz)+Vx*(1-math.cos(Wz)))/Wz)]

    #Chagne the frame to {S}
    Phi_k = chasis_config[0]
    T = np.array([
        [1, 0,                0],
        [0, math.cos(Phi_k), -1*math.sin(Phi_k)],
        [0, math.sin(Phi_k),    math.cos(Phi_k)]
    ])

    qDelta = np.matmul(T, np.array(qbDelta))

    #Calculate new chasis configuration
    new_chasis_config = chasis_config + qDelta

    return new_chasis_config



def NextState(current_configuration, control_vector, time_step, joint_Speed_limit, wheel_Speed_limit):
    """Performs robot configuration calculation of a 4 wheel Mecanum omnidirectional robot with 5R arm given control velocities
        :param current_configuration: delta theta andles
        :param control_vector : A 9-vector containing the 5 arm joint velocities and 4 wheel velocities
        :param time_step : Simelation time step length in seconds
        :param joint_Speed_limit : An absolute value describing the joint velocity limit of 5R arm in either direction(clockwise and anti-clockwise).
        :param wheel_Speed_limit : An absolute value describing the wheel speed limit in either direction(forward and backward).
        :return:  
            new_chasis_config : returns a 12 vector containing updated chasis configuration+5 joint angles + 5 wheel angles
    
    Example Input:
            current_configuration = [0,0,0,0,0,0,0,0,0,0,0,0]    
            control_vector = [0,0,0,0,0,-10,10,-10,10]
            time_step = 0.01
            joint_Speed_limit = 10
            wheel_Speed_limit = 30
        Output:
            new_chasis_config : [0.012337349337844642, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1, 0.1, 0.1, -0.1]
            """
    #apply velocity limits
    control_vector[0:5]  = [joint_Speed_limit if i > joint_Speed_limit else i for i in control_vector[0:5]]
    control_vector[0:5]  = [joint_Speed_limit if i < -1*joint_Speed_limit else i for i in control_vector[0:5]]
    control_vector[5:9]  = [wheel_Speed_limit if i > wheel_Speed_limit else i for i in control_vector[5:9]]
    control_vector[5:9]  = [wheel_Speed_limit if i < -1* wheel_Speed_limit else i for i in control_vector[5:9]]

    #Extract robot configuration
    chasis_configuration = current_configuration[0:3]
    arm_Configuration = current_configuration[3:8]
    wheel_angles = current_configuration[8:12]

    #Extract control speeds
    joint_Speeds = control_vector[0:5]
    wheel_Speeds = control_vector[5:9]

    #Estimate joint angles
    updated_arm_joint_angles = []
    for (theta, thetadot) in itertools.zip_longest(arm_Configuration, joint_Speeds):
        updated_arm_joint_angles.append(theta+(thetadot*time_step))

    #Estimate wheel angles
    updated_wheel_angles = []
    for (theta, thetadot) in itertools.zip_longest(wheel_angles, wheel_Speeds):
        updated_wheel_angles.append(theta+(thetadot*time_step))


    #Estimate Chasis configuration
    wheels_deltatheta = []
    for (theta_new, theta_old) in itertools.zip_longest(updated_wheel_angles, wheel_angles):
        wheels_deltatheta.append(theta_new-theta_old)
    updated_chasis_Config = MecanumOdometry(wheels_deltatheta, chasis_configuration)

    return updated_chasis_Config.tolist()+updated_arm_joint_angles+updated_wheel_angles







#Debug and verification code
if __name__ == "__main__":
    print("Generating Milestone1 simulation file in \"../results\milestone_tests\Odometry_test.csv for scene 6\" ")
    #Test function to check the NextState function
    def loopkinematics(robot_configuration, control_vector, TimeStep, TotalTime, joint_Speed_limit, wheel_Speed_limit):
        count = int(TotalTime/TimeStep)
        
        task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,"milestone_tests")
        if not os.path.exists(task_directory):
            os.mkdir(task_directory)
        with open(os.path.join(task_directory,"Odometry_test.csv"), 'w') as f:  
            str = np.array2string(np.array(robot_configuration), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "")
            f.write(str+",0"+"\n")
            for i in range(count):
                robot_configuration = NextState(robot_configuration, control_vector, TimeStep, joint_Speed_limit, wheel_Speed_limit)
                str = np.array2string(np.array(robot_configuration), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
                f.write(str+",0"+"\n")

    initial_config = [0,0,0,0,0,0,0,0,0,0,0,0]
    arm_theta_dot = [0,0,0,0,0]
    chasis_u = [-10,10,10,-10]
    joint_speeds = arm_theta_dot+chasis_u
    loopkinematics(initial_config,joint_speeds,Simulation_Parameters.TimeStep,1,Simulation_Parameters.joint_Speed_limit,Simulation_Parameters.wheel_Speed_limit)

    print("Done.")