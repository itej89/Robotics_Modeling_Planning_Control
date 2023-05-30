from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 
import os

from KinematicsConfiguration import *
from KinematicSimulator import *
from TrajectoryGenerator import *
from FeedbackControl import *


def Perform_Pick_and_Place(TaskName, Tse, robot_configuration, Kp, Ki,  feed_forward_enabled = 1):

    task_directory = os.path.join(os.path.dirname(os.path.abspath(__file__)), TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)

    FeedbackControl_Parameters.kp_matrix = Kp * np.identity(6)
    FeedbackControl_Parameters.ki_matrix = Ki * np.identity(6)

    # Trajectory_parameters.estimateTrajectoryTimes(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
    #     Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)


    Full_path, GripperState =  TrajectoryGenerator(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)
    
    WriteTrajectoryToCSV(os.path.join(task_directory,"trajectory.csv"),Full_path, GripperState)


    Full_path = np.concatenate(Full_path, axis=0)
    GripperState = np.concatenate(GripperState, axis=0).tolist()

    robot_configuration_list = []
    Xerr_list = []
    for i in range(len(Full_path)-1):
        control_vector, Xerr = control.FeedbackControl(robot_configuration, Full_path[i], Full_path[i+1], feed_forward_enabled)
        control_vector = np.concatenate((control_vector[4:9],control_vector[0:4]))

        robot_configuration = NextState(robot_configuration, control_vector, Simulation_Parameters.TimeStep,\
            Simulation_Parameters.joint_Speed_limit, Simulation_Parameters.wheel_Speed_limit)
        
        robot_configuration_list.append(robot_configuration)
        Xerr_list.append(Xerr)

    
    with open(os.path.join(task_directory,"error.csv"), 'w') as f:  
        for i in range(len(Xerr_list)):
            str_arr = np.array2string(np.array(Xerr_list[i]), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+"\n")

    with open(os.path.join(task_directory,"CoppeliaSim.csv"), 'w') as f:  
        for i in range(len(robot_configuration_list)):
            str_arr = np.array2string(np.array(robot_configuration_list[i]), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+","+str(GripperState[i])+"\n")
          

if __name__ == "__main__":

    #Test2 Feed forward test
    control  = Control()
    Tse = np.array([
        [0,0,1,0],
        [0,1,0,0],
        [-1,0,0,0.5],
        [0,0,0,1]
    ])

    robot_configuration = [0.485,0,0.25,0,0,0,0,0,0,0,0,0]

    # Perform_Pick_and_Place("best",Tse, robot_configuration, Kp=1.7, Ki=0.02)
    # Perform_Pick_and_Place("overshoot",Tse, robot_configuration, Kp=1.48, Ki=0.15,  feed_forward_enabled = 0)

    Cube_Configuration.Tsc_start =  kinematic_configuration.configTomatrix(1.570,0,1)
    Cube_Configuration.Tsc_start[2][3] = 0.025
    Cube_Configuration.Tsc_goal =  kinematic_configuration.configTomatrix(-1.570,-1,-2)
    Cube_Configuration.Tsc_goal[2][3] = 0.025

    Perform_Pick_and_Place("newtask",Tse, robot_configuration, Kp=1.7, Ki=0.02)


