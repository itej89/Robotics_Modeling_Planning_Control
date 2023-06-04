
import os
import sys
from pathlib import Path

from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 


from KinematicsConfiguration import *
from KinematicSimulator import *
from TrajectoryGenerator import *
from FeedbackControl import *
from error_plotter import *


def Perform_Pick_and_Place(TaskName, Tse, robot_configuration, Kp, Ki,  feed_forward_enabled = 1):
    """Performs Pick and Place operation

    :param TaskName: A String Containning the Task Name (Decides the sub directory in "results" folder into which results are saved)
    :param Tse: A 4x4 matrix containing the initial configuration of the robot end effector in {S} frame
    :param robot_configuration: A 6-vector containing the robot configuraiton in the format 
            (Φ, x, y, θ1, θ2, θ3, θ4, θ5,, w1, w2, w3, w4) = chasis configuration, 2 arm joint angles, 4wheel angles
    :param Kp: Proportional controller gain
    :param Ki: Integral controller gain
    :param feed_forward_enabled: Informs if feed forward portion needs to be added (enabled by default)

    :return: 
        -Writes genrated Trajectory information to  "../results/{TaskName}/trajectory.csv"
        -Writes Simulation information to  "../results/{TaskName}/CoppeliaSim.csv"
        -Writes Xerror information to  "../results/{TaskName}/error.csv"
        -Writes log information to  "../results/{TaskName}/log.txt" if "PRINT_TO_CONSOLE" is false in MAIN function
        -Writes Xerror plot to  "../results/{TaskName}/error_plot.png"


    Example Input:
                TaskName = "best"

                Tse = np.array([
                [0,0,1,0],
                [0,1,0,0],
                [-1,0,0,0.5],
                [0,0,0,1]
                ])

                robot_configuration = [0.485,0,0.25,0,0,0,0,0,0,0,0,0]

                Kp = 1.7
                Ki = 0.02
    Output:
        -Writes genrated Trajectory information to  "../results/best/trajectory.csv"
        -Writes Simulation information to  "../results/best/CoppeliaSim.csv"
        -Writes Xerror information to  "../results/best/error.csv"
        -Writes log information to  "../results/best/log.txt" if "PRINT_TO_CONSOLE" is false in MAIN function
        -Writes Xerror plot to  "../results/best/error_plot.png"
    """


    #print simulation configuration-------------------------------------
    print("Trajectory start location - Tse : ",Tse.tolist())
    print("Robot Configuration : ",robot_configuration)

    print("\n Simulation Prameters:")
    print("\tTimeStep : ",Simulation_Parameters.TimeStep)
    print("\tJoint Velocity Limits : ",Simulation_Parameters.joint_Speed_limit)
    print("\tWheel Velocity Limits : ",Simulation_Parameters.wheel_Speed_limit)
    print("\tJoint Angle Limits : ",Simulation_Parameters.joint_limits)


    print("\n Trajectory Prameters:")
    print(f"\tTimings in sec: \
    {Trajectory_parameters.Trajectory1_time},\
    {Trajectory_parameters.Trajectory2_time},\
    {Trajectory_parameters.Grap_Time},\
    {Trajectory_parameters.Trajectory4_time},\
    {Trajectory_parameters.Trajectory5_time},\
    {Trajectory_parameters.Trajectory6_time},\
    {Trajectory_parameters.Grap_Time},\
    {Trajectory_parameters.Trajectory8_time}")
    print("\tK multiplier: ",Trajectory_parameters.K_Param)
    print("\tTce grasp: ",Trajectory_parameters.Tce_grasp.tolist())
    print("\tTce standoff: ",Trajectory_parameters.Tce_standoff.tolist())

    print("\n Control Prameters:")
    print("\tfeed forward added : ",feed_forward_enabled)
    print("\tKp : ",Kp)
    print("\tKi : ",Ki)

    print("\n Task paramters:")
    print("\tCube start : ",Cube_Configuration.Tsc_start.tolist())
    print("\tCube goal  : ",Cube_Configuration.Tsc_goal.tolist())
    #-----------------------------------------------------------------------------------




    #Create Task direcoty to save the task data
    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)

    FeedbackControl_Parameters.kp_matrix = Kp * np.identity(6)
    FeedbackControl_Parameters.ki_matrix = Ki * np.identity(6)


    #Generate Trajectory
    print("\nGenerating Trajectory. . .")
    Full_path, GripperState =  TrajectoryGenerator(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)
    
    WriteTrajectoryToCSV(os.path.join(task_directory,"trajectory.csv"),Full_path, GripperState)
    print("Trajectory Created.")


    Full_path = np.concatenate(Full_path, axis=0)
    GripperState = np.concatenate(GripperState, axis=0).tolist()


    #Run control loop
    print("\nControl loop running...")
    robot_configuration_list = []
    Xerr_list = []
    control = Control()
    for i in range(len(Full_path)-1):
        control_vector, Xerr = control.FeedbackControl(robot_configuration, Full_path[i], Full_path[i+1], feed_forward_enabled)
        control_vector = np.concatenate((control_vector[4:9],control_vector[0:4]))

        robot_configuration = NextState(robot_configuration, control_vector, Simulation_Parameters.TimeStep,\
            Simulation_Parameters.joint_Speed_limit, Simulation_Parameters.wheel_Speed_limit)
        
        robot_configuration_list.append(robot_configuration)
        Xerr_list.append(Xerr)
    print("Finished Control loop.")


    #Save Simulation file, error file and error plot
    print("\nSaving Simulation data. . .")
    with open(os.path.join(task_directory,"CoppeliaSim.csv"), 'w') as f:  
        for i in range(len(robot_configuration_list)):
            str_arr = np.array2string(np.array(robot_configuration_list[i]), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+","+str(GripperState[i])+"\n")
    

    with open(os.path.join(task_directory,"error.csv"), 'w') as f:  
        for i in range(len(Xerr_list)):
            str_arr = np.array2string(np.array(Xerr_list[i]), precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+"\n")

    plot_error(os.path.join(task_directory,"error.csv"),TaskName)
    print("Done.")

          

if __name__ == "__main__":


    print("Running Modern Robotics Capstone project. . .")
    #Parameter to control where the console output will be written
    PRINT_TO_CONSOLE  = False

    #Configuration for "best" and "overshoot" tasks
    Tse = np.array([
        [0,0,1,0],
        [0,1,0,0],
        [-1,0,0,0.5],
        [0,0,0,1]
    ])
    robot_configuration = [0.485,0,0.25,0,0,0,0,0,0,0,0,0]


    # ---------------------------------------------------------------------------------------------------------------
    # best configuration Task ----------------------------------------------------------------------------------------
    # ---------------------------------------------------------------------------------------------------------------
    TaskName = "best"
    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)
    if not PRINT_TO_CONSOLE: sys.stdout = open(os.path.join(task_directory,"log.txt"), "w")

    print("---------------------------------------------------")
    print("Best configuration - Simulation")
    print("---------------------------------------------------")
    Perform_Pick_and_Place(TaskName,Tse, robot_configuration, Kp=1.7, Ki=0.02)
    print("---------------------------------------------------")

    if not PRINT_TO_CONSOLE: sys.stdout.close()
    # ---------------------------------------------------------------------------------------------------------------




    # ---------------------------------------------------------------------------------------------------------------
    #overshoot configuration Task -----------------------------------------------------------------------------------
    # ---------------------------------------------------------------------------------------------------------------
    TaskName = "overshoot"
    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)
    if not PRINT_TO_CONSOLE: sys.stdout = open(os.path.join(task_directory,"log.txt"), "w")

    print("\n\n---------------------------------------------------")
    print("Overshoot configuration - Simulation")
    print("---------------------------------------------------")
    Perform_Pick_and_Place(TaskName,Tse, robot_configuration, Kp=1.9, Ki=0.02,  feed_forward_enabled = 0)
    print("---------------------------------------------------")

    if not PRINT_TO_CONSOLE: sys.stdout.close()
    # ---------------------------------------------------------------------------------------------------------------




    # ---------------------------------------------------------------------------------------------------------------
    #newtask configuration Task -------------------------------------------------------------------------------------
    # ---------------------------------------------------------------------------------------------------------------
    TaskName = "newtask"
    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)
    if not PRINT_TO_CONSOLE: sys.stdout = open(os.path.join(task_directory,"log.txt"), "w")

    print("\n\n---------------------------------------------------")
    print("New Task configuration - Simulation")
    print("---------------------------------------------------")

    #Cube configuration for new task
    Cube_Configuration.Tsc_start =  kinematic_configuration.configTomatrix(1.570,0,1)
    Cube_Configuration.Tsc_start[2][3] = 0.025
    Cube_Configuration.Tsc_goal =  kinematic_configuration.configTomatrix(-1.570,-1,-2)
    Cube_Configuration.Tsc_goal[2][3] = 0.025

    Perform_Pick_and_Place(TaskName,Tse, robot_configuration, Kp=1.7, Ki=0.02)
    print("---------------------------------------------------")
    if not PRINT_TO_CONSOLE: sys.stdout.close()
    # ---------------------------------------------------------------------------------------------------------------
    print("Done")


