import os
import sys
from pathlib import Path

from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 

from KinematicsConfiguration import *

#Type to store A single Trajectory information
class Trajectory:
    Path = None

    
    def __init__(self,start,end,T,step,method,IsGrasping = 0) -> None:
        """#Constructor takes initilizes following parameters describing the trajectory
            #start : A4x4 matrix describing the start location of the trajectory
            #end : A4x4 matrix describing the end location of the trajectory
            #T : Scalar describing the total time of the trajectory
            #step : A scalar describingt he time step in the trajectory
            #mathod : time scaling method 3 for cubuc or 5 for quintic
            #IsGrasping : 0 or 1 indicating of the robot gripper state for the entire trajectory
        """
        self.start = start
        self.end = end
        self.T = T
        self.step = step
        self.method = method
        self.IsGrasping = IsGrasping

    @classmethod
    
    def from_path(cls, Path,IsGrasping = 0) -> None:
        """Method directly initialized the path give the trajectory array
            #param path : list of 4x4 matrices describing the trajectory
            #IsGrasping : gripper state
        """

        trj = cls(None,None,0,0,0,0)
        trj.Path = Path
        trj.IsGrasping = IsGrasping
        return trj

    def BuildTrajectory(self):
        """Mthod build trajectory from the class parameters
        """
        if self.step > 0:
            self.Path = mr.ScrewTrajectory(self.start, self.end, self.T,self.T/self.step,self.method)


#Generate Complete Task Trajectory
def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff):
    """Performs Feed back control and returns Control velocities and Xerr

    :param Tse_initial: A 4x4 matrix containing the start point of the trajectory in {S} frame
    :param Tsc_initial: A 4x4 matrix containing starting location of the cube in {S} frame
    :param Tsc_final:   A 4x4 matrix containing final location of the cube in {S} frame
    :param Tce_grasp:   A 4x4 matrix containing the end effector orientaion while grasping the cube in cube-frame
    :param Tce_standoff: A 4x4 matrix containing the end effector orientaion while stand off above the cube in cube-frame

    :return: 
        Full_Path : An array containing 8 subarrays of Trasformation matrices where each array describs a sub trajectory
        GripperState : An array containing 8 subarrays dwscribing the gripper state with respect to the Trasformation matrix in the Full path

    Example Input:
                robot_configuration = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0]    
                Tse_initial = np.array([[0.0, 0.0, 1.0, 0.0], 
                                        [0.0, 1.0, 0.0, 0.0], 
                                        [-1.0, 0.0, 0.0, 0.5], 
                                        [0.0, 0.0, 0.0, 1.0]]) 
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
                Tce grasp:  np.array([
                                        [-0.7071067811865475, 0.0, 0.7071067811865476, 0.0],
                                        [0.0, 1.0, 0.0, 0.0], 
                                        [-0.7071067811865476, 0.0, -0.7071067811865475, 0.0], 
                                        [0.0, 0.0, 0.0, 1.0]])
                Tce standoff:  np.array([
                                        [-0.7071067811865475, 0.0, 0.7071067811865476, 0.0], 
                                        [0.0, 1.0, 0.0, 0.0], 
                                        [-0.7071067811865476, 0.0, -0.7071067811865475, 0.1], 
                                        [0.0, 0.0, 0.0, 1.0]])

    Output:      
            Full_Path:   
                    [[array([[ 0.16996714,  0.        ,  0.98544973,  0.3868135 ],
                    [ 0.        ,  1.        ,  0.        ,  0.        ],
                    [-0.98544973,  0.        ,  0.16996714,  0.57019384],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]), array([[ 0.16996431,  0.        ,  0.98545022,  0.38681584],
                    [ 0.        ,  1.        ,  0.        ,  0.        ],
                    [-0.98545022,  0.        ,  0.16996431,  0.57019349],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]]), array([[ 0.16995583,  0.        ,  0.98545168,  0.38682285],
                    [ 0.        ,  1.        ,  0.        ,  0.        ],
                    [-0.98545168,  0.        ,  0.16995583,  0.57019243],
                    [ 0.        ,  0.        ,  0.        ,  1.        ]])]....
            GripperState :  [[0, 0, 0] .......
        """


    #Compute required Trajectory end points in {S} frame
    Tse_standoff_above_cube = np.matmul(Tsc_initial, Tce_standoff)
    Tse_at_cube = np.matmul(Tsc_initial, Tce_grasp)
    Tse_standoff_above_goal = np.matmul(Tsc_final, Tce_standoff)
    Tse_at_goal =  np.matmul(Tsc_final, Tce_grasp)


    #A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block. 
    Trajectory1 = Trajectory(Tse_initial, Tse_standoff_above_cube,Trajectory_parameters.Trajectory1_time,Trajectory_parameters.Trajecotry_resolution,3)
    #A trajectory to move the gripper down to the grasp position. 
    Trajectory2 = Trajectory(Tse_standoff_above_cube, Tse_at_cube,Trajectory_parameters.Trajectory2_time,Trajectory_parameters.Trajecotry_resolution,3)

    #Closing of the gripper.
    Trajectory3 = []
    for i in range(math.ceil(Trajectory_parameters.Grap_Time/Trajectory_parameters.Trajecotry_resolution)*1):
        Trajectory3.append(Tse_at_cube)
    Trajectory3 = Trajectory.from_path(np.array(Trajectory3), IsGrasping=1)


    #A trajectory to move the gripper back up to the "standoff" configuration. 
    Trajectory4 = Trajectory(Tse_at_cube, Tse_standoff_above_cube,Trajectory_parameters.Trajectory4_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)
    #A trajectory to move the gripper to a "standoff" configuration above the final configuration. 
    Trajectory5 = Trajectory(Tse_standoff_above_cube, Tse_standoff_above_goal,Trajectory_parameters.Trajectory5_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)
    #A trajectory to move the gripper to the final configuration of the object. 
    Trajectory6 = Trajectory(Tse_standoff_above_goal, Tse_at_goal,Trajectory_parameters.Trajectory6_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)

    #Opening of the gripper. 
    Trajectory7 = []
    for i in range(math.ceil(Trajectory_parameters.Grap_Time/Trajectory_parameters.Trajecotry_resolution)*1):
        Trajectory7.append(Tse_at_goal)
    Trajectory7 = Trajectory.from_path(np.array(Trajectory7), IsGrasping=0)


    #A trajectory to move the gripper back to the "standoff" configuration.
    Trajectory8 = Trajectory(Tse_at_goal, Tse_standoff_above_goal,Trajectory_parameters.Trajectory8_time,Trajectory_parameters.Trajecotry_resolution,3)

    Full_Trajectory = [Trajectory1, Trajectory2, Trajectory3, \
                        Trajectory4, Trajectory5, Trajectory6,\
                        Trajectory7, Trajectory8]


    Full_Path = []
    GripperState = []

    #Build Path from Trajectory specification
    for trajectory in Full_Trajectory:
        trajectory.BuildTrajectory()
        Full_Path.append(trajectory.Path)
        trj_GripperState = [trajectory.IsGrasping]*len(trajectory.Path)
        GripperState.append(trj_GripperState)

    return np.array(Full_Path), np.array(GripperState)


def WriteTrajectoryToCSV(file_name,Full_Path, GripperState):
    """Performs Feed back control and returns Control velocities and Xerr

    :param file_name: absolute path of a csv file to save the trajectory information
    :param Full_Path : An array containing 8 subarrays of Trasformation matrices where each array describs a sub trajectory
    :param GripperState : An array containing 8 subarrays dwscribing the gripper state with respect to the Trasformation matrix in the Full path

    :return: 
    saves the Full_Path and respective GripperState into a csv file saved at file_name
    """

    with open(file_name, 'w') as f: 
        Full_path_concat = np.concatenate(Full_Path, axis=0)
        GripperState_concat = np.concatenate(GripperState, axis=0).tolist() 

        for i in range(len(Full_path_concat)):
            flatten = Full_path_concat[i].flatten()
            formated = np.concatenate((flatten[0:3],flatten[4:7],flatten[8:11], np.array([flatten[3], flatten[7], flatten[11]])))
            str_arr = np.array2string(formated, precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+","+str(GripperState_concat[i])+"\n")



#Debug and verification code
if __name__ == "__main__":

    print("Generating Milestone2 validation trajectory file in \"../results\milestone_tests\Feedforward_control_trajectory_test.csv for scene 8\" ")
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
    

    robot_configuration = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0]
    Tse = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8])
    
    Full_Path, GripperState = TrajectoryGenerator(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)


    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,"milestone_tests")
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)
    WriteTrajectoryToCSV(os.path.join(task_directory,"trajectory_test.csv"),Full_Path, GripperState)
    print("Done.")