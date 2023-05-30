from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 

from KinematicsConfiguration import *

class Trajectory:
    Path = None
    def __init__(self,start,end,T,step,method,IsGrasping = 0) -> None:
        self.start = start
        self.end = end
        self.T = T
        self.step = step
        self.method = method
        self.IsGrasping = IsGrasping

    @classmethod
    def from_path(cls, Path,IsGrasping = 0) -> None:
        trj = cls(None,None,0,0,0,0)
        trj.Path = Path
        trj.IsGrasping = IsGrasping
        return trj

    def BuildTrajectory(self):
        if self.step > 0:
            self.Path = mr.ScrewTrajectory(self.start, self.end, self.T,self.T/self.step,self.method)


def TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff):
   

    Tsc_standoff_above_cube = np.matmul(Tsc_initial, Tce_standoff)
    Tsc_at_cube = np.matmul(Tsc_initial, Tce_grasp)
    Tsc_standoff_above_goal = np.matmul(Tsc_final, Tce_standoff)
    Tsc_at_goal =  np.matmul(Tsc_final, Tce_grasp)


    Trajectory1 = Trajectory(Tse_initial, Tsc_standoff_above_cube,Trajectory_parameters.Trajectory1_time,Trajectory_parameters.Trajecotry_resolution,3)
    Trajectory2 = Trajectory(Tsc_standoff_above_cube, Tsc_at_cube,Trajectory_parameters.Trajectory2_time,Trajectory_parameters.Trajecotry_resolution,3)


    Trajectory3 = []
    for i in range(math.ceil(Trajectory_parameters.Grap_Time/Trajectory_parameters.Trajecotry_resolution)*1):
        Trajectory3.append(Tsc_at_cube)
    Trajectory3 = Trajectory.from_path(np.array(Trajectory3), IsGrasping=1)



    Trajectory4 = Trajectory(Tsc_at_cube, Tsc_standoff_above_cube,Trajectory_parameters.Trajectory4_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)
    Trajectory5 = Trajectory(Tsc_standoff_above_cube, Tsc_standoff_above_goal,Trajectory_parameters.Trajectory5_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)
    Trajectory6 = Trajectory(Tsc_standoff_above_goal, Tsc_at_goal,Trajectory_parameters.Trajectory6_time,Trajectory_parameters.Trajecotry_resolution,3, IsGrasping=1)


    Trajectory7 = []
    for i in range(math.ceil(Trajectory_parameters.Grap_Time/Trajectory_parameters.Trajecotry_resolution)*1):
        Trajectory7.append(Tsc_at_goal)
    Trajectory7 = Trajectory.from_path(np.array(Trajectory7), IsGrasping=0)



    Trajectory8 = Trajectory(Tsc_at_goal, Tsc_standoff_above_goal,Trajectory_parameters.Trajectory8_time,Trajectory_parameters.Trajecotry_resolution,3)

    Full_Trajectory = [Trajectory1, Trajectory2, Trajectory3, \
                        Trajectory4, Trajectory5, Trajectory6,\
                        Trajectory7, Trajectory8]


    Full_Path = []
    GripperState = []
    for trajectory in Full_Trajectory:
        trajectory.BuildTrajectory()
        Full_Path.append(trajectory.Path)
        trj_GripperState = [trajectory.IsGrasping]*len(trajectory.Path)
        GripperState.append(trj_GripperState)


    return np.array(Full_Path), np.array(GripperState)

def WriteTrajectoryToCSV(file_name,Full_Path, GripperState):
    with open(file_name, 'w') as f: 
        Full_path_concat = np.concatenate(Full_Path, axis=0)
        GripperState_concat = np.concatenate(GripperState, axis=0).tolist() 

        for i in range(len(Full_path_concat)):
            flatten = Full_path_concat[i].flatten()
            formated = np.concatenate((flatten[0:3],flatten[4:7],flatten[8:11], np.array([flatten[3], flatten[7], flatten[11]])))
            str_arr = np.array2string(formated, precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+","+str(GripperState_concat[i])+"\n")

if __name__ == "__main__":

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

    Trajectory_parameters.estimateTrajectoryTimes(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)
    
    Full_Path, GripperState = TrajectoryGenerator(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)

    WriteTrajectoryToCSV("trajectory_test.csv",Full_Path, GripperState)