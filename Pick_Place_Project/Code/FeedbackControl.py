from asyncio import FastChildWatcher
from matplotlib.pyplot import cla
import core as mr
import numpy as np
import math
import itertools 

from KinematicsConfiguration import *
from KinematicSimulator import *
from TrajectoryGenerator import *


class Control:

    Xerr_integral = np.array([0,0,0,0,0,0])


    def FeedbackControl(self, robot_configuration, Xd, Xd_next, feed_forward_enabled = 1):

        (pi,x,y) = robot_configuration[0:3]
        Tsb = kinematic_configuration.configTomatrix(pi,x,y)
        X = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8])


        Xerr =  mr.se3ToVec(mr.MatrixLog6(np.matmul(mr.TransInv(X), Xd)))

        self.Xerr_integral = self.Xerr_integral+Xerr*FeedbackControl_Parameters.delta_t



        ff_ref_twist = mr.se3ToVec((1/FeedbackControl_Parameters.delta_t) * \
                            mr.MatrixLog6(np.matmul(mr.TransInv(Xd), Xd_next)))
    

        ff_part = np.matmul(mr.Adjoint(np.matmul(np.linalg.inv(X), Xd)), ff_ref_twist)
        proportional_part = np.matmul(FeedbackControl_Parameters.kp_matrix , Xerr)
        integral_part = np.matmul(FeedbackControl_Parameters.ki_matrix , self.Xerr_integral)


        end_Effector_twist = (feed_forward_enabled * ff_part) + proportional_part + integral_part

        Teb = np.matmul(mr.TransInv(X), Tsb)
        Jbase = np.matmul(mr.Adjoint(Teb), FeedbackControl_Parameters.F6)
        Jarm  = mr.JacobianBody(kinematic_configuration.Blist, robot_configuration[3:8])

        while(True):
            Je = np.concatenate((Jbase, Jarm), axis=1)
            control = np.matmul(np.linalg.pinv(Je,1e-3), end_Effector_twist)

            thetadot = control[4:]
            joint_angles = robot_configuration[3:8]

            IsOutOfLimit = False
            joint_pos = None
            for i in range(5):

                if thetadot[i] < -1*Simulation_Parameters.joint_Speed_limit:
                    thetadot[i] = -1*Simulation_Parameters.joint_Speed_limit
                elif thetadot[i] > Simulation_Parameters.joint_Speed_limit:
                    thetadot[i] = Simulation_Parameters.joint_Speed_limit

                joint_pos = joint_angles[i] + (thetadot[i] * Simulation_Parameters.TimeStep)

                if(joint_pos < Simulation_Parameters.joint_limits[i][0] or joint_pos > Simulation_Parameters.joint_limits[i][1]):
                    Jarm[:,i] = 0.0
                    IsOutOfLimit = True
            
            if not IsOutOfLimit:
                break
            


        # print("X : ", X)
        # print("Xd : ", Xd)
        # print("Xerr : ", Xerr)
        # print("Vd : ", ff_ref_twist)
        # print("[Ad(X-Xd)]Vd : ", ff_part)
        # print("V : ", end_Effector_twist)
        # print("Je : \n", Je)
        # print("control : ",control)
        return control, Xerr




if __name__ == "__main__":
    robot_configuration = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0]


    #Test 1
    Xd = np.array([
        [0,0,1,0.5],
        [0,1,0,0],
        [-1,0,0,0.5],
        [0,0,0,1]
    ])

    Xd_next = np.array([
        [0,0,1,0.6],
        [0,1,0,0],
        [-1,0,0,0.3],
        [0,0,0,1]
    ])
    FeedbackControl_Parameters.ki_matrix = 0 * np.identity(6)
    FeedbackControl_Parameters.kp_matrix = 0 * np.identity(6)
    control  = Control()
    control.FeedbackControl(robot_configuration, Xd, Xd_next)






    #Test2 Feed forward test
    FeedbackControl_Parameters.ki_matrix = 0 * np.identity(6)
    FeedbackControl_Parameters.kp_matrix = 0 * np.identity(6)
    control  = Control()

    Tse = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8])


    Trajectory_parameters.estimateTrajectoryTimes(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)

    Full_path, GripperState =  TrajectoryGenerator(Tse, Cube_Configuration.Tsc_start, Cube_Configuration.Tsc_goal, \
        Trajectory_parameters.Tce_grasp, Trajectory_parameters.Tce_standoff)
    

    Full_path = np.concatenate(Full_path, axis=0)
    GripperState = np.concatenate(GripperState, axis=0).tolist()

    robot_configuration_list = []

    flatten = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8]).flatten()
    formated = np.concatenate((flatten[0:3],flatten[4:7],flatten[8:11], np.array([flatten[3], flatten[7], flatten[11]])))
    np.append(formated, 0)
    robot_configuration_list.append(formated)


    #Control loop
    for i in range(len(Full_path)-1):
        control_vector, Xerr = control.FeedbackControl(robot_configuration, Full_path[i], Full_path[i+1])
        control_vector = np.concatenate((control_vector[4:9],control_vector[0:4]))

        robot_configuration = NextState(robot_configuration, control_vector, Simulation_Parameters.TimeStep,\
            Simulation_Parameters.joint_Speed_limit, Simulation_Parameters.wheel_Speed_limit)

        flatten = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8]).flatten()
        formated = np.concatenate((flatten[0:3],flatten[4:7],flatten[8:11], np.array([flatten[3], flatten[7], flatten[11]])))
        np.append(formated, GripperState[i])

        robot_configuration_list.append(formated)


    with open("Feedforward_control_trajectory_test.csv", 'w') as f:  
        for i in range(len(robot_configuration_list)):
            str_arr = np.array2string(robot_configuration_list[i], precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+"\n")