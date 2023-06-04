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

    #Parameter to store integral error
    Xerr_integral = np.array([0,0,0,0,0,0])


    def FeedbackControl(self, robot_configuration, Xd, Xd_next, feed_forward_enabled = 1):
        """Performs Feed back control and returns Control velocities and Xerr

            :param robot_configuration: A 6-vector containing the robot configuraiton in the format 
            :param Xd: A 4x4 matrix containing desired end effector configuration from trajectory at current timestep
            :param Xd: A 4x4 matrix containing desired end effector configuration from trajectory at next timestep
            :param feed_forward_enabled: Flag to inform if feed-forward part need to be added

            :return: 
                control velocities: 9-vector (5 Joint velocities and 4 wheel velocities)
                Xerr : Configuration error between robot configuraion X and desired configuration Xd at current timestep

            Example Input:
                        robot_configuration = [0,0,0,0,0,0.2,-1.6,0,0,0,0,0]    
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
                        feed_forward_enabled = 1
            Output:
                control velocities : [ 1.57169600e+02  1.57169600e+02  1.57169600e+02  1.57169600e+02
                                       9.98475972e-14 -1.00000000e+01  1.00000000e+01 -1.00000000e+01
                                       1.14980366e-15] 
                Xerr : [0.         0.17079633 0.         0.07968904 0.         0.10691679]
                """

        #Calculate X from Robot configuration
        (pi,x,y) = robot_configuration[0:3]
        Tsb = kinematic_configuration.configTomatrix(pi,x,y)
        X = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8])

        #Compute Xerr
        Xerr =  mr.se3ToVec(mr.MatrixLog6(np.matmul(mr.TransInv(X), Xd)))

        #Update Integral error
        self.Xerr_integral = self.Xerr_integral+Xerr*FeedbackControl_Parameters.delta_t


        #Estimate desired twist to the next state
        ff_ref_twist = mr.se3ToVec((1/FeedbackControl_Parameters.delta_t) * \
                            mr.MatrixLog6(np.matmul(mr.TransInv(Xd), Xd_next)))
    
        #Compute feed forward part
        ff_part = np.matmul(mr.Adjoint(np.matmul(np.linalg.inv(X), Xd)), ff_ref_twist)
        #Compute proportional part
        proportional_part = np.matmul(FeedbackControl_Parameters.kp_matrix , Xerr)
        #Compute Integral part
        integral_part = np.matmul(FeedbackControl_Parameters.ki_matrix , self.Xerr_integral)

        #Estimate required twist from current configuration
        end_Effector_twist = (feed_forward_enabled * ff_part) + proportional_part + integral_part

        #convert Twist to robot control velocities
        Teb = np.matmul(mr.TransInv(X), Tsb)
        Jbase = np.matmul(mr.Adjoint(Teb), FeedbackControl_Parameters.F6)
        Jarm  = mr.JacobianBody(kinematic_configuration.Blist, robot_configuration[3:8])

        #-------------------------------------------------------
        #Code to verify and avoid joint angle limit situations
        #-------------------------------------------------------
        MAX_RE_CALCUALTIONS = 1000
        retry_count = 0
        IsOutOfLimit = False
        while(retry_count <= MAX_RE_CALCUALTIONS):
            retry_count += 1

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
            
        #WORST CASE SCENARIO - CONTROL FAILED
        if IsOutOfLimit:
            control = [0]*9
        #-------------------------------------------------------

        return control, Xerr



#Debug and verification code
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


    
    print("Generating Milestone3 fedforward trajectory tracking file in \"../results\milestone_tests\trajectory_test.csv for scene 8\" ")
    #Test2 Feed forward trajectory estimation test
    FeedbackControl_Parameters.ki_matrix = 0 * np.identity(6)
    FeedbackControl_Parameters.kp_matrix = 0 * np.identity(6)
    control  = Control()

    Tse = kinematic_configuration.Tse(robot_configuration[0],robot_configuration[1],robot_configuration[2], robot_configuration[3:8])

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



    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,"milestone_tests")
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)
    with open(os.path.join(task_directory,"Feedforward_control_trajectory_test.csv"), 'w') as f:  
        for i in range(len(robot_configuration_list)):
            str_arr = np.array2string(robot_configuration_list[i], precision=None, suppress_small=True, separator=',').strip('[').strip(']').replace("\n", "").replace(" ", "")
            f.write(str_arr+"\n")
    

    print("Done.")