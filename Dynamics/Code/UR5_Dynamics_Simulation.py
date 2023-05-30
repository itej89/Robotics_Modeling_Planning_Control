import numpy as np
import core as mr


def Simulate_UR5_freefall(Mlist, Glist, Slist, theta, total_time, time_step, iterates_file_name):
    
    #Initialize parameters
    g = [0,0,-9.81]
    Ftip = [0.0]*6
    theta_rate = [0.0]*6
    theta_accel = [0.0]*6
    tau = [0.0]*6


    #Contains list of all estimated joints for simulation
    estimated_joint_angles_list = []
    Time = 0

    while Time < total_time :

        estimated_joint_angles_list.append(theta)

        # Run forward dynamics to calculate acceleration
        theta_accel = mr.ForwardDynamics(theta, theta_rate, tau, g, Ftip, Mlist, Glist, Slist)  

        #Estimate joint velocity and acceleration for the given time step
        theta, theta_rate = mr.EulerStep(theta, theta_rate, theta_accel,0.01)
        Time += time_step
        
        print("\ntheta : ", theta, "\ntheta_rate :", theta_rate, "\ntheta_accel : ",theta_accel)

        
    #Write all estimated join angles in each iteration to a csv file
    if iterates_file_name!="":
        open(iterates_file_name, 'w').close()

        f = open(iterates_file_name, "a")
        for theta in estimated_joint_angles_list:
            f.write(np.array2string(np.array(theta), precision=4, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace(" ", "")+"\n")
        f.close()




M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67]
Slist = [[0,         0,         0,         0,        0,        0],
        [0,         1,         1,         1,        0,        1],
        [1,         0,         0,         0,       -1,        0],
        [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
        [0,         0,         0,         0,  0.81725,        0],
        [0,         0,     0.425,   0.81725,        0,  0.81725]]



#Simulation 1
theta = [0,0,0,0,0,0]
Simulation_Time = 3                # 3 seconds
Time_step = 0.01        # 100 steps per second
Simulate_UR5_freefall(Mlist, Glist, Slist,theta, Simulation_Time, Time_step, "simulation1.csv")

#Simulation 2
theta = [0,-1,0,0,0,0]
Simulation_Time = 5                # 5 seconds
Time_step = 0.01        # 100 steps per second
Simulate_UR5_freefall(Mlist, Glist, Slist, theta , Simulation_Time, Time_step, "simulation2.csv")