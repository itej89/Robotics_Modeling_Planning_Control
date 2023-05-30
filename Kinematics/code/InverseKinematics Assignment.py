from certifi import where
import numpy as np
from numpy import linalg as LA
import core as mr
import math




def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot and prints information for each iteration

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """

    estimated_joint_angles_list = []
    maxiterations = 20
    i = 0
    print("\nIteration ", i,":")

    thetalist = np.array(thetalist0).copy()
    estimated_joint_angles_list.append(thetalist)
    print("\nInitial guess : ", np.array2string(thetalist, precision=3, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ""))


    #Find the end effector orientation for the initial joint angles
    se3_end = mr.FKinBody(M, Blist, thetalist)
    print("\nSE(3) end − effector config :", np.array2string(np.divide(se3_end,1000,where=[[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,0]]), precision=4, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", "  "))


    #Estimate the twist that is still required to tranform from the current end effector orientation to the desired orientation 
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(se3_end), T)))
    print("\nerror twist V_b :",np.array2string(Vb, precision=3, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ""))

    #calulate error from the angular component of twist
    angular_err_mag = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    print("\nangular error magnitude ∣∣omega_b∣∣ :",'{:f}'.format(angular_err_mag))

    #calulate error from the linear component of twist
    linear_err_mag = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    print("\nlinear error magnitude ∣∣v_b∣∣ :",'{:f}'.format(linear_err_mag))

    #evalkuate the error for the given tolerance range
    err =  angular_err_mag > eomg \
          or linear_err_mag > ev


    while err and i < maxiterations:
        i = i + 1
        print("\n\nIteration ", i,":")

        #Add error in the joint angles for the current estimate of end effector twist
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        estimated_joint_angles_list.append(thetalist)
        print("\njoint vector : ", np.array2string(thetalist, precision=3, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ""))


        #Find the end effector orientation for the new joint angles
        se3_end = mr.FKinBody(M, Blist, thetalist)
        print("\nSE(3) end − effector config :", np.array2string(np.divide(se3_end,1000,where=[[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,0]]), precision=4, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", "  "))

        #Estimate the twist that is still required to tranform from the current end effector orientation to the desired orientation 
        Vb \
        = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(se3_end), T)))
        print("\nerror twist V_b :",np.array2string(Vb, precision=3, separator=' ', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ""))

        #calulate error from the angular component of twist
        angular_err_mag = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        print("\nangular error magnitude ∣∣omega_b∣∣ :",'{:f}'.format(angular_err_mag))

        #calulate error from the linear component of twist
        linear_err_mag = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        print("\nlinear error magnitude ∣∣v_b∣∣ :", '{:f}'.format(linear_err_mag))
        

        #evalkuate the error for the given tolerance range
        err =  angular_err_mag > eomg \
            or linear_err_mag > ev
        
       
    #Write all estimated join angles in each iteration to a csv file
    f = open("iterates.csv", "a")
    f.write(np.array2string(np.array(estimated_joint_angles_list), precision=4, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace(" ", ""))
    f.close()
    return (thetalist, not err)






#-------------Code to find inverse kinematics for UR5 robot---------------------

#desired End effector orientation
Tsd = np.array([[0, 1, 0, -500],
                [0, 0, -1, 100],
                [-1,0,  0, 100],
                [0, 0,  0, 1]])

#End effector orientation in UR5 zero position
M = np.array([[-1,0,0,425+392],
                [0,0,1,109+82],
                [0,1,0,89-95],
                [0,0,0,1]])

#Screw axis in end-effector frame
Blist = np.array([[0,1,0,109+82,0,425+392],
                    [0,0,1,95,-(425+392),0],
                    [0,0,1,95,-392,0],
                    [0,0,1,95,0,0],
                    [0,-1,0,-82,0,0],
                    [0,0,1,0,0,0]]).T

#Initial guess for theta values generated using CoppeliaSim
# thetalist0 = np.array([2.691, -0.712, 1.145, -0.712, -0.340, -1.516])
thetalist0 = np.array([0, 0, 0, 0, 0, 0])

#Angular Error tolerance
eomeg = 0.001
#Linear Error tolerance
ev = 0.0001

theta, error = IKinBodyIterates(Blist, M, Tsd, thetalist0, eomeg, ev)

print("\n\nFinal result: ", 
"\ntheta :", np.array2string(theta, precision=3, separator=',', suppress_small=True).replace("[", "").replace("]", "").replace("\n", ""),
"\nerror :", error)

#-------------End of find inverse kinematics for UR5 robot---------------------