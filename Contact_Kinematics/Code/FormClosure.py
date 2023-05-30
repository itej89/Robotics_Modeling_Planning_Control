import math
import numpy as np
from scipy.optimize import linprog

pi = math.pi 




def CrossProduct(v1, v2):
    return (v1[0]*v2[1]) - (v1[1]*v2[0]);


def  CheckForFormClosure(normals, number_of_contacts):
    F = []
    
    #Conver coordinates and angles into wrench vectors
    for n in normals:
        h = math.cos(n[2])
        if abs(h) < 1e-10:
            h = 0
        v = math.sin(n[2]) 
        if abs(v) < 1e-10:
            v = 0

        F.append([CrossProduct((n[0], n[1]), (h,v)), h, v])


    f = np.array([1]*number_of_contacts)
    A = np.identity(number_of_contacts)*-1
    b = np.array([-1]*number_of_contacts)
    F = np.array(F).T
    print(F)
    Aeq = F
    beq = np.array([0,0,0])
    k = linprog(f,A,b,Aeq,beq,method="interior-point")
    print(k)

    FormClosure = False
    if all(i >= 1 for i in k["x"]):
        FormClosure = True

    return (FormClosure, k["x"])


if __name__ == "__main__":
    print("\n\n---------------------------------------------------")
    print("Form Closure")
    print("---------------------------------------------------")
    normals = [[0,0, pi], [0,0, 3*pi/2], [2,1, 0], [2,1, pi/2]]
    print("Contact points :", normals)

    FormClosure, k = CheckForFormClosure(normals,4)

    if FormClosure:
        print("Form Closure successful")
        print("Elemets of k:",k)

    else:
        print("Form Closure failed")
        print("Elemets of k:",k)
    print("---------------------------------------------------")



    print("\n\n---------------------------------------------------")
    print("No Form Closure")
    print("---------------------------------------------------")
    normals = [[0,0, pi], [0,0, 3*pi/2], [2,0.5, 0]]
    print("Contact points :", normals)

    FormClosure, k = CheckForFormClosure(normals,3)

    if FormClosure:
        print("Form Closure successful")
        print("Elemets of k:",k)

    else:
        print("Form Closure failed")
        print("Elemets of k:",k)
    print("---------------------------------------------------")
