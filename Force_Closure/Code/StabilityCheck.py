import math
import numpy as np
from scipy.optimize import linprog
import json
from DescritionTypes import *

pi = math.pi 
g = 9.8


class ForceClosureEvaluator:

    #method to verify Form closure given mass and contact descriptions
    def Verify_force_closure(self, mass_properties, contact_descriptions)->bool:
        body_wrench_list = {}
        body_external_wrench_list = {}

        #Convert mass description to external force on the body
        for mass_desc in mass_properties:
            print("mass  : ",json.dumps(mass_desc.__dict__))
            if mass_desc.body_id not in body_external_wrench_list.keys():
                body_external_wrench_list[mass_desc.body_id] = []
            g_force = -1*mass_desc.mass*g
            body_external_wrench_list[mass_desc.body_id].append([self.CrossProduct(mass_desc.cm_location, (0, g_force)), 0, g_force])

        #Convert contact descripn to wrenches
        for contact in contact_descriptions:
            print("contact  : ",json.dumps(contact.__dict__))
            if contact.body_id not in body_wrench_list.keys():
                body_wrench_list[contact.body_id] = []
            
            body_wrench_list[contact.body_id].append([contact.location[0], contact.location[1], contact.theta_normal+contact.alpha_wrench])
            body_wrench_list[contact.body_id].append([contact.location[0], contact.location[1], contact.theta_normal-contact.alpha_wrench])

        #Check constriant coeffients for contact wrenches of each body

        body_wrench_coefficients = {}
        for mass in mass_properties:
            closure_Status, k = self.CheckForConstraints(body_wrench_list[mass.body_id], body_external_wrench_list[mass.body_id], 3)

            if not closure_Status:
                print("\nForce Closure failed for mass : ",mass.body_id)
                print("\nDetails : ", k)
                return False, {},{},{}

            body_wrench_coefficients[mass.body_id] = k["x"]

        return True, body_wrench_list, body_external_wrench_list, body_wrench_coefficients


    #Method to perform linear programming using inequality contstraints
    #Takes 3 arguments (contact points with normal directions, External force, dimentions(indicating planar(3)/spacial(6)))
    def  CheckForConstraints(self, contacts, Fext ,dimention):

        #Variable holds list of wrenches calculated at contact points
        F = []

        #Conver coordinates and angles into wrench vectors
        #each n = (x,y, theta)
        for n in contacts:
            F.append(self.ConvertToWrench(n[2], (n[0], n[1])))

        f = np.array([1]*len(contacts))
        A = np.identity(len(contacts))*-1
        b = np.array([-1]*len(contacts))
        F = np.array(F).T
        Aeq = F
        beq = np.array(Fext)*-1

        k = linprog(f,A,b,Aeq,beq, method='interior-point')

        ForceClosure = False
        #Check if all coefficients are non negative
        if all(i >= 0 for i in k["x"]) and k["success"]:
            ForceClosure = True

        return (ForceClosure,k)


    def CrossProduct(self, v1, v2):
        return (v1[0]*v2[1]) - (v1[1]*v2[0])

    def ConvertToWrench(self, theta, p):
        h = math.cos(theta)
        if abs(h) < 1e-10:
            h = 0
        v = math.sin(theta) 
        if abs(v) < 1e-10:
            v = 0
        
        return([self.CrossProduct((p[0], p[1]), (h,v)), h, v])


if __name__ == "__main__":

    _ForceClosureEvaluator = ForceClosureEvaluator()

    print("\n\n---------------------------------------------------")
    print("Form Closure : For Given Test Assembly")
    print("---------------------------------------------------")
    mass_description = [
        mass_properties(body_ID=1, mass=2,  CM_location=(25,35)),
        mass_properties(body_ID=2, mass=10, CM_location=(66,42))
    ]

    contact_descriptions = [
        contact_description(1, 0, (0,0),   pi/2, 0.5),
        contact_description(1, 2, (60,60), pi,   0.5),
        contact_description(2, 1, (60,60), 0,    0.5),
        contact_description(2, 0, (60,0),  pi/2, 0.5),
        contact_description(2, 0, (72,0),  pi/2, 0.5),
    ]

    status, body_wrench_list, body_external_wrench_list, body_wrench_coefficients = _ForceClosureEvaluator.Verify_force_closure(mass_description, contact_descriptions)
    
    if status:
        print("\nForce Closure Succesful")



    print("\n\n---------------------------------------------------")
    print("No Form Closure : For Given Test Assembly")
    print("---------------------------------------------------")
    mass_description = [
        mass_properties(body_ID=1, mass=2,  CM_location=(25,35)),
        mass_properties(body_ID=2, mass=5,  CM_location=(66,42))
    ]

    contact_descriptions = [
        contact_description(1, 0, (0,0),   pi/2, 0.1),
        contact_description(1, 2, (60,60), pi,   0.5),
        contact_description(2, 1, (60,60), 0,    0.5),
        contact_description(2, 0, (60,0),  pi/2, 0.5),
        contact_description(2, 0, (72,0),  pi/2, 0.5),
    ]

    status, body_wrench_list, body_external_wrench_list, body_wrench_coefficients = _ForceClosureEvaluator.Verify_force_closure(mass_description, contact_descriptions)
    
    if status:
        print("\nForce Closure Succesful")



    print("\n\n---------------------------------------------------")
    print("Form Closure : For Own Assembly")
    print("---------------------------------------------------")
    mass_description = [
        mass_properties(body_ID=1, mass=2,  CM_location=(25,35)),
        mass_properties(body_ID=2, mass=10, CM_location=(66,42)),
        mass_properties(body_ID=3, mass=10, CM_location=(111,35))
    ]

    contact_descriptions = [
        contact_description(1, 0, (0,0),   pi/2, 0.5),
        contact_description(1, 2, (60,60), pi,   0.5),
        
        contact_description(2, 0, (60,0),  pi/2, 0.5),
        contact_description(2, 0, (72,0),  pi/2, 0.5),

        contact_description(2, 1, (60,60), 0,    0.5),
        contact_description(2, 3, (72,60), pi,    0.5),

        contact_description(3, 0, (132,0),   pi/2, 0.5),
        contact_description(3, 2, (72,60),   0,    0.5),
    ]

   
    status, body_wrench_list, body_external_wrench_list, body_wrench_coefficients = _ForceClosureEvaluator.Verify_force_closure(mass_description, contact_descriptions)
    
    if status:
        #----------------------------------------------------------------------------------------------
        #Evaluating toatal forces on each body
        #----------------------------------------------------------------------------------------------
        print("\n---------------------------------------------------------------------------------------")
        print("Evaluation of total forces on each body")
        print("--------------------------------------------------------------------------------------")
        for body_ID in body_wrench_list.keys():
            i=0
            Total_wrench = [0]*3
            for wrench in body_wrench_list[body_ID]:
                _wrench =  [k * body_wrench_coefficients[body_ID][i] for k in wrench]
                Total_wrench = [a + b for a, b in zip(Total_wrench, _wrench)]
                i +=1
            
            print("\nTotal Wrench of Body ",body_ID, " is ", Total_wrench)
            print("Gravitational Wrench of Body ",body_ID, " is ", body_external_wrench_list[body_ID])
        print("\n--------------------------------------------------------------------------------------")
        #----------------------------------------------------------------------------------------------

        print("\nForce Closure Succesful")



    print("\n\n---------------------------------------------------")
    print("No Form Closure : For Own Assembly")
    print("---------------------------------------------------")
    mass_description = [
        mass_properties(body_ID=1, mass=2,  CM_location=(25,35)),
        mass_properties(body_ID=2, mass=10, CM_location=(66,42)),
        mass_properties(body_ID=3, mass=10, CM_location=(111,35))
    ]

    contact_descriptions = [
        contact_description(1, 0, (0,0),   pi/2, 0.5),
        contact_description(1, 2, (60,60), pi,   0.5),
        
        contact_description(2, 0, (60,0),  pi/2, 0.5),
        contact_description(2, 0, (72,0),  pi/2, 0.5),

        contact_description(2, 1, (60,60), 0,    0.5),
        contact_description(2, 3, (72,60), pi,    0.5),

        contact_description(3, 0, (132,0),   pi/2, 0.1),
        contact_description(3, 2, (72,60),   0,    0.5),
    ]

    status, body_wrench_list, body_external_wrench_list, body_wrench_coefficients = _ForceClosureEvaluator.Verify_force_closure(mass_description, contact_descriptions)
    
    if status:
        print("\nForce Closure Succesful")






























