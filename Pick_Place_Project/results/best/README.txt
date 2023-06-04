-----------------------------------------------------
Best task parameters and results
-----------------------------------------------------
Parameters:
    TYPE OF CONTROLLER : FEED_FORWARD-PI

    Control Prameters:
    Kp :  1.7
    Ki :  0.02

    Task paramters:
    Cube start :  (x,y,θ) = [1, 0, 0]
    Cube goal  :  (x,y,θ) = [0, -1,-pi/2]


    The configuration that satisfies the orientation difference greatethan 30 degree and position difference greater than 0.2m between robot congiguration and reference trajectory initial position.
        Trajectory start configuration -  orientation = [0.  1.  0.  0.  0.  0.5] (for Tse :  [[0.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.5], [0.0, 0.0, 0.0, 1.0]])
                                        

        Robot Initial end effector     - orientation :  [0.  0.  0.46620836 0.17622725 0.34286871 0.7535] 
                                        for chasis configuration : (phi,x,y) = [0.485, 0, 0.25], 
                                            Joint angles=[0, 0, 0, 0, 0], 
                                            wheel angles=[0, 0, 0, 0] 



Results: 
    -As expected the robot correctly picks up the cube and places at final position without error
    -the error plot clearly shows the all Xerr compoenents approaching zero