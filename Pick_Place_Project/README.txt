Modern Robotics Capstone Project Description:
http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone

What to submit: You will submit a single .zip file of a directory with the following contents:

    1. A file called README.txt or README.pdf. This file should briefly explain your software and your results. If you needed to follow a different approach to solve the problem than the one described above, explain why and explain your solution method. If you encountered anything surprising, or if there is something you still don't understand, or if you think an important point is neglected in the description of the project on this page, explain it. If you implemented singularity avoidance, joint limit avoidance, or any other enhancement over the basic project description given on this page, explain your method. You may also wish to include more results in the three results directories described below, showing the results when using your enhancements vs. when you don't use your enhancements, to highlight the value of your enhancements.
    2. Your commented code in a directory called "code." Your code should be commented, so it is clear to the reader what the code is doing. No need to go overboard with too many comments, but keep in mind your reviewer may not be fluent in your programming language. Your code comments must include an example of how to use the code, and to make the code easy to run, each separate task you solve should have its own script, so by invoking the script, the code runs with all the appropriate inputs. (This makes it easy for others to test your code and modify it to run with other inputs.) Apart from the scripts, only turn in functions that you wrote or modified; you don't need to turn in other MR functions that your code uses. If your code is in MATLAB or Python, just turn in the source files (text files) with your functions. If your code is in Mathematica, turn in (a) your .nb notebook file and (b) a .pdf printout of your code, so a reviewer can read your code without having to have the Mathematica software.
    3. A directory called "results" with the results of your program. This directory should contain three directories: one titled "best," one titled "overshoot," and one titled "newTask." The directories "best" and "overshoot" both solve a pick-and-place task where the initial and final configurations of the cube are at the default locations in the capstone CoppeliaSim scene, i.e., the initial block configuration is at (x,y,\theta) = (1~\text{m}, 0~\text{m}, 0~\text{rad}) and the final block configuration is at (x,y,\theta) = (0~\text{m},-1~\text{m},-\pi/2~\text{rad}). The directory "newTask" should have different initial and final block configurations, which you are free to choose yourself. In all cases, the initial configuration of the end-effector should have at least 30 degrees of orientation error and 0.2 m of position error from the first configuration on the reference trajectory. The directory "best" should contain results using a well-tuned controller, either feedforward-plus-P or feedforward-plus-PI. The convergence exhibited by the controller does not necessarily have to be fast (in fact, it is more interesting if the convergence is not too fast, so the transient response is clearly visible), but the motion should be smooth, with no overshoot, and very little error by partway through trajectory segment 1. The directory "overshoot" should contain the results using a less-well-tuned controller, one that exhibits overshoot and a bit of oscillation. Nonetheless, the error should be eliminated before the end of trajectory segment 1. Your controller for "overshoot" will likely be feedforward-plus-PI or just PI. You can use any controller to solve the "newTask" task. In each of the three directories, give:
        A very brief README.txt or README.pdf file that indicates the type of controller, the feedback gains, and any other useful information about the results. For the "newTask" directory, indicate the initial and goal configurations for the cube.
        The CoppeliaSim .csv file produced by your program when it is called with the input from the log file.
        A video of your .csv file being animated by the CoppeliaSim scene.
        The Xerr data file produced by your program.
        A plot of the six elements of Xerr as a function of time, showing the convergence to zero. This plot should not require any special software (e.g., MS excel) to be viewable. In other words, you should save it as a .pdf or other freely-viewable format.
        A log file showing your program being called with the input. In MATLAB, for example, this log file could be something like: 



--------------------------------------------------------------------------------------------------------------------------------
SUBMISSION:
--------------------------------------------------------------------------------------------------------------------------------
Project Directory structure: 

    Robotics Course 6 (Capstone) Project

       ├── README.txt (this file)
       ├── Code
       │   ├── Main.py
       │   ├── KinematicSimulator.py
       │   └── TrajectoryGenerator.py
       │   ├── FeedbackControl.py
       │   ├── error_plotter.py
       │   ├── KinematicsConfiguration.py
       └── results
           ├── best
           │   ├── CoppeliaSim.csv
           │   ├── error.csv
           │   ├── error_plot.png
           │   ├── log.txt
           │   ├── README.txt
           │   ├── trajectory.csv
           │   └── Video.mp4
           ├── milestone_tests
           │   ├── Feedforward_control_trajectory_test.csv
           │   ├── Odometry_test.csv
           │   └── trajectory_test.csv
           ├── newtask
           │   ├── CoppeliaSim.csv
           │   ├── error.csv
           │   ├── error_plot.png
           │   ├── log.txt
           │   ├── README.txt
           │   ├── trajectory.csv
           │   └── video.mp4
           └── overshoot
               ├── CoppeliaSim.csv
               ├── error.csv
               ├── error_plot.png
               ├── log.txt
               ├── README.txt
               ├── trajectory.csv
               └── video.mp4


--------------------------------------------------------------------------------------------------------------------------------
Description of "Code" Directory contents:
--------------------------------------------------------------------------------------------------------------------------------
          Parameter Configuration file:
            - KinematicsConfiguration.py - Contains all the parameters related to different sub tasks in the following
              classes
                - chasis_params, kinematic_configuration - Information provided by the Task
                - Cube_Configuration    - Modify this to change the task start and goal locations
                - Simulation_Parameters - Contans simulation time step and joint limits
                - Trajectory_parameters - Contains Trajectory Generator paramters
                - FeedbackControl_Parameters - Contains Controller parameters
            


          Milestone 1: youBot Kinematics Simulator and csv Output
                -KinematicSimulator.py - Contains Functionality related to SImulation and Odometry
                    - Implements "NextState" funtion to estimate robot configuration given its current configuration 
                    	and velocities
                        
                        NextState function: Performs robot configuration calculation of a 4 wheel Mecanum omnidirectional
                         robot with 5R arm given control velocities
                            Input:
                                :current_configuration: delta theta andles
                                :control_vector : A 9-vector containing the 5 arm joint velocities and 4 wheel velocities
                                :time_step : Simelation time step length in seconds
                                :joint_Speed_limit : An absolute value describing the joint velocity limit of 5R arm in 
                                 either direction(clockwise and anti-clockwise).
                                :wheel_Speed_limit : An absolute value describing the wheel speed limit in 
                                 either direction(forward and backward).
                            :Output:  
                                new_chasis_config : returns a 12 vector containing updated chasis configuration+
                                5 joint angles + 5 wheel angles


                    - Script usage for Milestone1 verification:
                        Run this script to create a sample odometry file at "results/milestone_tests/Odometry_test.csv"
                        (Loaded in Scene 6) that can be used for verification
                        Load the file "results/milestone_tests/Odometry_test.csv" inScene 6 to verify the motion against
                         milestone1 data given in Project description



            Milestone 2: Reference Trajectory Generation
            
                - TrajectoryGenerator.py - Contains Functionality related to Generate Trajectory
                    - Implements "TrajectoryGenerator" funtion to estimate robot configuration given its current 
                    configuration and velocities

                            TrajectoryGenerator function: Performs Feed back control and returns Control velocities and Xerr
                                Input:
                                    :Tse_initial: A 4x4 matrix containing the start point of the trajectory in {S} frame
                                    :Tsc_initial: A 4x4 matrix containing starting location of the cube in {S} frame
                                    :Tsc_final:   A 4x4 matrix containing final location of the cube in {S} frame
                                    :Tce_grasp:   A 4x4 matrix containing the end effector orientaion while grasping the 
                                    cube in cube-frame
                                    :Tce_standoff: A 4x4 matrix containing the end effector orientaion while stand off above
                                     the cube in cube-frame

                                :Output: 
                                    Full_Path : An array containing 8 subarrays of Trasformation matrices where each 
                                    array describs a sub trajectory
                                    GripperState : An array containing 8 subarrays dwscribing the gripper state with 
                                    respect to the Trasformation matrix in the Full path

                    - Script usage for Milestone2 verification:
                        Run this script to create a sample Trajectory file at "results/milestone_tests/trajectory_test.csv"
                        (Loaded in Scene 8) that can be used for verification
                        Load the file "results/milestone_tests/trajectory_test.csv" in Scene 8 to verify the motion against
                         milestone2 data given in Project description




            Milestone 3: Feedforward Control
          
                FeedbackControl.py - Contains Functionality related to feed back control loop            
                    - Implements "FeedbackControl" funtion to generate control vector givne the configuration and 
                    desired states
                        (The function also implementes a simple joint limit verification logic using control commands -
                        limits are taken usnig Scene 3)

                        FeedbackControl funtionality: Performs Feed back control and returns Control velocities and Xerr
                            Input:
                                :robot_configuration: A 6-vector containing the robot configuraiton in the format 
                                :Xd: A 4x4 matrix containing desired end effector configuration from trajectory at 
                                	current timestep
                                :Xd: A 4x4 matrix containing desired end effector configuration from trajectory at 
                                	next timestep
                                :feed_forward_enabled: Flag to inform if feed-forward part need to be added

                            :Output: 
                                control velocities: 9-vector (5 Joint velocities and 4 wheel velocities)
                                Xerr : Configuration error between robot configuraion X and desired configuration Xd at
                                 	current timestep

                    - Script usage for Milestone3 verification:
                        Run this script to create a feed-forward trajectory tracking estimated file at "results/milestone_tests
                        /Feedforward_control_trajectory_test.csv"(Scene 8) that can be used for verification
                        Load the file "results/milestone_tests/Feedforward_control_trajectory_test.csv" in Scene 8 to
                         verify the motion against milestone2 data given in Project description




            Final Step: Completing the Project and Your Submission
                Main.py - Main file 
                - Implements "Perform_Pick_and_Place" function that 
                - Integrates Simulation, Trajectory generation, Feedback control to simulate a pick and place task 

                    fundtionality: """Performs Pick and Place operation
                        Input:
                            :TaskName: A String Containning the Task Name (Decides the sub directory in "results" folder 
                            into which results are saved)
                            :Tse: A 4x4 matrix containing the initial configuration of the robot end effector in {S} frame
                            :robot_configuration: A 6-vector containing the robot configuraiton in the format 
                                    (Φ, x, y, θ1, θ2, θ3, θ4, θ5,, w1, w2, w3, w4) = 
                                    chasis configuration, 2 arm joint angles, 4wheel angles
                            :Kp: Proportional controller gain
                            :Ki: Integral controller gain
                            :feed_forward_enabled: Informs if feed forward portion needs to be added (enabled by default)

                        :Output: 
                            -Writes genrated Trajectory information to  "../results/{TaskName}/trajectory.csv"
                            -Writes Simulation information to  "../results/{TaskName}/CoppeliaSim.csv"
                            -Writes Xerror information to  "../results/{TaskName}/error.csv"
                            -Writes log information to  "../results/{TaskName}/log.txt" 
                            	(if "PRINT_TO_CONSOLE" is set to false in MAIN function)
                            -Writes Xerror plot to  "../results/{TaskName}/error_plot.png"
                
                    - contains the __Main__ logic to generate files for all three Tasks {"best", "overshoot", "newtask"}
                    
                    Script usage:
                        - use the variabel "PRINT_TO_CONSOLE" to route the log either to "log.txt" 
                        	(created in respective task directory) or to the "console"
                        - Run this file to generate all three taks {"best", "overshoot", "newtask"} related
                          files in the results directory

          ---- error_plotter.py - Contains useful functionality to plot Xerr data and save it
          			  as a "error_plot.png" file in the "resutls/[task]" directory





--------------------------------------------------------------------------------------------------------------------------------
Description of "results" Directory contents:
--------------------------------------------------------------------------------------------------------------------------------
        "..results/best" : The following configuration has been used for "best" results 
                            Inputs:
                                TYPE OF CONTROLLER : FEED_FORWARD-PI

                                Control Prameters:
                                Kp :  1.7
                                Ki :  0.02

                                Task paramters:
                                Cube start :  (x,y,θ) = [1, 0, 0]
                                Cube goal  :  (x,y,θ) = [0, -1,-π/2]


                                The configuration that satisfies the orientation difference greatethan 30 degree
                                 and position difference greater than 0.2m between robot congiguration and reference
                                  trajectory initial position.
                                  
                                    Trajectory start configuration -  orientation = [0.  1.  0.  0.  0.  0.5] 
                                    (for Tse :  [[0.0, 0.0, 1.0, 0.0], 
                                    [0.0, 1.0, 0.0, 0.0], 
                                    [-1.0, 0.0, 0.0, 0.5], 
                                    [0.0, 0.0, 0.0, 1.0]])
                                                                    

                                    Robot Initial end effector     - 
                                    orientation :  [0.  0.  0.46620836 0.17622725 0.34286871 0.7535] 
                                                   for chasis configuration : (phi,x,y) = [0.485, 0, 0.25], 
                                                       Joint angles=[0, 0, 0, 0, 0], 
                                                       wheel angles=[0, 0, 0, 0] 
                            Contents:
                                CoppeliaSim.csv - Simualtion file for Scene 6
                                Video.mp4       - Simulation video
                                trajectory.csv  - reference trajectory for best task for Scene 8
                                error.csv       - Xerr file
                                error_plot.png  - Xerror plot showing convergence to zero
                                log.txt         - log of configurationa and best task output
                                README.txt      - parameters and results of the best task


                            Observation: 
                                -As expected the robot correctly picks up the cube and places at final position without error
                                -the error plot clearly shows the all Xerr compoenents approaching zero
                            


        "..results/overshoot" : The following configuration has been used for "overshoot" results 
                            Inputs:
                                TYPE OF CONTROLLER : PI

                                Control Prameters:
                                Kp :  1.48
                                Ki :  0.15


                                Task paramters:
                                Cube start :  (x,y,θ) = [1, 0, 0]
                                Cube goal  :  (x,y,θ) = [0, -1,-π/2]


                                 The configuration that satisfies the orientation difference greatethan 30 degree
                                 and position difference greater than 0.2m between robot congiguration and reference
                                  trajectory initial position.
                                  
                                    Trajectory start configuration -  orientation = [0.  1.  0.  0.  0.  0.5] 
                                    (for Tse :  [[0.0, 0.0, 1.0, 0.0], 
                                    [0.0, 1.0, 0.0, 0.0], 
                                    [-1.0, 0.0, 0.0, 0.5], 
                                    [0.0, 0.0, 0.0, 1.0]])
                                                                    

                                    Robot Initial end effector     - 
                                    orientation :  [0.  0.  0.46620836 0.17622725 0.34286871 0.7535] 
                                                   for chasis configuration : (phi,x,y) = [0.485, 0, 0.25], 
                                                       Joint angles=[0, 0, 0, 0, 0], 
                                                       wheel angles=[0, 0, 0, 0] 
                                                       
                            Contents:
                                CoppeliaSim.csv - Simualtion file for Scene 6
                                Video.mp4       - Simulation video
                                trajectory.csv  - reference trajectory for best task for Scene 8
                                error.csv       - Xerr file
                                error_plot.png  - Xerror plot showing oscillations
                                log.txt         - log of configurationa and overshoot task output
                                README.txt      - parameters and results of the overshoot task


                             Observation: 
                                -As expected the robot overshoots in the beggining and then correctly picks up the cube and places at final position without error
                                -The error plot and the video clearly shows the oscillations in Xerr in the beggining and goes to zero by end of trajectory 1


        "..results/newtask" : The following configuration has been used for "newtask" results 
                            Inputs:
                                TYPE OF CONTROLLER : FEED_FORWARD-PI

                                Control Prameters:
                                Kp :  1.7
                                Ki :  0.02


                                Task paramters:
                                Cube start :  (x,y,θ) = [0,   1, -π/2]
                                Cube goal  :  (x,y,θ) = [-1, -2, -π/2]


                                The configuration that satisfies the orientation difference greatethan 30 degree
                                 and position difference greater than 0.2m between robot congiguration and reference
                                  trajectory initial position.
                                  
                                    Trajectory start configuration -  orientation = [0.  1.  0.  0.  0.  0.5] 
                                    (for Tse :  [[0.0, 0.0, 1.0, 0.0], 
                                    [0.0, 1.0, 0.0, 0.0], 
                                    [-1.0, 0.0, 0.0, 0.5], 
                                    [0.0, 0.0, 0.0, 1.0]])
                                                                    

                                    Robot Initial end effector     - 
                                    orientation :  [0.  0.  0.46620836 0.17622725 0.34286871 0.7535] 
                                                   for chasis configuration : (phi,x,y) = [0.485, 0, 0.25], 
                                                       Joint angles=[0, 0, 0, 0, 0], 
                                                       wheel angles=[0, 0, 0, 0] 
                                                        
                            Contents:
                                CoppeliaSim.csv - Simualtion file for Scene 6
                                Video.mp4       - Simulation video
                                trajectory.csv  - reference trajectory for best task for Scene 8
                                error.csv       - Xerr file
                                error_plot.png  - Xerror plot showing convergence to zero
                                log.txt         - log of configurationa and newtask task output
                                README.txt      - parameters and results of the newtask task


                            Observation: 
                                -As expected the robot correctly picks up the cube and places at final position without error
                                -the error plot clearly shows the all Xerr compoenents approaching zero



        "..results/milestone_tests" : The following files will getr generated when individual milestone files are ran
            The Input configuration to generate these files are given as test data in the foillowing link
            http://hades.mech.northwestern.edu/index.php/Mobile_Manipulation_Capstone

            Milestone1 validation:  Odometry_test.csv
                Input:
                    u = ( − 10,10,10, − 10). The robot chassis should spin counterclockwise in place by 1.234 radians.
                Output:
                    Load the file Odometry_test.csv to Scene 6 to validate that  robot behaves as expected


            Milestone2 validation:  trajectory_test.csv
                    Load the file trajectory_test.csv to Scene 8 to validate that a valid trajectory has been generated

            Milestone2 validation:  Feedforward_control_trajectory_test.csv
                    Load the file Feedforward_control_trajectory_test.csv to Scene 6 to validate that a controller
                     is able to approximately track the reference trajectory with feedforward controller

