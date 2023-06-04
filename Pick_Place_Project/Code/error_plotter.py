import matplotlib.pyplot as plt
import csv
import os
from pathlib import Path

from KinematicsConfiguration import *




def plot_error(file, TaskName):
    """Plots error.csv file in matplot and saves it as error_plot.png in the "results/{TaskName}" directory
        time interfaval for the plot is extracted from "FeedbackControl_Parameters->delta_t" in "KinematicsConfiguration.py" file

    :param file: error.csv  file absolute path containing Xerror data in 6-vector
    :param TaskName: the sub-directory in "results" directory into which the plot is saved as error_plot.png

    :return: 
        saves matplot of Xerror data into error_plot.png file in "results/{TaskName}/error_plot.png"

    Example Input:
                file = "/home/Name/Documents/Project/results/best/error.csv"
                TaskName = "best"
    Output:
        -Writes Xerror plot to  "/home/Name/Documents/Project/results/best/error_plot.png"
        """

    task_directory = os.path.join(Path(os.path.abspath(__file__)).parent.parent, "results" ,TaskName)
    if not os.path.exists(task_directory):
        os.mkdir(task_directory)

    x1 = []
    x2 = []
    x3 = []
    x4 = []
    x5 = []
    x6 = []
    Time = []
    
    with open(file,'r') as csvfile:
        #re4ad error.csv file
        plots = csv.reader(csvfile, delimiter=',')
        time_step = 0
        for row in plots:
            #extract 6 components of the Xerror from the csv file
            x1.append(float(row[0]))
            x2.append(float(row[1]))
            x3.append(float(row[2]))
            x4.append(float(row[3]))
            x5.append(float(row[4]))
            x6.append(float(row[5]))
            #Create Time axis at an interval "FeedbackControl_Parameters->delta_t" in "KinematicsConfiguration.py" file
            Time.append(time_step)
            time_step += FeedbackControl_Parameters.delta_t

    #Plot all 6 components of the Xerror again Same Time axis
    plt.figure()
    plt.plot(Time, x1, label='Xerr[0]')
    plt.plot(Time, x2, label='Xerr[1]')
    plt.plot(Time, x3, label='Xerr[2]')
    plt.plot(Time, x4, label='Xerr[3]')
    plt.plot(Time, x5, label='Xerr[4]')
    plt.plot(Time, x6, label='Xerr[5]')
    plt.xlabel('Time')
    plt.ylabel('Xerror')
    plt.title('Error Plot')
    plt.legend()

    #Save the plot at "task_directory/error_plot.png"
    plt.savefig(os.path.join(task_directory,"error_plot.png"))


if __name__ == "__main__":
    #Error Code to visulize an Xerr plot
    plot_error("/home/tej/Documents/Courses/Coursera/Robotics/Projects/Robotics Course 6 (Capstone) Project/results/overshoot/error.csv","")
    plt.show()