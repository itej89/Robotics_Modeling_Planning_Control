import matplotlib.pyplot as plt
import csv

from KinematicsConfiguration import *

x1 = []
x2 = []
x3 = []
x4 = []
x5 = []
x6 = []
Time = []


def plot_error(file):
    with open(file,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        print(plots)
        time_step = 0
        for row in plots:
            x1.append(float(row[0]))
            x2.append(float(row[1]))
            x3.append(float(row[2]))
            x4.append(float(row[3]))
            x5.append(float(row[4]))
            x6.append(float(row[5]))
            Time.append(time_step)
            time_step += FeedbackControl_Parameters.delta_t

    plt.plot(Time, x1, label='Loaded from file!')
    plt.plot(Time, x2, label='Loaded from file!')
    plt.plot(Time, x3, label='Loaded from file!')
    plt.plot(Time, x4, label='Loaded from file!')
    plt.plot(Time, x5, label='Loaded from file!')
    plt.plot(Time, x6, label='Loaded from file!')
    plt.xlabel('Time')
    plt.ylabel('error')
    plt.title('Xerr Plot')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    plot_error("/home/tej/Documents/Courses/Coursera/Robotics/Projects/Robotics Course 6 (Capstone) Project/Code/newtask/error.csv")