#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import csv
import matplotlib.pyplot as plt
import numpy as np

SAVEDATA = False


class AmclSaveValues():
    def __init__(self):
        rospy.init_node('PlotCov')
        self.sub_particles = rospy.Subscriber(
            'amcl_pose', PoseWithCovarianceStamped, self.CallbackError)
        self.publish_x_upper = rospy.Publisher('x_upper', Float32)
        self.publish_x_lower = rospy.Publisher('x_lower', Float32)
        self.publish_y_upper = rospy.Publisher('y_upper', Float32)
        self.publish_y_lower = rospy.Publisher('y_lower', Float32)

    def CallbackError(self, msg):
        pose = msg.pose.pose.position
        cov = msg.pose.covariance
        std = math.sqrt(cov[0])
        self.publish_x_upper.publish(pose.x + 2*std)
        self.publish_x_lower.publish(pose.x - 2*std)
        std = math.sqrt(cov[7])
        self.publish_y_upper.publish(pose.y + 2*std)
        self.publish_y_lower.publish(pose.y - 2*std)

    def run(self):
        rospy.spin()


class AmclPlotCov():

    def __init__(self):
        self.position_x = []
        self.position_y = []
        self.time = []

    def PlotCov(self):
        fig, ax = plt.subplots(2, 1, figsize=(10, 6))

        # Plot Position X with 95% Confidence Interval
        ax[0].plot(self.time, self.position_x[:, 0], 'o', label='Estimated X Position')
        ax[0].fill_between(self.time, self.position_x[:, 2], self.position_x[:, 1],
                           alpha=0.2, label='95% Confidence Interval for X')
        ax[0].set_ylabel('Position X (m)')
        # Plot between 0 and 1
        ax[0].set_ylim([-1.5, 2])
        ax[0].legend()
        ax[0].grid()

        # Plot Position Y with 95% Confidence Interval
        ax[1].plot(self.time, self.position_y[:, 0], 'o', label='Estimated Y Position')
        ax[1].fill_between(self.time, self.position_y[:, 2], self.position_y[:, 1],
                           alpha=0.2, label='95% Confidence Interval for Y')
        ax[1].set_ylabel('Position Y (m)')
        ax[1].set_ylim([-1.5, 2])
        ax[1].legend()
        ax[1].grid()

        # General plot adjustments
        plt.tight_layout(rect=[0, 0, 1, 0.95])
        plt.xlabel("Time (s)")
        # fig.suptitle("Estimated Position for AMCL with Confidence Intervals", fontsize=16)
        plt.show()

    def load(self, filepath):
        with open(filepath, 'r') as csvfile:
            reader = csv.reader(csvfile)
            next(reader)

            for row in reader:
                self.time.append(float(row[0]))
                # position, lower, upper
                self.position_x.append([float(row[43]), float(row[46]), float(row[48])])
                self.position_y.append([float(row[44]), float(row[47]), float(row[49])])

            self.time = np.array(self.time) - self.time[0]
            self.position_x = np.array(self.position_x)
            self.position_y = np.array(self.position_y)


if __name__ == '__main__':
    if SAVEDATA:  # Not needed anymore since we save everything through plotjuggler
        AmclPlotCov = AmclPlotCov()
        AmclPlotCov.run()
    else:
        plot = AmclPlotCov()
        plot.load('/home/rods/Desktop/IRob/plots/Cov_amcl_dataset_update_fix.csv')
        plot.PlotCov()
