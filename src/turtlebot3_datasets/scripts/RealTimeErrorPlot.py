#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import TimeMsg
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class ErrorPlot:
    def __init__(self):
        self.Translation = np.array([0.939, 1.275])
        Quat = np.array([0.001, -0.003, 0.738, 0.675])
        self.Rotation = R.from_quat(Quat).as_matrix()[:2, :2]
        self.LastOdom = None
        self.LastGT = None
        self.InitialTime = None
        self.processing = False
        self.x_data = []
        self.y_data = []
        self.TotalError = 0.0
        self.TotalMsgs = 0.0
        self.AvgError = 0.0
        rospy.init_node('ErrorPlot')
        rospy.Subscriber('odometry/filtered', Odometry,
                         self.CallbackOdom, queue_size=1)
        rospy.Subscriber('ground_truth', PoseWithCovarianceStamped,
                         self.CallbackGT, queue_size=1)

    def CallbackOdom(self, msg):
        if not self.processing:
            self.LastOdom = msg
            self.CheckMsg()

    def CallbackGT(self, msg):
        if not self.processing:
            self.LastGT = msg
            self.CheckMsg()

    def TransformPosition(self, PosX, PosY):
        OrigPos = np.array([PosX, PosY])
        return self.Rotation @ OrigPos + self.Translation

    def EuclideanDistance(self):
        OdomPosi = self.TransformPosition(
            self.LastOdom.pose.pose.position.x, self.LastOdom.pose.pose.position.y)
        GTPosi = [self.LastGT.pose.pose.position.x,
                  self.LastGT.pose.pose.position.y]
        return math.sqrt((OdomPosi[0] - GTPosi[0]) ** 2 + (OdomPosi[1] - GTPosi[1]) ** 2)

    def CheckMsg(self):
        if self.LastOdom is not None and self.LastGT is not None:
            self.processing = True
            self.Error()

    def Error(self):
        if self.InitialTime is None:
            self.InitialTime = TimeMsg(self.LastGT)

        ErrorDist = self.EuclideanDistance()
        self.x_data.append(TimeMsg(self.LastGT) - self.InitialTime)
        self.y_data.append(ErrorDist)

        self.TotalError += ErrorDist
        self.TotalMsgs += 1
        self.AvgError = self.TotalError / self.TotalMsgs

        self.LastOdom = None
        self.LastGT = None

        self.processing = False

    def get_x_data(self):
        return self.x_data

    def get_y_data(self):
        return self.y_data

    def GetError(self):
        return self.AvgError


if __name__ == '__main__':
    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], 'r-')
    plt.xlabel("Time (s)")
    plt.ylabel("Error (m)")
    plt.title("Error between Ground Truth and EKF")
    error_text = ax.text(0.95, 0.95, '', transform=ax.transAxes,
                         verticalalignment='top', horizontalalignment='right',
                         bbox=dict(facecolor='white', alpha=0.5))
    plt.show()
    plot = ErrorPlot()
    while not rospy.is_shutdown():
        line.set_xdata(plot.get_x_data())
        line.set_ydata(plot.get_y_data())
        error_text.set_text(f"Average Error: {plot.GetError():.3f} m")
        ax.relim()
        ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)
