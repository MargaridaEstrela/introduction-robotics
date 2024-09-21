#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import TimeMsg
import numpy as np


class SaveData:
    def __init__(self):
        self.Odom = {"PosX": [], "PosY": [], "Time": []}
        self.Gt = {"PosX": [], "PosY": [], "Time": []}

        rospy.init_node('SaveData')

        rospy.Subscriber('odometry/filtered', Odometry,
                         self.CallbackOdom)
        rospy.Subscriber('ground_truth', PoseWithCovarianceStamped,
                         self.CallbackGT)

    def CallbackOdom(self, msg):
        self.Odom["PosX"].append(msg.pose.pose.position.x)
        self.Odom["PosY"].append(msg.pose.pose.position.y)
        self.Odom["Time"].append(TimeMsg(msg))

    def CallbackGT(self, msg):
        self.Gt["PosX"].append(msg.pose.pose.position.x)
        self.Gt["PosY"].append(msg.pose.pose.position.y)
        self.Gt["Time"].append(TimeMsg(msg))

    def save_data(self):
        OdomX = np.array(self.Odom["PosX"])
        OdomY = np.array(self.Odom["PosY"])
        OdomTime = np.array(self.Odom["Time"])
        GtX = np.array(self.Gt["PosX"])
        GtY = np.array(self.Gt["PosY"])
        GtTime = np.array(self.Gt["Time"])

        np.savez_compressed('odometry_data.npz', PosX=OdomX,
                            PosY=OdomY, Time=OdomTime)
        np.savez_compressed('ground_truth_data.npz',
                            PosX=GtX, PosY=GtY, Time=GtTime)

        rospy.loginfo("Data saved")

    def run(self):
        rospy.spin()
        self.save_data()


if __name__ == '__main__':
    plot = SaveData()
    plot.run()
