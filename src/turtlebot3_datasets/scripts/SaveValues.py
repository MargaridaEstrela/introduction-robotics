#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import TimeMsg
import numpy as np
import os

FILE_END_NAME = "NewCov"


class SaveData:
    def __init__(self):
        self.Estimation = {"PosX": [], "PosY": [],
                           "Time": [], "CovX": [], "CovY": []}
        self.Gt = {"PosX": [], "PosY": [], "Time": []}

        rospy.init_node('SaveData')

        rospy.Subscriber('odometry/filtered', Odometry,
                         self.CallbackEstimation)
        rospy.Subscriber('ground_truth', PoseWithCovarianceStamped,
                         self.CallbackGT)

    def CallbackEstimation(self, msg):
        self.Estimation["PosX"].append(msg.pose.pose.position.x)
        self.Estimation["PosY"].append(msg.pose.pose.position.y)
        self.Estimation["Time"].append(TimeMsg(msg))
        self.Estimation["CovX"].append(msg.pose.covariance[0])
        self.Estimation["CovY"].append(msg.pose.covariance[7])

    def CallbackGT(self, msg):
        self.Gt["PosX"].append(msg.pose.pose.position.x)
        self.Gt["PosY"].append(msg.pose.pose.position.y)
        self.Gt["Time"].append(TimeMsg(msg))

    def save_data(self):
        EstimationX = np.array(self.Estimation["PosX"])
        EstimationY = np.array(self.Estimation["PosY"])
        EstimationTime = np.array(self.Estimation["Time"])
        EstimationCovX = np.array(self.Estimation["CovX"])
        EstimationCovY = np.array(self.Estimation["CovY"])

        np.savez_compressed(os.path.join('Error_data', 'EKF_data_' + FILE_END_NAME + '.npz'), PosX=EstimationX,
                            PosY=EstimationY, Time=EstimationTime, CovX=EstimationCovX, CovY=EstimationCovY)

        GtX = np.array(self.Gt["PosX"])
        GtY = np.array(self.Gt["PosY"])
        GtTime = np.array(self.Gt["Time"])

        np.savez_compressed(os.path.join('Error_data', 'ground_truth_data_' + FILE_END_NAME + '.npz'),
                            PosX=GtX, PosY=GtY, Time=GtTime)

        rospy.loginfo("Data saved")

    def run(self):
        rospy.spin()
        self.save_data()


if __name__ == '__main__':
    plot = SaveData()
    plot.run()
