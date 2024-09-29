#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class EKFErrorPublisher:
    def __init__(self):
        self.translation = np.array([0.939, 1.275])
        quat = np.array([0.001, -0.003, 0.738, 0.675])
        self.rotation = R.from_quat(quat).as_matrix()[:2, :2]

        self.total_error = 0.0
        self.total_msgs = 0

        rospy.init_node("ekf_error_publisher")

        # Subscribers
        rospy.Subscriber("odometry/filtered", Odometry, self.odom_callback)
        rospy.Subscriber("ground_truth", PoseWithCovarianceStamped, self.gt_callback)

        # Publisher for error
        self.error_pub = rospy.Publisher("ekf_error", Float32, queue_size=10)

        # Store the latest messages
        self.odom_msg = None
        self.gt_msg = None

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.calculate_and_publish_error()

    def gt_callback(self, msg):
        self.gt_msg = msg
        self.calculate_and_publish_error()

    def calculate_and_publish_error(self):
        if self.odom_msg and self.gt_msg:
            # Transform odometry position
            odom_pos = self.transform_position(
                self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y
            )
            # Ground truth position
            gt_pos = np.array(
                [self.gt_msg.pose.pose.position.x, self.gt_msg.pose.pose.position.y]
            )

            # Calculate Euclidean distance between odometry and ground truth
            error = np.linalg.norm(odom_pos - gt_pos)

            # Update total error and message count
            self.total_error += error
            self.total_msgs += 1

            # Compute average error
            avg_error = self.total_error / self.total_msgs

            # Publish the average error
            self.error_pub.publish(avg_error)

            # Reset the odometry and ground truth messages
            self.odom_msg = None
            self.gt_msg = None

    def transform_position(self, x, y):
        orig_pos = np.array([x, y])
        return self.rotation @ orig_pos + self.translation


if __name__ == "__main__":
    EKFErrorPublisher()
    rospy.spin()
