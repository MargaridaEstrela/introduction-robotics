#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import numpy as np


class EKFErrorPublisher:
    def __init__(self):
        self.total_error = 0.0
        self.total_msgs = 0

        rospy.init_node("amcl_error_publisher")

        # Subscribers
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("ground_truth", PoseWithCovarianceStamped, self.gt_callback)

        # Publisher for error
        self.error_pub = rospy.Publisher("amcl_error", Float32, queue_size=10)

        # Store the latest messages
        self.amcl_msg = None
        self.gt_msg = None

    def amcl_callback(self, msg):
        self.amcl_msg = msg
        self.calculate_and_publish_error()

    def gt_callback(self, msg):
        self.gt_msg = msg
        self.calculate_and_publish_error()

    def calculate_and_publish_error(self):
        if self.amcl_msg and self.gt_msg:
            # Amcl position
            amcl_pos = np.array(
                [self.amcl_msg.pose.pose.position.x, self.amcl_msg.pose.pose.position.y]
            )
            # Ground truth position
            gt_pos = np.array(
                [self.gt_msg.pose.pose.position.x, self.gt_msg.pose.pose.position.y]
            )

            # Calculate Euclidean distance between amcl and ground truth
            error = np.linalg.norm(amcl_pos - gt_pos)

            # Update total error and message count
            self.total_error += error
            self.total_msgs += 1

            # Compute average error
            avg_error = self.total_error / self.total_msgs

            # Publish the average error
            self.error_pub.publish(avg_error)

            # Reset the amcl and ground truth messages
            self.amcl_msg = None
            self.gt_msg = None


if __name__ == "__main__":
    EKFErrorPublisher()
    rospy.spin()
