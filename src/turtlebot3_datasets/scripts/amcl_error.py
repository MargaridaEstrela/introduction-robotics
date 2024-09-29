#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import numpy as np
import tf


class AMCLErrorPublisher:
    def __init__(self):
        self.total_error = 0.0
        self.total_msgs = 0

        rospy.init_node("amcl_error_publisher")

        # Subscribers
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)

        # Publisher for error
        self.error_pub = rospy.Publisher("amcl_error", Float32, queue_size=10)

        # Store the latest AMCL message
        self.amcl_msg = None

        # Initialize tf listener to get the odometry pose from tf tree
        self.tf_listener = tf.TransformListener()

    def amcl_callback(self, msg):
        self.amcl_msg = msg
        self.calculate_and_publish_error()

    def get_odom_pose(self):
        try:
            # Get the transformation from odom to base_link (current odometry pose)
            (trans, rot) = self.tf_listener.lookupTransform(
                "/odom", "/base_link", rospy.Time(0)
            )
            return np.array([trans[0], trans[1]])  # Get x and y positions from odometry
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            return None

    def calculate_and_publish_error(self):
        if self.amcl_msg:
            # Get AMCL position
            amcl_pos = np.array(
                [self.amcl_msg.pose.pose.position.x, self.amcl_msg.pose.pose.position.y]
            )

            # Get odometry position from tf (odom to base_link)
            odom_pos = self.get_odom_pose()
            if odom_pos is None:
                return  # Skip if we can't get the odometry position

            # Calculate Euclidean distance between amcl and odometry (error)
            error = np.linalg.norm(amcl_pos - odom_pos)

            # Update total error and message count
            self.total_error += error
            self.total_msgs += 1

            # Compute average error
            avg_error = self.total_error / self.total_msgs

            # Publish the average error
            self.error_pub.publish(avg_error)

            # Reset amcl message for the next iteration
            self.amcl_msg = None


if __name__ == "__main__":
    AMCLErrorPublisher()
    rospy.spin()
