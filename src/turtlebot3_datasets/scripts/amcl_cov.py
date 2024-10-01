#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Float32
import numpy as np
import tf


class AMCLErrorPublisher:
    def __init__(self):

        rospy.init_node("amcl_cov_publisher")

        self.amcl_msg = None
        self.particles_msg = None
        self.processing = False

        # Subscribers
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("particlecloud", PoseArray, self.particles_callback)

        # Publisher for error
        self.error_cov_pub = rospy.Publisher("amcl_cov", Float32, queue_size=50)
        self.error_particles_pub = rospy.Publisher("amcl_num_particles", Float32, queue_size=50)

        # Store the latest AMCL message and particles message

    def amcl_callback(self, msg):
        if not self.processing:
            self.amcl_msg = msg
            self.CheckMsg()

    def particles_callback(self, msg):
        if not self.processing:
            self.particles_msg = msg
            self.CheckMsg()

    def CheckMsg(self):
        if self.amcl_msg and self.particles_msg:
            self.processing = True
            self.calculate_and_publish_error()

    def calculate_and_publish_error(self):
        amcl_pos = np.array([self.amcl_msg.pose.pose.position.x,
                            self.amcl_msg.pose.pose.position.y])

        particles = np.array([[pose.position.x, pose.position.y]
                              for pose in self.particles_msg.poses])

        error = np.linalg.norm(amcl_pos-particles, axis=1)

        # Just keep the distance between the AMCL pose and the furthest particle
        error = np.max(error)

        # Update total error and message count
        total_particles = len(particles)

        self.error_cov_pub.publish(error)
        self.error_particles_pub.publish(total_particles)

        # Reset amcl message and particles msg for the next iteration
        self.amcl_msg = None
        self.particles_msg = None
        self.processing = False


if __name__ == "__main__":
    AMCLErrorPublisher()
    rospy.spin()
