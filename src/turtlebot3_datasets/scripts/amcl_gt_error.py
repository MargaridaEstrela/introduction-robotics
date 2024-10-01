#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import numpy as np
import tf2_ros
import tf2_geometry_msgs


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

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
            try:
                # Look up the transform from mocap to map frame
                transform = self.tf_buffer.lookup_transform(
                    "map", "mocap", rospy.Time(0), rospy.Duration(1.0)
                )

                # Create a PoseStamped from the ground truth PoseWithCovarianceStamped
                gt_pose_stamped = PoseStamped()
                gt_pose_stamped.header = (
                    self.gt_msg.header
                )  # Copy the header from ground truth message
                gt_pose_stamped.pose = self.gt_msg.pose.pose  # Copy the pose

                # Transform the ground truth pose to the map frame
                transformed_gt_pose_stamped = tf2_geometry_msgs.do_transform_pose(
                    gt_pose_stamped, transform
                )

                # Amcl position in map frame
                amcl_pos = np.array(
                    [
                        self.amcl_msg.pose.pose.position.x,
                        self.amcl_msg.pose.pose.position.y,
                    ]
                )

                # Transformed ground truth position in map frame
                gt_pos = np.array(
                    [
                        transformed_gt_pose_stamped.pose.position.x,
                        transformed_gt_pose_stamped.pose.position.y,
                    ]
                )

                # Calculate Euclidean distance between amcl and transformed ground truth
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

            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logwarn(f"Failed to transform ground truth: {e}")


if __name__ == "__main__":
    EKFErrorPublisher()
    rospy.spin()
