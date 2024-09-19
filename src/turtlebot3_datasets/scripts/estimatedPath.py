#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class EstimatedPathDrawer:
    def __init__(self):
        rospy.init_node("estimated_path_drawer")

        self.path_pub = rospy.Publisher("estimated_path", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "mocap"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber("odometry/filtered", Odometry, self.callback)

    def callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                "mocap", "odom", rospy.Time(0), rospy.Duration(1.0)
            )

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.pose = msg.pose.pose

            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

            self.path.poses.append(transformed_pose)
            self.path_pub.publish(self.path)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn("TF lookup failed: {}".format(e))

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    EstimatedPathDrawer().run()
