#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped


class PathDrawer:
    def __init__(self):
        rospy.init_node("path_drawer")
        self.path_pub = rospy.Publisher("path_gt", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "mocap"

        rospy.Subscriber("pose_gt", PoseWithCovarianceStamped, self.callback)

    def callback(self, msg):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "mocap"
        pose.header.stamp = msg.header.stamp
        pose.pose = msg.pose.pose

        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    PathDrawer().run()
