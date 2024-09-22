#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import TimeMsg

TIME_INTERVAL = 0.2


class PathDrawer:
    def __init__(self):
        self.prevTime = -1
        rospy.init_node("path_drawer")
        self.path_pub = rospy.Publisher("amcl_path", Path, queue_size=10)
        self.path = Path()
        self.path.header.frame_id = "map"
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.callback)

    def callback(self, msg):
        currentTime = TimeMsg(msg)
        if currentTime - self.prevTime > TIME_INTERVAL:
            self.prevTime = currentTime
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = msg.header.stamp
            pose.pose = msg.pose.pose

            self.path.poses.append(pose)
            self.path_pub.publish(self.path)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    PathDrawer().run()
