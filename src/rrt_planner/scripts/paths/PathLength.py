#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path


class PathLength:
    def __init__(self):
        rospy.init_node('path_length')
        rospy.Subscriber('/move_base/RRTPlannerROS/global_plan', Path, self.path_callback)
        self.path_length = 0.0

    def distance(self, point1, point2):
        return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2)**0.5

    def path_callback(self, msg):
        poses = msg.poses
        dist = 0.0

        for i in range(0, len(poses)-1):
            dist += self.distance(poses[i].pose.position, poses[i+1].pose.position)

        self.path_length += dist
        rospy.loginfo(f"New Path Length: {dist:.3f} with {len(poses)} nodes")
        rospy.loginfo(f"Path Length Until Now: {self.path_length:.3f}")

    def run(self):
        rospy.spin()
        rospy.loginfo("Total Path length: " + str(self.path_length))


if __name__ == '__main__':
    path_length = PathLength()
    path_length.run()
