#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path


class PathLength:
    def __init__(self):
        rospy.init_node('path_length')
        rospy.Subscriber('/move_base/RRTPlannerROS/global_plan', Path, self.path_callback)
        rospy.Subscriber('/amcl_path', Path, self.path_amcl_callback)
        self.path_length_rrt = 0.0
        self.amcl_poses = None
        self.discard_distance = 0.0

    def distance(self, poses):
        dist = 0.0
        for i in range(0, len(poses)-1):
            point1 = poses[i].pose.position
            point2 = poses[i+1].pose.position
            dist += ((point1.x - point2.x)**2 + (point1.y - point2.y)**2)**0.5
        return dist

    def path_callback(self, msg):
        poses = msg.poses

        # Calculate the path length of the rrt planner
        dist_rrt = self.distance(poses)

        # Calculate the path length using amcl poses
        if self.amcl_poses:
            self.dist_amcl = self.distance(self.amcl_poses)

        # Our rrt didn't send its initial path, so we discard the first path of the amcl to compare
        if self.path_length_rrt == 0.0:
            self.discard_distance = self.dist_amcl

        rospy.loginfo(f"Path Length using amcl poses: {(self.dist_amcl-self.discard_distance):.3f}")
        rospy.loginfo(f"Path Length Until Now rrt planner: {self.path_length_rrt:.3f}")

        # Only updated after printing to compare the two paths lengths
        self.path_length_rrt += dist_rrt

        rospy.loginfo(f"New Path Length: {dist_rrt:.3f} with {len(poses)} nodes\n")

    def path_amcl_callback(self, msg):
        self.amcl_poses = msg.poses

    def run(self):
        rospy.spin()
        rospy.loginfo(f"Total Path length rrt planner: {self.path_length_rrt:.3f}")
        rospy.loginfo(f"Total path using amcl poses: {(self.dist_amcl-self.discard_distance):.3f}")


if __name__ == '__main__':
    path_length = PathLength()
    path_length.run()
