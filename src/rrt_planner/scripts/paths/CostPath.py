#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import matplotlib.pyplot as plt


class PathLength:
    def __init__(self):
        rospy.init_node('path_length')
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.save_map)
        # rospy.Subscriber('/move_base/RRTPlannerROS/global_plan', Path, self.path_callback)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.path_callback)

        self.map = None

    def save_map(self, msg):
        rospy.loginfo(f"Map received {msg.info.width}x{msg.info.height}")
        width, height = msg.info.width, msg.info.height
        self.res = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map = np.array(msg.data).reshape((height, width)).T

    def path_callback(self, msg):
        poses = msg.poses
        num_poses = 0
        cost = 0
        for pose in poses:
            x, y = self.WorldToMap(pose.pose.position.x, pose.pose.position.y)
            if x:  # If the point is inside the map
                cost += self.map[x, y]
                num_poses += 1

        mean_cost = cost / num_poses
        rospy.loginfo(f"Mean cost of the path: {mean_cost}")

    def WorldToMap(self, wx, wy):
        if self.map is not None:
            if not (wx < self.origin_x or wy < self.origin_y):
                mx = int((wx - self.origin_x) / self.res)
                my = int((wy - self.origin_y) / self.res)

                # map.shape -> (Width, Height)
                if mx < self.map.shape[0] and my < self.map.shape[1]:
                    return mx, my

        return None, None

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    path_length = PathLength()
    path_length.run()
