#!/usr/bin/env python
import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Navigate():
    def __init__(self):
        rospy.init_node('navigate')
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def move_to_goal(self, objective):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = objective[0]
        goal.target_pose.pose.position.y = objective[1]
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.client.wait_for_result()
        return self.client.get_result()

    def run(self):
        waypoints = rospy.get_param('~waypoints', [])
        for waypoint in waypoints:
            rospy.sleep(1)
            result = self.move_to_goal(waypoint)
            rospy.loginfo(result)


if __name__ == '__main__':
    nav = Navigate()
    nav.run()
