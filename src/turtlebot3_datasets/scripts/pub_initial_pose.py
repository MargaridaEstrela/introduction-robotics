#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped


def publish_initial_pose():
    rospy.init_node("initial_pose_publisher")

    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

    # Create initial pose
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.pose.pose.position.x = -1.9880502143006074
    initial_pose.pose.pose.position.y = -0.4673281598919172
    initial_pose.pose.pose.position.z = 0.0
    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = -0.0017424146329955382
    initial_pose.pose.pose.orientation.w = 0.9999984819944712

    initial_pose.pose.covariance = [
        0.1615181128390335,
        -0.015028723797889509,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.01502872379788962,
        0.20050510873591773,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.059872828965145085,
        0.0,
        0.0,
        0.0,
        0.0,
    ]

    rospy.sleep(1)
    pub.publish(initial_pose)

    rospy.spin()


if __name__ == "__main__":
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
