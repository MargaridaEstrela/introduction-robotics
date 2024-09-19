#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseGroundTruth:
    def __init__(self):
        rospy.init_node("PoseroundTruth")
        self.prevTime = -1
        self.pub = rospy.Publisher("ground_truth", Marker, queue_size=10)
        self.pub_pose = rospy.Publisher(
            "pose_gt", PoseWithCovarianceStamped, queue_size=10
        )
        rospy.Subscriber("/tf", TFMessage, self.Callback)

    def PublishMarker(self, msg):
        marker = Marker()
        marker.header.frame_id = "mocap"
        marker.header.stamp = msg.header.stamp
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = msg.transform.translation
        marker.pose.orientation = msg.transform.rotation
        # scale
        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.05, 0.1
        # color
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
            0,
            0.8,
            0.3,
            0.8,
        )

        self.pub.publish(marker)

    def PublishPose(self, msg):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "mocap"
        pose.header.stamp = msg.header.stamp
        pose.pose.pose.position = msg.transform.translation
        pose.pose.pose.orientation = msg.transform.rotation
        self.pub_pose.publish(pose)

    def Callback(self, msg):
        for msg in msg.transforms:
            if msg.child_frame_id == "mocap_laser_link":
                currentTime = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
                if currentTime - self.prevTime >= 1:
                    self.PublishMarker(msg)
                    self.PublishPose(msg)
                    self.prevTime = currentTime

    def initialize(self):
        rospy.spin()


if __name__ == "__main__":
    PoseGroundTruth().initialize()
