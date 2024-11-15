#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from utils import TimeMsg

TIME_INTERVAL_GT = 1


class PoseGroundTruth:
    def __init__(self):
        rospy.init_node("PoseroundTruth")
        self.prevTime = -1
        self.pub_gt = rospy.Publisher(
            "ground_truth", PoseWithCovarianceStamped, queue_size=10)
        self.pub_marker = rospy.Publisher("marker_gt", Marker, queue_size=10)
        self.pub_pose = rospy.Publisher(
            "pose_gt", PoseWithCovarianceStamped, queue_size=10)
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

        self.pub_marker.publish(marker)

    def PublishPose(self, msg, publisher):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "mocap"
        pose.header.stamp = msg.header.stamp
        pose.pose.pose.position = msg.transform.translation
        pose.pose.pose.orientation = msg.transform.rotation
        publisher.publish(pose)

    def Callback(self, msg):
        for msg in msg.transforms:
            if msg.child_frame_id == "mocap_laser_link":
                currentTime = TimeMsg(msg)
                if currentTime - self.prevTime >= TIME_INTERVAL_GT:
                    self.PublishPose(msg, self.pub_pose)
                    self.PublishMarker(msg)
                    self.prevTime = currentTime

                self.PublishPose(msg, self.pub_gt)

    def initialize(self):
        rospy.spin()


if __name__ == "__main__":
    PoseGroundTruth().initialize()
