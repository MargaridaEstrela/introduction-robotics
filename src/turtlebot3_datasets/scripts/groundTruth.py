#!/usr/bin/env python
import rospy
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


class GroundTruth:
    def __init__(self):
        rospy.init_node('GroundTruth')
        self.prevTime = -1
        self.pub = rospy.Publisher('ground', Marker, queue_size=10)
        rospy.Subscriber('/tf', TFMessage, self.Callback)

    def DefineMarker(self, msg):
        marker = Marker()
        marker.header.frame_id = "mocap"
        marker.header.stamp = msg.header.stamp
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = msg.transform.translation
        marker.pose.orientation = msg.transform.rotation
        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.1
        marker.color.r = 0
        marker.color.g = 0.8
        marker.color.b = 0.3
        marker.color.a = 0.8
        return marker

    def Callback(self, msg):
        for msg in msg.transforms:
            if msg.child_frame_id == "mocap_laser_link":
                currentTime = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
                if currentTime - self.prevTime >= 1:
                    marker = self.DefineMarker(msg)
                    self.prevTime = currentTime
                    self.pub.publish(marker)

    def initialize(self):
        rospy.spin()


if __name__ == '__main__':
    GroundTruth().initialize()
