#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker


def publish_markers():
    rospy.init_node('waypoint_marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    # Define the waypoints
    waypoints = [
        [-2.0, 0.7],
        [-0.1, 1.35],
        [1.043, -3.328],
        [1.363, -0.479]
    ]

    # Create a marker for each waypoint
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "waypoints"
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 0.2  # Size of the marker in x direction
    marker.scale.y = 0.2  # Size of the marker in y direction
    marker.scale.z = 0.2  # Size of the marker in z direction
    marker.color.a = 1.0  # Opacity of the marker
    marker.color.r = 1.0  # Red color intensity
    marker.color.g = 0.0  # Green color intensity
    marker.color.b = 0.0  # Blue color intensity

    # Publish markers at each waypoint
    for idx, waypoint in enumerate(waypoints):
        rospy.sleep(1)  # Sleep for a second between publishing markers
        marker.id = idx  # Unique ID for each marker
        marker.pose.position.x = waypoint[0]
        marker.pose.position.y = waypoint[1]
        marker.pose.position.z = 0.0  # Assume the marker is placed on a flat plane
        marker.pose.orientation.w = 1.0  # Default orientation
        # Publish the marker
        marker_pub.publish(marker)
        rospy.loginfo(f"Published marker at waypoint {waypoint}")


if __name__ == '__main__':
    try:
        publish_markers()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
