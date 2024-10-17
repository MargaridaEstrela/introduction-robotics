#!/usr/bin/env python

import rospy
import math
import numpy as np

from utils import TimeMsg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult

from scipy.interpolate import interp1d


class PathMetrics:
    def __init__(self):
        rospy.init_node("path_metrics")

        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)
        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback
        )
        rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self.goal_result_callback
        )

        # Initialize variables
        self.current_pose = None
        self.current_goal = None
        self.first_timestamp = None
        self.current_goal_start_time = None

        # Track Navigation paths
        self.dist_nav = 0
        self.nav_poses = None
        self.nav_paths_count = 0
        self.path_length_nav = 0.0
        self.total_nav_paths_count = 0

        # Track AMCL path
        self.amcl = []
        self.amcl_poses = []
        self.path_length_amcl = 0.0
        self.discard_distance = 0.0

        # Track goals
        self.time_to_goal = []
        self.goal_reached = False
        self.goals_reached_count = 0
        self.current_waypoint_index = 0

    def normalize_time(self, timestamp):
        """
        Normalize the timestamp by subtracting the first timestamp.
        """
        if self.first_timestamp is None:
            self.first_timestamp = timestamp  # Set the first valid timestamp
        return timestamp - self.first_timestamp

    def distance_between_poses(self, pose1, pose2):
        """Calculate the Euclidean distance between two poses."""
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2
            + (pose1.position.y - pose2.position.y) ** 2
        )

    def distance(self, poses):
        dist = 0.0
        for i in range(len(poses) - 1):
            point1 = poses[i].pose.position
            point2 = poses[i + 1].pose.position
            dist += math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2)
        return dist

    def distance_to_goal(self, current_pose):
        """
        Calculates distance from the current position to the current goal.
        """
        if self.current_goal is None:
            return float("inf")  # No goal set

        # Access position directly from self.current_goal
        goal_x = self.current_goal.position.x
        goal_y = self.current_goal.position.y

        # Access position from current_pose (which is already Pose)
        current_x = current_pose.position.x
        current_y = current_pose.position.y

        # Calculate the Euclidean distance to the goal
        return math.sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)

    def interpolate_poses(self, poses, num_points, pose_type="PoseStamped"):
        """
        Linearly interpolate between the poses to ensure a uniform number of points.
        Handles both PoseStamped and PoseWithCovarianceStamped types.
        """
        if pose_type == "PoseStamped":
            x = [pose.pose.position.x for pose in poses]
            y = [pose.pose.position.y for pose in poses]
        elif pose_type == "PoseWithCovarianceStamped":
            x = [pose.pose.pose.position.x for pose in poses]
            y = [pose.pose.pose.position.y for pose in poses]

        # Create interpolation functions for x and y
        t_original = np.linspace(0, 1, len(poses))
        t_interpolated = np.linspace(0, 1, num_points)

        interp_x = interp1d(t_original, x, kind="linear")
        interp_y = interp1d(t_original, y, kind="linear")

        # Generate interpolated points
        interpolated_x = interp_x(t_interpolated)
        interpolated_y = interp_y(t_interpolated)

        # Create new list of interpolated poses
        interpolated_poses = []
        for i in range(num_points):
            if pose_type == "PoseStamped":
                new_pose = PoseStamped()  # Create a new PoseStamped
                new_pose.header = poses[0].header  # Copy header from original
                new_pose.pose.position.x = interpolated_x[i]
                new_pose.pose.position.y = interpolated_y[i]
                new_pose.pose.position.z = poses[0].pose.position.z  # Keep z value
                # Optionally copy orientation from original pose if needed
                new_pose.pose.orientation = poses[0].pose.orientation
                interpolated_poses.append(new_pose)
            elif pose_type == "PoseWithCovarianceStamped":
                new_pose = (
                    PoseWithCovarianceStamped()
                )  # Create a new PoseWithCovarianceStamped
                new_pose.header = poses[0].header  # Copy header from original
                new_pose.pose.pose.position.x = interpolated_x[i]
                new_pose.pose.pose.position.y = interpolated_y[i]
                new_pose.pose.pose.position.z = poses[
                    0
                ].pose.pose.position.z  # Keep z value
                # Optionally copy orientation and covariance from the original pose
                new_pose.pose.pose.orientation = poses[0].pose.pose.orientation
                new_pose.pose.covariance = poses[
                    0
                ].pose.covariance  # Copy covariance (if needed)
                interpolated_poses.append(new_pose.pose)

        return interpolated_poses

    def path_smoothness(self, poses, target_num_points=None):
        """
        Computes the angular changes between consecutive segments of the path and
        returns a value representing the smoothness of the path. A lower value
        indicates a smoother path, while a higher value indicates more frequent
        or sharper turns.
        """

        curvatures = []
        for i in range(1, len(poses) - 1):
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            # Compute distances between consecutive points
            d01 = math.sqrt((p1.x - p0.x) ** 2 + (p1.y - p0.y) ** 2)
            d12 = math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)
            d02 = math.sqrt((p2.x - p0.x) ** 2 + (p2.y - p0.y) ** 2)

            # Calculate the curvature using the formula:
            # curvature = 4 * (area of triangle formed by p0, p1, p2) / (d01 * d12 * d02)
            # Area of the triangle = 0.5 * |p0.x(p1.y - p2.y) + p1.x(p2.y - p0.y) + p2.x(p0.y - p1.y)|
            area = (
                abs(p0.x * (p1.y - p2.y) + p1.x * (p2.y - p0.y) + p2.x * (p0.y - p1.y))
                / 2.0
            )

            if d01 * d12 * d02 != 0:
                curvature = 4 * area / (d01 * d12 * d02)
            else:
                curvature = 0

            curvatures.append(curvature)

        return sum(curvatures) / len(curvatures) if curvatures else 0.0

    def average_deviation(self, nav_poses, amcl_poses):
        """
        Determine the minimum length between the two sets of poses (nav and AMCL)
        """
        min_len = min(len(nav_poses), len(amcl_poses))
        deviation_sum = 0.0

        # Loop through the poses up to the minimum length to compare corresponding points
        for i in range(min_len):
            point_nav = nav_poses[i].pose.position
            point_amcl = amcl_poses[i].pose.position
            deviation_sum += math.sqrt(
                (point_nav.x - point_amcl.x) ** 2 + (point_nav.y - point_amcl.y) ** 2
            )
        return deviation_sum / min_len if min_len > 0 else 0.0

    def get_metrics(self):
        self.path_length_nav += self.dist_nav
        dist_amcl = self.distance(self.amcl_poses)
        dist_amcl_adjusted = dist_amcl - self.discard_distance

        rospy.loginfo(f"Nav Path Points: {len(self.nav_poses)}")
        rospy.loginfo(f"AMCL Path Points: {len(self.amcl_poses)}")

        if self.nav_paths_count == 1:
            rospy.loginfo(
                f"Goal {self.current_waypoint_index} reached on the first try!"
            )
        else:
            rospy.loginfo(
                f"Goal {self.current_waypoint_index} reached after replanning."
            )

        num_points = min(len(self.nav_poses), len(self.amcl_poses))

        nav_interpolated = self.interpolate_poses(
            self.nav_poses, num_points, pose_type="PoseStamped"
        )
        amcl_interpolated = self.interpolate_poses(
            self.amcl, num_points, pose_type="PoseWithCovarianceStamped"
        )

        smoothness_nav = self.path_smoothness(self.nav_poses)
        smoothness_amcl = self.path_smoothness(self.amcl_poses)

        deviation = self.average_deviation(nav_interpolated, amcl_interpolated)

        rospy.loginfo(f"Nav Path Length: {self.dist_nav:.3f} meters")
        rospy.loginfo(f"Nav Path Smoothness: {smoothness_nav:.3f}")

        rospy.loginfo(f"AMCL Path Length: {dist_amcl_adjusted:.3f} meters")
        rospy.loginfo(f"AMCL Path Smoothness: {smoothness_amcl:.3f}")
        rospy.loginfo(
            f"Average Deviation between Nav and AMCL paths: {deviation:.3f} meters"
        )

        # Reset status for the next goal
        self.discard_distance = dist_amcl
        self.nav_paths_count = 0
        self.dist_nav = 0
        self.goal_reached = False
        self.current_goal = None
        self.current_goal_start_time = None

        rospy.loginfo("################################")

    def goal_callback(self, msg):
        """
        Callback to update the current goal from move_base/current_goal
        """
        self.current_goal = msg.goal.target_pose.pose
        self.current_waypoint_index += 1
        self.current_goal_start_time = TimeMsg(msg)
        rospy.loginfo(
            f"New goal received: {self.current_goal.position.x}, {self.current_goal.position.y}"
        )
        rospy.loginfo(
            f"Distance to goal: {self.distance_to_goal(self.current_pose):.3f} meters"
        )

    def goal_result_callback(self, msg):
        """
        Handle the result from move_base to check if the goal was reached.
        """
        # Check if the goal was reached successfully
        if msg.status.status == 3:  # Status 3 corresponds to "Goal reached"
            self.goal_reached = True
            self.goals_reached_count += 1
            elapsed_time = TimeMsg(msg) - self.current_goal_start_time
            self.time_to_goal.append(elapsed_time)
            rospy.loginfo(
                f"Goal reached successfully! Time to goal: {elapsed_time:.2f} seconds"
            )
            self.get_metrics()

    def amcl_pose_callback(self, msg):
        """
        Handle AMCL pose updates
        """

        self.current_pose = msg.pose.pose
        self.amcl_poses.append(msg.pose)
        self.amcl.append(msg)

    def path_callback(self, msg):
        """
        Handle nav path updates and calculate metrics.
        When replanning occurs, add only the part of the previous path that has already been traversed.
        """
        self.nav_poses = msg.poses

        # If this is not the first path for the current goal, process the replanning
        if self.nav_paths_count > 0:
            if self.current_pose:
                # Find the closest pose in the previous path to the current robot's position
                closest_pose_idx, min_distance = self.get_closest_pose_idx(
                    self.previous_path, self.current_pose
                )
                if closest_pose_idx is not None:
                    self.dist_nav += min_distance  # Add the distance traversed so far
                    rospy.loginfo(
                        f"Adding {min_distance:.3f} meters of traversed path."
                    )
                else:
                    rospy.logwarn(
                        "Couldn't find a close pose, adding entire previous path."
                    )
                    self.dist_nav += self.distance(self.previous_path)
            else:
                rospy.logwarn(
                    "Current pose not available, adding full previous path distance."
                )
                self.dist_nav += self.distance(self.previous_path)
        else:
            # For the first path, add the entire path distance
            self.dist_nav += self.distance(self.nav_poses)

        rospy.loginfo(f"Accumulated Nav Path Distance: {self.dist_nav:.3f} meters")

        # Update path tracking variables
        self.nav_paths_count += 1
        self.total_nav_paths_count += 1
        self.previous_path = self.nav_poses

    def get_closest_pose_idx(self, path, current_pose):
        """
        Find the index of the closest pose in the previous path to the robot's current position.
        This helps calculate how far the robot has traveled on the previous path.
        """
        closest_idx = None
        min_distance = float("inf")
        for i, pose_stamped in enumerate(path):
            dist = self.distance_between_poses(pose_stamped.pose, current_pose)
            if dist < min_distance:
                min_distance = dist
                closest_idx = i
        rospy.loginfo(
            f"Closest point in previous path found at index {closest_idx} with distance {min_distance:.3f}"
        )
        return closest_idx, min_distance

    def run(self):
        rospy.spin()
        rospy.loginfo(
            f"Final AMCL Path Length: {self.distance(self.amcl_poses):.3f} meters"
        )
        rospy.loginfo(f"Final Nav Path Length: {self.path_length_nav:.3f} meters")
        rospy.loginfo(f"Total Nav Paths: {self.total_nav_paths_count}")
        rospy.loginfo(f"Total waypoints: {self.current_waypoint_index}")
        rospy.loginfo(f"Waypoints Reached: {self.goals_reached_count}")

        if self.time_to_goal:
            rospy.loginfo(
                f"Average Time to Goal: {sum(self.time_to_goal) / len(self.time_to_goal):.2f} seconds"
            )
            rospy.loginfo(f"Fastest Time to Goal: {min(self.time_to_goal):.2f} seconds")
            rospy.loginfo(f"Slowest Time to Goal: {max(self.time_to_goal):.2f} seconds")


if __name__ == "__main__":
    path_metrics = PathMetrics()
    path_metrics.run()
