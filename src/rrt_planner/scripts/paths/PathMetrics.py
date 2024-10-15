#!/usr/bin/env python

import rospy
import math
import numpy as np

from utils import TimeMsg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from scipy.interpolate import interp1d


class PathMetrics:
    def __init__(self):
        rospy.init_node("path_metrics")

        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_callback)
        rospy.Subscriber(
            "/move_base/RRTPlannerROS/global_plan", Path, self.path_callback
        )
        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback
        )

        # Initialize path metrics
        self.path_length_rrt = 0.0
        self.path_length_amcl = 0.0
        self.current_pose = None
        self.amcl_poses = []
        self.amcl = []
        self.rrt_poses = None
        self.current_goal = None
        self.discard_distance = 0.0
        self.first_amcl_path = True

        # Track goals
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.3)
        self.current_goal_start_time = None
        self.first_timestamp = None
        self.start_time = None
        self.rrt_paths_count = 0
        self.total_rrt_paths_count = 0
        self.goal_reached = False
        self.goals_reached_count = 0

        # Time metrics for each goal
        self.time_to_goal = []
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

    def check_goal_reached(self, current_pose):
        """
        Checks if the current goal has been reached within the goal tolerance.
        """
        distance_to_goal = self.distance_to_goal(current_pose)
        return distance_to_goal <= self.goal_tolerance

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

    def subsample_poses(self, poses, num_points):
        """
        Subsamples a path to have the specified number of points.
        """
        if len(poses) <= num_points:
            return poses

        indices = np.linspace(0, len(poses) - 1, num_points, dtype=int)
        return [poses[i] for i in indices]

    def path_smoothness(self, poses, target_num_points=None):
        """
        Computes the angular changes between consecutive segments of the path and
        returns a value representing the smoothness of the path. A lower value
        indicates a smoother path, while a higher value indicates more frequent
        or sharper turns.
        """
        if target_num_points and len(poses) > 2:
            poses = self.interpolate_poses(poses, target_num_points)

        angles = []

        for i in range(1, len(poses) - 1):
            # Get the positions of three consecutive poses (p0, p1, p2)
            p0 = poses[i - 1].pose.position
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position
            v1 = [p1.x - p0.x, p1.y - p0.y]
            v2 = [p2.x - p1.x, p2.y - p1.y]
            angle = math.atan2(v2[1], v2[0]) - math.atan2(v1[1], v1[0])
            angles.append(angle)

        # A lower value indicates a smoother path
        return (
            math.sqrt(sum([a**2 for a in angles]) / len(angles))
            if len(angles) > 0
            else 0.0
        )

    def average_deviation(self, rrt_poses, amcl_poses):
        """
        Determine the minimum length between the two sets of poses (RRT and AMCL)
        """
        min_len = min(len(rrt_poses), len(amcl_poses))
        deviation_sum = 0.0

        # Loop through the poses up to the minimum length to compare corresponding points
        for i in range(min_len):
            point_rrt = rrt_poses[i].pose.position
            point_amcl = amcl_poses[i].pose.position
            deviation_sum += math.sqrt(
                (point_rrt.x - point_amcl.x) ** 2 + (point_rrt.y - point_amcl.y) ** 2
            )
        return deviation_sum / min_len if min_len > 0 else 0.0

    def get_metrics(self):
        dist_rrt = self.distance(self.rrt_poses)

        dist_amcl = self.distance(self.amcl_poses)
        dist_amcl_adjusted = dist_amcl - self.discard_distance

        # num_points = min(len(self.rrt_poses), len(self.amcl_poses))

        # rrt_interpolated = self.interpolate_poses(
        #     self.rrt_poses, num_points, pose_type="PoseStamped"
        # )
        # amcl_interpolated = self.interpolate_poses(
        #     self.amcl, num_points, pose_type="PoseWithCovarianceStamped"
        # )

        # amcl_subsampled = self.subsample_poses(self.amcl_poses, num_points)
        # rrt_subsampled = self.subsample_poses(self.rrt_poses, num_points)

        smoothness_rrt = self.path_smoothness(self.rrt_poses)
        smoothness_amcl = self.path_smoothness(self.amcl_poses)

        deviation = self.average_deviation(self.rrt_poses, self.amcl_poses)

        if self.rrt_paths_count == 1:
            rospy.loginfo(
                f"Goal {self.current_waypoint_index} reached on the first try!"
            )
        else:
            rospy.loginfo(
                f"Goal {self.current_waypoint_index} reached after replanning."
            )

        # Reset distance tracking for the next segment
        self.discard_distance = dist_amcl
        self.rrt_paths_count = 0
        self.goal_reached = False

        rospy.loginfo(f"AMCL Path Length: {dist_amcl_adjusted:.3f} meters")
        # rospy.loginfo(f"AMCL Path Smoothness: {smoothness_amcl:.3f}")
        # rospy.loginfo(
        #     f"Average Deviation between RRT and AMCL paths: {deviation:.3f} meters"
        # )

        rospy.loginfo(f"RRT Path Length: {dist_rrt:.3f} meters")
        # rospy.loginfo(f"RRT Path Smoothness: {smoothness_rrt:.3f}")

        rospy.loginfo("################################")

    def goal_callback(self, msg):
        """
        Callback to update the current goal from move_base/current_goal
        """
        self.current_goal = msg.pose
        self.current_waypoint_index += 1
        rospy.loginfo(
            f"New goal received: {self.current_goal.position.x}, {self.current_goal.position.y}"
        )

    def amcl_pose_callback(self, msg):
        """
        Handle AMCL pose updates and check if the goal has been reached.
        """

        msg_time = self.normalize_time(TimeMsg(msg))
        self.current_pose = msg.pose.pose

        if self.current_goal is None:
            return

        # Set the start time when the first AMCL update is received after a new goal is set
        if self.current_goal_start_time is None:
            self.current_goal_start_time = msg_time
            rospy.loginfo(
                f"Distance to goal: {self.distance_to_goal(self.current_pose):.3f} meters"
            )

        # Set the end time when the goal is reached
        elif self.check_goal_reached(self.current_pose):
            self.goal_reached = True
            elapsed_time = msg_time - self.current_goal_start_time
            self.time_to_goal.append(elapsed_time)
            rospy.loginfo(f"Goal reached! Time to goal: {elapsed_time:.2f} seconds")
            self.current_goal = None
            self.current_goal_start_time = None

            self.get_metrics()
            self.goals_reached_count += 1

        self.amcl_poses.append(msg.pose)
        self.amcl.append(msg)

    def path_callback(self, msg):
        """
        Handle RRT path updates and calculate metrics.
        """
        self.rrt_poses = msg.poses

        dist_rrt = self.distance(self.rrt_poses)
        self.rrt_paths_count += 1
        self.total_rrt_paths_count += 1

        # If this is not the first path for the current goal, mark it as a reattempt
        if self.rrt_paths_count > 1:
            rospy.loginfo(
                f"Re-planning attempt for waypoint {self.current_waypoint_index}"
            )
            if self.current_pose:
                closest_pose_idx, min_distance = self.get_closest_pose_idx(
                    self.previous_path, self.current_pose
                )
                if closest_pose_idx is not None:
                    # Append the remaining part of the previous path to the new path
                    previous_path_dist = self.distance(self.previous_path)
                    distance_to_remove = previous_path_dist - min_distance

                    # Add the remaining previous path length to the total distance
                    dist_rrt -= distance_to_remove

                    # rospy.loginfo(
                    #     f"Adding {min_distance:.3f} meters from previous path to new path"
                    # )

        # Update total RRT path length
        self.path_length_rrt += dist_rrt
        self.previous_path = self.rrt_poses

    def get_closest_pose_idx(self, path, current_pose):
        """Find the index of the closest pose in the path to the robot's current position."""
        closest_idx = None
        min_distance = float("inf")
        for i, pose_stamped in enumerate(path):
            dist = self.distance_between_poses(pose_stamped.pose, current_pose)
            if dist < min_distance:
                min_distance = dist
                closest_idx = i
        rospy.loginfo(
            f"Closest point found at index {closest_idx} with distance {min_distance:.3f}"
        )
        return closest_idx, min_distance

    def run(self):
        rospy.spin()
        rospy.loginfo(
            f"Final AMCL Path Length: {self.distance(self.amcl_poses):.3f} meters"
        )
        rospy.loginfo(f"Final RRT Path Length: {self.path_length_rrt:.3f} meters")
        rospy.loginfo(f"Total RRT Paths: {self.total_rrt_paths_count}")
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
