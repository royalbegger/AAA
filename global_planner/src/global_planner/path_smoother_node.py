#!/usr/bin/env python3
# coding=utf-8

import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from global_planner.gauss_smooth_3d import gauss_smooth_3d


class PathSmootherNode:

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/path")
        self.output_topic = rospy.get_param("~output_topic", "/snooth_path")
        self.kernel_size = rospy.get_param("~kernel_size", 7)
        self.num_scale = rospy.get_param("~num_scale", 3)
        self.sigma = rospy.get_param("~sigma", 1.0)
        self.min_points = rospy.get_param("~min_points", 2)

        self.smoother = gauss_smooth_3d(self.kernel_size, self.num_scale)
        self.path_pub = rospy.Publisher(self.output_topic, Path, queue_size=10)
        self.path_sub = rospy.Subscriber(self.input_topic, Path, self.path_callback, queue_size=10)

        rospy.loginfo("Path smoother listening on %s and publishing to %s",
                      self.input_topic, self.output_topic)

    def path_callback(self, path_msg):
        if len(path_msg.poses) < self.min_points:
            self.path_pub.publish(path_msg)
            return

        traj_pts = np.array([
            [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            for pose in path_msg.poses
        ], dtype=float)

        try:
            smoothed_pts = self.smoother.smooth_3d_trajectory(traj_pts, sigma=self.sigma)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "Failed to smooth path: %s", str(exc))
            self.path_pub.publish(path_msg)
            return

        smoothed_path = Path()
        smoothed_path.header = path_msg.header

        for idx, point in enumerate(smoothed_pts):
            pose_stamped = PoseStamped()
            pose_stamped.header = smoothed_path.header
            pose_stamped.pose.position.x = float(point[0])
            pose_stamped.pose.position.y = float(point[1])
            pose_stamped.pose.position.z = float(point[2]) if len(point) > 2 else 0.0

            yaw = self.compute_yaw(smoothed_pts, idx)
            pose_stamped.pose.orientation.z = math.sin(yaw * 0.5)
            pose_stamped.pose.orientation.w = math.cos(yaw * 0.5)
            smoothed_path.poses.append(pose_stamped)

        if path_msg.poses:
            smoothed_path.poses[-1].pose.orientation = path_msg.poses[-1].pose.orientation

        self.path_pub.publish(smoothed_path)

    @staticmethod
    def compute_yaw(points, index):
        if len(points) < 2:
            return 0.0

        if index < len(points) - 1:
            dx = points[index + 1][0] - points[index][0]
            dy = points[index + 1][1] - points[index][1]
        else:
            dx = points[index][0] - points[index - 1][0]
            dy = points[index][1] - points[index - 1][1]

        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return 0.0

        return math.atan2(dy, dx)


def main():
    rospy.init_node("path_smoother_node")
    PathSmootherNode()
    rospy.spin()


if __name__ == "__main__":
    main()
