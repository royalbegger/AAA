#!/usr/bin/python3
import math

import message_filters
import numpy as np
import rospy
import tf.transformations as tf_trans
import transforms3d.euler as t3d_euler
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker

from navigation_pkg.multi_obj_tracker import MultiObjectTracker


class DynamicObstacleTrackerNode:
    def __init__(self):
        rospy.init_node("dynamic_obstacle_tracker", anonymous=True)

        self.max_range = rospy.get_param("~max_range", 5.0)
        self.map_bound = rospy.get_param("~map_bound", 50.0)
        self.robot_radius = rospy.get_param("~robot_radius", 0.2)
        self.map_resolution = rospy.get_param("~map_resolution", 0.1)
        self.inflation_factor = rospy.get_param("~inflation_factor", 1.2)
        self.dbscan_eps = rospy.get_param("~dbscan_eps", 0.3)
        self.dbscan_min_samples = rospy.get_param("~dbscan_min_samples", 4)
        self.dynamic_speed_threshold = rospy.get_param("~dynamic_speed_threshold", 0.1)
        self.prediction_steps = rospy.get_param("~prediction_steps", 10)
        self.prediction_dt = rospy.get_param("~prediction_dt", 0.2)

        self.centroid_pub = rospy.Publisher("/obstacle_centroids", PoseArray, queue_size=1)
        self.tracker_velocity_pub = rospy.Publisher("/dynamic_obstacle_velocity", PoseArray, queue_size=1)
        self.cluster_marker_pub = rospy.Publisher("/dbscan_centroids", Marker, queue_size=1)
        self.prediction_marker_pub = rospy.Publisher(
            "/dynamic_obstacle_predicted_trajectories",
            Marker,
            queue_size=1,
        )
        self.map_pub = rospy.Publisher("/grid_map", OccupancyGrid, queue_size=10)

        self.mot = MultiObjectTracker()

        self.lidar_sub = message_filters.Subscriber("/front/scan", LaserScan)
        self.odom_sub = message_filters.Subscriber("/odometry/filtered", Odometry)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.odom_sub],
            queue_size=10,
            slop=0.05,
        )
        self.sync.registerCallback(self.sync_callback)

    def sync_callback(self, scan_msg, odom_msg):
        x, y, yaw = self._extract_robot_pose(odom_msg)
        lidar_ranges, local_points = self._extract_scan_points(scan_msg)
        del lidar_ranges

        if len(local_points) < 5:
            self._publish_empty_visuals()
            return

        world_points = self._transform_points(local_points, x, y, yaw)
        grid_map = self.grid_map_from_points(world_points)
        self.publish_grid_map(grid_map)

        detections = self._cluster_detections(world_points)
        self.mot.update(detections)
        self.publish_tracking_results()

    def _extract_robot_pose(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        orientation_q = odom_msg.pose.pose.orientation
        q_ros = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        _, _, yaw = t3d_euler.quat2euler(q_ros, axes="sxyz")
        return x, y, yaw

    def _extract_scan_points(self, scan_msg):
        ranges = np.asarray(scan_msg.ranges, dtype=float)
        ranges[np.isnan(ranges)] = self.max_range
        ranges[np.isinf(ranges)] = self.max_range
        ranges[ranges > self.max_range] = self.max_range

        lidar_ranges = np.flip(ranges.copy())
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        local_points = np.vstack((xs, ys)).T
        local_points = local_points[ranges < self.max_range]
        return lidar_ranges, local_points

    def _transform_points(self, local_points, x, y, yaw):
        rot = np.array(
            [
                [np.cos(yaw), -np.sin(yaw)],
                [np.sin(yaw), np.cos(yaw)],
            ]
        )
        trans = np.array([x, y])
        return (rot @ local_points.T).T + trans

    def _cluster_detections(self, world_points):
        clustering = DBSCAN(
            eps=self.dbscan_eps,
            min_samples=self.dbscan_min_samples,
        ).fit(world_points)
        labels = clustering.labels_

        detections = []
        for label in set(labels):
            if label == -1:
                continue

            cluster_pts = world_points[labels == label]
            if len(cluster_pts) < 3:
                continue

            detections.append(np.mean(cluster_pts, axis=0))
        return detections

    def publish_tracking_results(self):
        now = rospy.Time.now()

        centroid_msg = PoseArray()
        centroid_msg.header.stamp = now
        centroid_msg.header.frame_id = "odom"

        velocity_msg = PoseArray()
        velocity_msg.header.stamp = now
        velocity_msg.header.frame_id = "odom"

        centroid_marker = Marker()
        centroid_marker.header.stamp = now
        centroid_marker.header.frame_id = "odom"
        centroid_marker.ns = "dynamic_objects"
        centroid_marker.id = 0
        centroid_marker.type = Marker.SPHERE_LIST
        centroid_marker.action = Marker.ADD
        centroid_marker.scale.x = 0.3
        centroid_marker.scale.y = 0.3
        centroid_marker.scale.z = 0.3
        centroid_marker.color.r = 1.0
        centroid_marker.color.g = 0.0
        centroid_marker.color.b = 0.0
        centroid_marker.color.a = 1.0

        prediction_marker = Marker()
        prediction_marker.header.stamp = now
        prediction_marker.header.frame_id = "odom"
        prediction_marker.ns = "dynamic_object_predictions"
        prediction_marker.id = 0
        prediction_marker.type = Marker.LINE_LIST
        prediction_marker.action = Marker.ADD
        prediction_marker.scale.x = 0.06
        prediction_marker.color.r = 0.0
        prediction_marker.color.g = 1.0
        prediction_marker.color.b = 1.0
        prediction_marker.color.a = 1.0

        for track_id, state in self.mot.get_tracks():
            del track_id
            px, py, vx, vy, ax, ay = state
            speed = float(np.hypot(vx, vy))
            if speed < self.dynamic_speed_threshold:
                continue

            pose = Pose()
            pose.position.x = px
            pose.position.y = py

            heading = math.atan2(vy, vx)
            qx, qy, qz, qw = tf_trans.quaternion_from_euler(0.0, 0.0, heading)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            centroid_msg.poses.append(pose)

            vel_pose = Pose()
            vel_pose.position.x = vx
            vel_pose.position.y = vy
            vel_pose.position.z = speed
            vel_pose.orientation.x = qx
            vel_pose.orientation.y = qy
            vel_pose.orientation.z = qz
            vel_pose.orientation.w = qw
            velocity_msg.poses.append(vel_pose)

            centroid_marker.points.append(Point(px, py, 0.0))

            trajectory = self.predict_trajectory(px, py, vx, vy, ax, ay)
            for start, end in zip(trajectory[:-1], trajectory[1:]):
                prediction_marker.points.append(Point(start[0], start[1], 0.05))
                prediction_marker.points.append(Point(end[0], end[1], 0.05))

        self.centroid_pub.publish(centroid_msg)
        self.tracker_velocity_pub.publish(velocity_msg)
        self.cluster_marker_pub.publish(centroid_marker)
        self.prediction_marker_pub.publish(prediction_marker)

    def predict_trajectory(self, px, py, vx, vy, ax, ay):
        trajectory = [(px, py)]
        for step in range(1, self.prediction_steps + 1):
            t = step * self.prediction_dt
            pred_x = px + vx * t + 0.5 * ax * t * t
            pred_y = py + vy * t + 0.5 * ay * t * t
            trajectory.append((pred_x, pred_y))
        return trajectory

    def _publish_empty_visuals(self):
        now = rospy.Time.now()

        centroid_msg = PoseArray()
        centroid_msg.header.stamp = now
        centroid_msg.header.frame_id = "odom"

        velocity_msg = PoseArray()
        velocity_msg.header.stamp = now
        velocity_msg.header.frame_id = "odom"

        centroid_marker = Marker()
        centroid_marker.header.stamp = now
        centroid_marker.header.frame_id = "odom"
        centroid_marker.ns = "dynamic_objects"
        centroid_marker.id = 0
        centroid_marker.type = Marker.SPHERE_LIST
        centroid_marker.action = Marker.ADD
        centroid_marker.scale.x = 0.3
        centroid_marker.scale.y = 0.3
        centroid_marker.scale.z = 0.3
        centroid_marker.color.r = 1.0
        centroid_marker.color.a = 1.0

        prediction_marker = Marker()
        prediction_marker.header.stamp = now
        prediction_marker.header.frame_id = "odom"
        prediction_marker.ns = "dynamic_object_predictions"
        prediction_marker.id = 0
        prediction_marker.type = Marker.LINE_LIST
        prediction_marker.action = Marker.ADD
        prediction_marker.scale.x = 0.06
        prediction_marker.color.g = 1.0
        prediction_marker.color.b = 1.0
        prediction_marker.color.a = 1.0

        self.centroid_pub.publish(centroid_msg)
        self.tracker_velocity_pub.publish(velocity_msg)
        self.cluster_marker_pub.publish(centroid_marker)
        self.prediction_marker_pub.publish(prediction_marker)

    def grid_map_from_points(self, world_points):
        size = int(self.map_bound / self.map_resolution)
        grid_map = np.zeros((size, size), dtype=np.int8)
        offset = int(0.5 * self.map_bound / self.map_resolution)

        for point in world_points:
            col = int(point[0] / self.map_resolution) + offset
            row = int(point[1] / self.map_resolution) + offset
            if 0 <= row < size and 0 <= col < size:
                grid_map[row, col] = 1

        inflation_radius = int(
            (self.inflation_factor * self.robot_radius) / self.map_resolution
        )
        inflated_map = grid_map.copy()
        obstacle_indices = np.argwhere(grid_map == 1)

        for row, col in obstacle_indices:
            for d_row in range(-inflation_radius, inflation_radius + 1):
                for d_col in range(-inflation_radius, inflation_radius + 1):
                    next_row = row + d_row
                    next_col = col + d_col
                    if 0 <= next_row < size and 0 <= next_col < size:
                        if math.hypot(d_row, d_col) <= inflation_radius:
                            inflated_map[next_row, next_col] = 1

        return inflated_map

    def publish_grid_map(self, grid_data):
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "odom"
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = grid_data.shape[1]
        grid_msg.info.height = grid_data.shape[0]

        origin = Pose()
        origin.position.x = -0.5 * self.map_bound
        origin.position.y = -0.5 * self.map_bound
        origin.position.z = 0.0
        origin.orientation.w = 1.0
        grid_msg.info.origin = origin

        grid_msg.data = (grid_data * 100).flatten().tolist()
        self.map_pub.publish(grid_msg)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        DynamicObstacleTrackerNode().run()
    except rospy.ROSInterruptException:
        pass
