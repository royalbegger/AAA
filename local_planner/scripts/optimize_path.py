#!/usr/bin/python3
import math

import rospy
import tf2_ros
import tf.transformations as t3d
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class PurePursuitNode:
    def __init__(self):
        rospy.init_node('pure_pursuit_node', anonymous=True)

        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.path_topic = rospy.get_param('~path_topic', '/path')
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.lookahead_distance = rospy.get_param('~lookahead_distance', 0.8)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.25)
        self.linear_velocity = rospy.get_param('~linear_velocity', 0.45)
        self.min_linear_velocity = rospy.get_param('~min_linear_velocity', 0.08)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 1.5)
        self.rotate_in_place_angle = math.radians(rospy.get_param('~rotate_in_place_angle_deg', 70.0))
        self.rotate_kp = rospy.get_param('~rotate_kp', 1.8)

        self.current_position = (0.0, 0.0)
        self.current_heading = 0.0
        self.has_pose = False

        self.world_path_poses = []
        self.path_cumulative_distances = []
        self.last_nearest_idx = 0
        self.reached_goal = False

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.target_marker_pub = rospy.Publisher('/lookahead_target_marker', Marker, queue_size=10)
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_timer = rospy.Timer(rospy.Duration(0.05), self.tf_callback)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def path_callback(self, msg):
        """Receive the A* path in map/world coordinates."""
        self.world_path_poses = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.path_cumulative_distances = self.compute_path_cumulative_distances()
        self.last_nearest_idx = 0
        self.reached_goal = False
        rospy.loginfo("Received A* path with %d poses", len(self.world_path_poses))

    def tf_callback(self, event):
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.05))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2.0, "TF lookup failed: %s", exc)
            return

        q = trans.transform.rotation
        _, _, yaw = t3d.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_position = (
            trans.transform.translation.x,
            trans.transform.translation.y)
        self.current_heading = yaw
        self.has_pose = True

    def compute_path_cumulative_distances(self):
        if not self.world_path_poses:
            return []

        distances = [0.0]
        for i in range(1, len(self.world_path_poses)):
            x1, y1 = self.world_path_poses[i - 1]
            x2, y2 = self.world_path_poses[i]
            distances.append(distances[-1] + math.hypot(x2 - x1, y2 - y1))
        return distances

    def find_nearest_point(self):
        if not self.world_path_poses:
            return None, None

        robot_x, robot_y = self.current_position
        search_start = max(0, self.last_nearest_idx - 50)
        search_end = min(len(self.world_path_poses), self.last_nearest_idx + 51)

        min_dist = float('inf')
        best_idx = self.last_nearest_idx
        for i in range(search_start, search_end):
            x, y = self.world_path_poses[i]
            dist = math.hypot(x - robot_x, y - robot_y)
            if dist < min_dist:
                min_dist = dist
                best_idx = i

        if min_dist > 2.0:
            for i, (x, y) in enumerate(self.world_path_poses):
                dist = math.hypot(x - robot_x, y - robot_y)
                if dist < min_dist:
                    min_dist = dist
                    best_idx = i

        self.last_nearest_idx = best_idx
        return best_idx, self.path_cumulative_distances[best_idx]

    def get_lookahead_target(self):
        if not self.world_path_poses or not self.path_cumulative_distances:
            return None

        nearest_idx, nearest_dist = self.find_nearest_point()
        if nearest_idx is None:
            return None

        goal_x, goal_y = self.world_path_poses[-1]
        target_dist = nearest_dist + self.lookahead_distance
        total_len = self.path_cumulative_distances[-1]
        if target_dist >= total_len:
            return (goal_x, goal_y)

        for i in range(nearest_idx, len(self.world_path_poses) - 1):
            if self.path_cumulative_distances[i + 1] >= target_dist:
                seg_len = self.path_cumulative_distances[i + 1] - self.path_cumulative_distances[i]
                ratio = (target_dist - self.path_cumulative_distances[i]) / seg_len if seg_len > 0 else 0.0
                x1, y1 = self.world_path_poses[i]
                x2, y2 = self.world_path_poses[i + 1]
                return (x1 + ratio * (x2 - x1), y1 + ratio * (y2 - y1))

        return (goal_x, goal_y)

    def compute_pure_pursuit_control(self, target):
        dx = target[0] - self.current_position[0]
        dy = target[1] - self.current_position[1]
        lookahead_dist = math.hypot(dx, dy)
        if lookahead_dist < 1e-6:
            return 0.0, 0.0, 0.0

        target_heading = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_heading - self.current_heading)

        if abs(alpha) > self.rotate_in_place_angle:
            angular = self.clamp(
                self.rotate_kp * alpha,
                -self.max_angular_velocity,
                self.max_angular_velocity)
            return 0.0, angular, alpha

        curvature = 2.0 * math.sin(alpha) / max(lookahead_dist, 1e-3)
        turn_ratio = min(abs(alpha), self.rotate_in_place_angle) / self.rotate_in_place_angle
        speed_scale = self.clamp(1.0 - 0.75 * turn_ratio, 0.25, 1.0)
        linear = self.linear_velocity * speed_scale
        if self.linear_velocity > 0.0 and linear < self.min_linear_velocity:
            linear = self.min_linear_velocity

        angular = self.clamp(
            linear * curvature,
            -self.max_angular_velocity,
            self.max_angular_velocity)
        return linear, angular, alpha

    def publish_target_marker(self, target):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "pure_pursuit_lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = target[0]
        marker.pose.position.y = target[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.target_marker_pub.publish(marker)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.has_pose or not self.world_path_poses or self.reached_goal:
                rate.sleep()
                continue

            goal_x, goal_y = self.world_path_poses[-1]
            goal_dist = math.hypot(
                goal_x - self.current_position[0],
                goal_y - self.current_position[1])
            if goal_dist <= self.goal_tolerance:
                self.stop_robot()
                self.reached_goal = True
                rospy.loginfo("Reached goal within %.2f m, stopping.", self.goal_tolerance)
                rate.sleep()
                continue

            target = self.get_lookahead_target()
            if target is None:
                rate.sleep()
                continue

            self.publish_target_marker(target)
            linear, angular, _ = self.compute_pure_pursuit_control(target)

            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("====START PURE PURSUIT====")
        node = PurePursuitNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
