#!/usr/bin/env python

import rospy
import math
import numpy as np
import transforms3d.euler as t3d_euler
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import sys
from navigation_pkg.navigation_utils import (
    calcDanger, calc_h, calc_hp, calc_Hb, find_valleys, 
    calc_Target, pick_valley, pick_heading,
    project_trajectory, cost_trajectory, vfh_star_full
)

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.odom_callback)
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        
        # Read parameters and determine environment type.
        self.INIT_POSITION = rospy.get_param('init_position', [-2, 3, 1.57])
        self.GOAL_POSITION = rospy.get_param('goal_position', [0, 10])
        # If init position is exactly [-2, 3, 1.57], assume static; otherwise, dynamic.
        self.environment_mode = 'static' if self.INIT_POSITION == [-2, 3, 1.57] else 'dynamic'
        
        # Other parameters.
        self.max_range = 4.0
        self.sector_size = 8
        self.filter_width = 3
        self.threshold = 0.9
        self.robotDim = 0.6
        self.WidevalleyMin = 22

        # New parameters for VFH*
        self.ds = self.robotDim  # projected step distance 
        self.ng = 2              # goal depth (number of projection steps)

        # Safety bubble parameters:
        self.safety_bubble_width = 180  
        self.safety_distance =  0.45  

        
        self.current_position = (self.INIT_POSITION[0], self.INIT_POSITION[1])
        self.current_heading = 0.0

        self.target_absolute_position = (
            self.INIT_POSITION[0] + self.GOAL_POSITION[0],
            self.INIT_POSITION[1] + self.GOAL_POSITION[1]
        )

        
        self.init_fuzzy_controllers()

        # Initialize previous heading for temporal smoothing (memory term).
        self.prev_heading = None
        self.alpha = 0.5

        # Placeholders for sensor data.
        self.processed_lidar_ranges = None
        self.odom_data = None
        self.model_data = None

    def init_fuzzy_controllers(self):
        # --- Linear Velocity Fuzzy Controller ---
        if self.environment_mode == 'static':
            lin_vel_max = 0.7
            linear_velocity = ctrl.Consequent(np.arange(0, lin_vel_max + 0.01, 0.01), 'Linear_Velocity')
            linear_velocity['Very_Low']  = fuzz.trimf(linear_velocity.universe, [-0.175, 0, 0.175])
            linear_velocity['Low']       = fuzz.trimf(linear_velocity.universe, [0, 0.175, 0.35])
            linear_velocity['Medium']    = fuzz.trimf(linear_velocity.universe, [0.175, 0.35, 0.525])
            linear_velocity['High']      = fuzz.trimf(linear_velocity.universe, [0.35, 0.525, 0.7])
            linear_velocity['Very_High'] = fuzz.trimf(linear_velocity.universe, [0.525, 0.7, 0.7537])
        else:
            lin_vel_max = 1.3
            linear_velocity = ctrl.Consequent(np.arange(0, lin_vel_max + 0.01, 0.01), 'Linear_Velocity')
            scale = lin_vel_max / 0.7  # approximately 2.57
            linear_velocity['Very_Low']  = fuzz.trimf(linear_velocity.universe, [-0.175 * scale, 0, 0.175 * scale])
            linear_velocity['Low']       = fuzz.trimf(linear_velocity.universe, [0, 0.175 * scale, 0.35 * scale])
            linear_velocity['Medium']    = fuzz.trimf(linear_velocity.universe, [0.175 * scale, 0.35 * scale, 0.525 * scale])
            linear_velocity['High']      = fuzz.trimf(linear_velocity.universe, [0.35 * scale, 0.525 * scale, 0.7 * scale])
            linear_velocity['Very_High'] = fuzz.trimf(linear_velocity.universe, [0.525 * scale, 0.7 * scale, 0.7 * scale])
        
        obstacle_distance = ctrl.Antecedent(np.arange(0, 4.01, 0.01), 'Obstacle_Distance')
        obstacle_distance['Very_Near'] = fuzz.trapmf(obstacle_distance.universe, [-0.9, 0, 0, 0.9])
        obstacle_distance['Near']      = fuzz.trimf(obstacle_distance.universe, [0, 1, 2])
        obstacle_distance['Midway']    = fuzz.trimf(obstacle_distance.universe, [1, 2, 3])
        obstacle_distance['Far']       = fuzz.trimf(obstacle_distance.universe, [2, 3, 4])
        obstacle_distance['Very_Far']  = fuzz.trapmf(obstacle_distance.universe, [3.1, 4, 4, 4.9])
        
        rule1 = ctrl.Rule(obstacle_distance['Very_Near'], linear_velocity['Very_High'])
        rule2 = ctrl.Rule(obstacle_distance['Near'],      linear_velocity['High'])
        rule3 = ctrl.Rule(obstacle_distance['Midway'],    linear_velocity['Medium'])
        rule4 = ctrl.Rule(obstacle_distance['Far'],       linear_velocity['Low'])
        rule5 = ctrl.Rule(obstacle_distance['Very_Far'],  linear_velocity['Very_Low'])
        
        linear_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        self.linear_sim = ctrl.ControlSystemSimulation(linear_ctrl)
        
        #  Angular Velocity Fuzzy Controller ---
        angular_input = ctrl.Antecedent(np.arange(0, 90.01, 0.1), 'Angular_Input')
        angular_input['very_left']  = fuzz.trapmf(angular_input.universe, [-20.25, -2.25, 0, 22.5])
        angular_input['left']       = fuzz.trimf(angular_input.universe, [0, 22.5, 45])
        angular_input['zero']       = fuzz.trimf(angular_input.universe, [22.5, 45, 67.5])
        angular_input['right']      = fuzz.trimf(angular_input.universe, [45, 67.5, 90])
        angular_input['very_right'] = fuzz.trapmf(angular_input.universe, [67.5, 90, 90, 110.2])
        
        if self.environment_mode == 'static':
            # Static environment: Universe from -π to π.
            angular_output = ctrl.Consequent(np.arange(-np.pi, np.pi + 0.01, 0.01), 'Angular_Output')
            # Use a scaling factor based on the old static max (2.5) to the new max (π)
            scale_static = np.pi / 2.5  # ~1.25664
            angular_output['very_neg'] = fuzz.trapmf(angular_output.universe, 
                                                     [-2.5 * scale_static, -2.5 * scale_static, -2.5 * scale_static, -1.25 * scale_static])
            angular_output['neg']      = fuzz.trimf(angular_output.universe, 
                                                     [-2.5 * scale_static, -1.25 * scale_static, 0])
            angular_output['zero']     = fuzz.trimf(angular_output.universe, 
                                                     [-1.25 * scale_static, 0, 1.25 * scale_static])
            angular_output['pos']      = fuzz.trimf(angular_output.universe, 
                                                     [0, 1.25 * scale_static, 2.5 * scale_static])
            angular_output['very_pos'] = fuzz.trapmf(angular_output.universe, 
                                                     [1.25 * scale_static, 2.5 * scale_static, 2.5 * scale_static, 2.5 * scale_static])
        else:
            # Dynamic environment: Universe from -3.5 to 3.5.
            angular_output = ctrl.Consequent(np.arange(-3.5, 3.5 + 0.01, 0.01), 'Angular_Output')
            scale_dynamic = 3.5 / 3.0  # ~1.16667
            angular_output['very_neg'] = fuzz.trapmf(angular_output.universe, 
                                                     [-3.0 * scale_dynamic, -3.0 * scale_dynamic, -3.0 * scale_dynamic, -1.5 * scale_dynamic])
            angular_output['neg']      = fuzz.trimf(angular_output.universe, 
                                                     [-3.0 * scale_dynamic, -1.5 * scale_dynamic, 0])
            angular_output['zero']     = fuzz.trimf(angular_output.universe, 
                                                     [-1.5 * scale_dynamic, 0, 1.5 * scale_dynamic])
            angular_output['pos']      = fuzz.trimf(angular_output.universe, 
                                                     [0, 1.5 * scale_dynamic, 3.0 * scale_dynamic])
            angular_output['very_pos'] = fuzz.trapmf(angular_output.universe, 
                                                     [1.5 * scale_dynamic, 3.0 * scale_dynamic, 3.0 * scale_dynamic, 3.0 * scale_dynamic])
        
        rule1_ang = ctrl.Rule(angular_input['very_left'],  angular_output['very_pos'])
        rule2_ang = ctrl.Rule(angular_input['left'],       angular_output['pos'])
        rule3_ang = ctrl.Rule(angular_input['zero'],       angular_output['zero'])
        rule4_ang = ctrl.Rule(angular_input['right'],      angular_output['neg'])
        rule5_ang = ctrl.Rule(angular_input['very_right'], angular_output['very_neg'])
        
        angular_ctrl = ctrl.ControlSystem([rule1_ang, rule2_ang, rule3_ang, rule4_ang, rule5_ang])
        self.angular_sim = ctrl.ControlSystemSimulation(angular_ctrl)
    
    def lidar_callback(self, msg):
        lidar_range = np.array(msg.ranges)
        lidar_ranges = np.flip(lidar_range)
        lidar_ranges[lidar_ranges > self.max_range] = self.max_range
        self.processed_lidar_ranges = lidar_ranges
    
    def odom_callback(self, msg):
        self.odom_data = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        q_ros = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        roll, pitch, yaw = t3d_euler.quat2euler(q_ros, axes='sxyz')
        theta = math.degrees(yaw)
        if theta < -180:
            theta += 360
        elif theta > 180:
            theta -= 360
        self.current_position = (x + self.INIT_POSITION[0], y + self.INIT_POSITION[1])
        self.current_heading = theta
    
    def model_callback(self, msg):
        self.model_data = msg
    
    def vfh_star(self, hb, heading_sector, current_position, current_heading):
        """
        Full VFH* look-ahead using an A*–like search that penalizes switching.
        Pass the previous smoothed heading (or heading_sector if none) to add a switching penalty.
        """
        prev_h = self.prev_heading if self.prev_heading is not None else heading_sector
        candidate = vfh_star_full(current_position, current_heading, heading_sector,
                                  self.ds, self.ng, hb, self.threshold, self.robotDim, self.WidevalleyMin, prev_h)
        #rospy.loginfo("VFH* candidate heading: %d", candidate)
        return candidate
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            
            if self.processed_lidar_ranges is None or self.odom_data is None:
                rate.sleep()
                continue

            # - Goal Reached Check 
            distance_to_goal = math.hypot(
                self.target_absolute_position[0] - self.current_position[0],
                self.target_absolute_position[1] - self.current_position[1]
            )
            if distance_to_goal < 0.5:
                rospy.loginfo("Goal reached!")
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                break

            # VFH functions
            m = calcDanger(self.processed_lidar_ranges, self.max_range)
            h = calc_h(m, self.sector_size)
            hp = calc_hp(h, self.filter_width)
            hb = calc_Hb(h, self.threshold)
            heading_sector = calc_Target(self.target_absolute_position, self.current_position, self.current_heading)
            
            # Compute candidate heading using VFH* with A* search.
            candidate_heading = self.vfh_star(hb, heading_sector, self.current_position, self.current_heading)
            
            # Temporal smoothing: low-pass filter.
            if self.prev_heading is None:
                smoothed_heading = candidate_heading
            else:
                smoothed_heading = self.alpha * candidate_heading + (1 - self.alpha) * self.prev_heading
            self.prev_heading = smoothed_heading
            
            # --- Linear Velocity Calculation via Fuzzy Controller ---
            avg_val = np.min(h[35:55])
            if avg_val > self.max_range:
                avg_val = self.max_range
            self.linear_sim.input['Obstacle_Distance'] = avg_val
            self.linear_sim.compute()
            lin_vel = self.linear_sim.output['Linear_Velocity']
            
            # --- Angular Velocity Calculation via Fuzzy Controller ---
            self.angular_sim.input['Angular_Input'] = smoothed_heading
            self.angular_sim.compute()
            ang_vel = self.angular_sim.output['Angular_Output']
            
            twist = Twist()
            twist.linear.x = lin_vel
            twist.angular.z = ang_vel
            
            # --- Clear Path Override ---
            num_beams = len(self.processed_lidar_ranges)
            center_index = num_beams // 2
            #print("num beams:", num_beams)#720
            #print("center index", center_index)#360
            front_indices = slice(max(0, center_index - 60), min(num_beams, center_index + 61))
            front_readings = self.processed_lidar_ranges[front_indices]
            if (np.all(front_readings >= self.max_range) and 
                abs(smoothed_heading - heading_sector) < 5):
                #rospy.loginfo("Clear path detected and aligned with goal. Overriding speed to 2 m/s.")
                twist.linear.x = 2.0
            
            # --- Safety Bubble Check ---
            half_width = self.safety_bubble_width // 2
            start_idx = max(0, center_index - half_width)
            end_idx = min(num_beams, center_index + half_width + 1)
            safety_readings = self.processed_lidar_ranges[start_idx:end_idx]
            if np.any(safety_readings < self.safety_distance):
                #rospy.logwarn("Safety bubble triggered! Obstacle detected in front area.")
                twist.linear.x = 0.0
                mid = len(safety_readings) // 2
                left_clearance = np.min(safety_readings[:mid]) if mid > 0 else self.max_range
                right_clearance = np.min(safety_readings[mid:]) if mid < len(safety_readings) else self.max_range
                if left_clearance > right_clearance:
                    twist.angular.z = 0.5  # Turn left.
                    twist.linear.x = -0.4  #was 0.5
                else:
                    twist.angular.z = -0.5  # Turn right.
                    twist.linear.x = -0.4 # was 0.5
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("====START NAVIGATION====")
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass