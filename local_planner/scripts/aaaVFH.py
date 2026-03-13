#!/usr/bin/python3 
import rospy
import math
import numpy as np  
import transforms3d.euler as t3d_euler
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
import sys
from navigation_pkg.navigation_utils import (
    calcDanger, calc_h, calc_hp, calc_Hb, find_valleys, 
    calc_Target, pick_valley, pick_heading,
    project_trajectory, cost_trajectory, vfh_star_full
)
import tf2_ros
import tf.transformations as t3d
from geometry_msgs.msg import TransformStamped

class NavigationNode:
    def __init__(self):
        rospy.init_node('navigation_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub = rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        self.tf_buffer = tf2_ros.Buffer()  # 存储TF变换的缓冲区
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # 监听TF
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tf_callback)
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        
        # Read parameters and determine environment type.
        self.INIT_POSITION = rospy.get_param('init_position', [-2, 3, 1.57])
        self.GOAL_POSITION = rospy.get_param('goal_position', [0, 10])
        # If init position is exactly [-2, 3, 1.57], assume static; otherwise, dynamic.
        self.last_selected_index = 0
        self.path_flag = 0
        # Other parameters.
        self.max_range = 4.0
        self.sector_size = 8
        self.filter_width = 3
        self.threshold = 0.9
        self.robotDim = 0.6
        self.WidevalleyMin = 22
        self.is_distance_to_goal = False        # 当前位姿
        # New parameters for VFH*
        self.ds = self.robotDim  # projected step distance 
        self.ng = 2              # goal depth (number of projection steps)

        # Safety bubble parameters:
        self.safety_bubble_width = 180  
        self.safety_distance =  0.45  

        
        self.is_distance_to_goal = False        # 当前位姿
        self.current_position = (self.INIT_POSITION[0], self.INIT_POSITION[1])
        self.current_heading  = 0.0

        # 从全局路径中跟踪1米前方的点
        self.global_path = None
        self.lookahead_distance = 0.5  # 1米前向距离
        self.goal = None  # 等待RViz设置或从路径获取
        self.target_absolute_position = None
        self.init_fuzzy_controllers()

        # Initialize previous heading for temporal smoothing (memory term).
        self.prev_heading = None
        self.alpha = 0.5

        # Placeholders for sensor data.
        self.processed_lidar_ranges = None
        self.odom_data = False
        self.model_data = None
    def path_callback(self, msg):
        """订阅全局路径"""
        self.global_path = msg
        self.last_selected_index = 0
    
    def get_lookahead_target(self):
        """从路径中获取距离当前位置前方 lookahead_distance 米的点"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None
        if self.last_selected_index == 0:
            self.path_flag = 1
        else :
            self.path_flag = 0

        current_x, current_y = self.current_position

        # —— 新增：快到终点时直接锁终点 —— 
        last_pose = self.global_path.poses[-1]
        goal_x = last_pose.pose.position.x + self.INIT_POSITION[0]
        goal_y = last_pose.pose.position.y + self.INIT_POSITION[1]
        if math.hypot(goal_x - current_x, goal_y - current_y) <= 3.0:
            rospy.loginfo(f"距离终点 ≤3 m，直接锁定终点: x={goal_x:.2f}, y={goal_y:.2f}")
            return (goal_x, goal_y)
        # —— 新增结束 —— 

        # 计算当前位置到终点的方向向量
        to_goal_x = goal_x - current_x
        to_goal_y = goal_y - current_y
        
        # —— 新增：从上次选择的索引之后开始搜索，防止回退 —— 
        start_index = self.last_selected_index
        
        # 可选：如果当前位置已经远远超过上次选择的点，重置索引（防止异常情况）
        # 计算当前位置到最后选择点的距离
        if start_index < len(self.global_path.poses):
            last_selected_pose = self.global_path.poses[start_index]
            last_x = last_selected_pose.pose.position.x + self.INIT_POSITION[0]
            last_y = last_selected_pose.pose.position.y + self.INIT_POSITION[1]
            dist_to_last = math.hypot(current_x - last_x, current_y - last_y)
            
            # 如果距离上次选择的点超过5米，说明可能跳过了或出错了，重置搜索
            if dist_to_last > 5.0:
                rospy.logwarn(f"距离上次选择的点过远({dist_to_last:.2f}m)，重置路径点搜索")
                start_index = 0
        
        # 遍历路径点，从上次选择的索引之后开始
        for i in range(start_index, len(self.global_path.poses)):
            pose = self.global_path.poses[i]
            target_x = pose.pose.position.x + self.INIT_POSITION[0]
            target_y = pose.pose.position.y + self.INIT_POSITION[1]

            # 计算当前位置到该路径点的直线距离
            direct_distance = math.hypot(
                target_x - current_x,
                target_y - current_y)

            # 如果直线距离超过lookahead_distance，返回这个点并记录索引
            if direct_distance >= self.lookahead_distance:
                self.last_selected_index = i  # 记录本次选择的索引
                # rospy.loginfo(f"前向点[{i}]: x={target_x:.2f}, y={target_y:.2f}, 直线距离={direct_distance:.2f}m")
                return (target_x, target_y)
        # —— 新增结束 —— 

        # 如果遍历完都没找到（理论上不应发生），返回终点并重置索引
        rospy.loginfo(f"未找到满足前向距离的点，返回终点: x={goal_x:.2f}, y={goal_y:.2f}")
        self.last_selected_index = len(self.global_path.poses) - 1  # 重置到最后一个点
        return (goal_x, goal_y)

    def init_fuzzy_controllers(self):
        # --- Linear Velocity Fuzzy Controller ---
        lin_vel_max = 0.8
        linear_velocity = ctrl.Consequent(np.arange(0, lin_vel_max + 0.01, 0.01), 'Linear_Velocity')
        scale = 1.25
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
    
    def tf_callback(self, event):
        """
        定时器回调函数：周期性读取odom→base_link的TF变换，提取位置和朝向
        替代原有的odom话题回调函数
        """
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            "map",          # 父坐标系（对应odom话题的odom_frame）
            "base_link",     # 子坐标系（机器人基坐标系）
            rospy.Time(0),   # 获取最新的TF变换
            rospy.Duration(0.1) 
        )
        self.odom_data = True
        # 提取位置（叠加初始偏移，和你原代码逻辑一致）
        x = trans.transform.translation.x + self.INIT_POSITION[0]
        y = trans.transform.translation.y + self.INIT_POSITION[1]
        # 提取四元数并转换为偏航角（yaw），和你原代码逻辑一致
        q = trans.transform.rotation
        # 四元数顺序：x,y,z,w（tf.transformations标准格式）
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = t3d.euler_from_quaternion(quat)
        self.current_position = (x, y)
        self.current_heading = math.degrees(yaw)  # 转换为角度制

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
            
            if (self.processed_lidar_ranges is None or
                self.odom_data is False or
                self.global_path is None or
                self.is_distance_to_goal is True ):          # 等待拿到全局路径
                rate.sleep()
                continue

            # 动态获取前向跟踪目标点
            lookahead_target = self.get_lookahead_target()
            # rospy.loginfo(f"发布目标点: {lookahead_target}")
            if lookahead_target is not None:
                # 添加INIT_POSITION偏移
                self.target_absolute_position = (
                    lookahead_target[0] ,
                    lookahead_target[1] 
                )
                # rospy.loginfo(f"VFH")
            else:
                rate.sleep()
                continue
            # 计算到目标距离
            distance_to_goal = math.hypot(
                self.target_absolute_position[0] - self.current_position[0],
                self.target_absolute_position[1] - self.current_position[1])

            # VFH functions
            m = calcDanger(self.processed_lidar_ranges, self.max_range)
            h = calc_h(m, self.sector_size)
            hp = calc_hp(h, self.filter_width)
            hb = calc_Hb(h, self.threshold)
            heading_sector = calc_Target(self.target_absolute_position, self.current_position, self.current_heading)
            
            # Compute candidate heading using VFH* with A* search.
            candidate_heading = self.vfh_star(hb, heading_sector, self.current_position, self.current_heading)
            

            # --- Linear Velocity Calculation via Fuzzy Controller ---
            avg_val = np.min(h[35:55])
            if avg_val > self.max_range:
                avg_val = self.max_range
            self.linear_sim.input['Obstacle_Distance'] = avg_val
            self.linear_sim.compute()
            lin_vel = self.linear_sim.output['Linear_Velocity']
            
            smoothed_heading = candidate_heading 
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
            
            # --- Safety Bubble Check ---
            half_width = self.safety_bubble_width // 2
            start_idx = max(0, center_index - half_width)
            end_idx = min(num_beams, center_index + half_width + 1)
            safety_readings = self.processed_lidar_ranges[start_idx:end_idx]
            if np.any(safety_readings < self.safety_distance):
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
            dx = self.target_absolute_position[0] - self.current_position[0]
            dy = self.target_absolute_position[1] - self.current_position[1]
            
            # 计算目标方向的角度（角度制，与current_heading统一）
            target_heading_deg = np.degrees(np.arctan2(dy, dx))
            
            # 计算与当前航向的夹角差，并归一化到 [-180, 180]
            heading_diff_deg = target_heading_deg - self.current_heading
            heading_diff_deg = (heading_diff_deg + 180) % 360 - 180  # 归一化到 [-180, 180]
            heading_diff_deg = abs(heading_diff_deg)
            
            if heading_diff_deg > 45.0:
                # rospy.logwarn("[Safety Check] Heading diff %.1f° > 60°, performing in-place rotation", heading_diff_deg)
                
                # 停止前进，原地旋转对准目标
                twist.linear.x = 0.0
                
                # 根据夹角符号判断转向方向（使用未取绝对值的heading_diff）
                heading_diff_raw = target_heading_deg - self.current_heading
                heading_diff_raw = (heading_diff_raw + 180) % 360 - 180
                
                if heading_diff_raw > 0:
                    twist.angular.z = 1.0
                    rospy.loginfo("[Safety Check] Rotating LEFT to target")
                else:
                    twist.angular.z = -1.0
                    rospy.loginfo("[Safety Check] Rotating RIGHT to target")
            front_indices = slice(max(0, center_index - 60), min(num_beams, center_index + 61))
            front_readings = self.processed_lidar_ranges[front_indices]
            if (np.all(front_readings >= self.max_range) and 
                abs(smoothed_heading - heading_sector) < 5):
                #rospy.loginfo("Clear path detected and aligned with goal. Overriding speed to 2 m/s.")
                twist.linear.x = 2.0
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo("====START NAVIGATION====")
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
