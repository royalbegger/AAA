#!/usr/bin/env python3
import os
import rospy, math, numpy as np, transforms3d.euler as t3d_euler, skfuzzy as fuzz
from skfuzzy import control as ctrl
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, Point   # <-- 新增 
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


        # ************ 关键新增：监听 RViz 点击 goal ************
        # RViz 默认话题 /move_base_simple/goal ，消息类型 geometry_msgs/PoseStamped
        # 如果你用“Publish Point”工具，话题是 /clicked_point，类型 PointStamped
        # 下面同时兼容两种，优先 PoseStamped，其次 PointStamped


        # 参数只保留起点
        self.INIT_POSITION = rospy.get_param('init_position', [0, 0, 0])
        self.environment_mode = 'static'
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lidar_sub   = rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        self.tf_buffer = tf2_ros.Buffer()  # 存储TF变换的缓冲区
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # 监听TF
        self.timer = rospy.Timer(rospy.Duration(0.1), self.tf_callback)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        self.last_selected_index = 0
        # self.rviz_goal_sub = rospy.Subscriber(
        #     '/move_base_simple/goal', PoseStamped, self.rviz_goal_cb)
        self.rviz_point_sub = rospy.Subscriber(
            '/clicked_point', PointStamped, self.rviz_point_cb)
        # *******************************************************
        # 其余参数保持原样
        self.target_absolute_position = None
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.4)
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 0.6)    # m/s^2
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 1.5)  # rad/s^2
        self.cmd_dt = rospy.get_param('~cmd_dt', 0.1)
        self.prev_cmd_linear = 0.0
        self.prev_cmd_angular = 0.0
        self.last_cmd_time = None
        self.max_range = 4.0
        self.sector_size = 8
        self.vfh_sector_count = 90
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
        self.lookahead_distance = 0.8  # 1米前向距离
        self.max_turn_curvature = max(rospy.get_param('~max_turn_curvature', 3.0), 0.1)
        self.goal = None  # 等待RViz设置或从路径获取
        self.init_fuzzy_controllers()
        self.prev_heading = None
        self.alpha = 1.0
        self.processed_lidar_ranges = None
        self.odom_data = False
        self.viz_pub = rospy.Publisher('/vfh/debug/markers', MarkerArray, queue_size=1)
        self.lookahead_target_pub = rospy.Publisher('/vfh/lookahead_target', Marker, queue_size=1)  # 追踪点可视化
        self.id_cnt  = 0          # 全局 id 计数器，防止重名

        # 运行统计：时间、行驶里程、平均速度
        default_stats_file = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'test_record.txt'))
        self.stats_file = os.path.expanduser(
            rospy.get_param('~run_stats_file', default_stats_file))
        self.stats_start_time = None
        self.stats_end_time = None
        self.total_distance = 0.0
        self.last_distance_position = None
        self.stats_recorded = False
        rospy.on_shutdown(self.finish_run_stats)

        # 预定义颜色
        self.c_gray   = ColorRGBA(0.5, 0.5, 0.5, 0.8)
        self.c_red    = ColorRGBA(1.0, 0.0, 0.0, 0.9)
        self.c_green  = ColorRGBA(0.0, 1.0, 0.0, 0.9)
        self.c_green_fill = ColorRGBA(0.0, 1.0, 0.15, 0.22)
        self.c_blue   = ColorRGBA(0.0, 0.35, 1.0, 1.0)
        self.c_yellow = ColorRGBA(1.0, 1.0, 0.0, 0.9)

    # ---------------- 新增回调 ----------------
    def rviz_goal_cb(self, msg):
        """接收 geometry_msgs/PoseStamped，提取 x,y 作为目标"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.set_goal(x, y)

    def rviz_point_cb(self, msg):
        """接收 geometry_msgs/PointStamped，提取 x,y 作为目标"""
        x = msg.point.x
        y = msg.point.y
        self.set_goal(x, y)

    def set_goal(self, x, y):
        """公用函数：设置目标并打印日志"""
        x=x+self.INIT_POSITION[0]
        y=y+self.INIT_POSITION[1]
        self.goal = (x, y)
        self.target_absolute_position = (x, y)
        rospy.loginfo("New goal set by RViz: x=%.2f  y=%.2f", x, y)
    # -----------------------------------------
    def publish_vfh_markers(self, h, hp, hb, valleys,
                            target_sector, chosen_sector,
                            target_position=None):
        """所有 VFH 中间结果一次性发出去"""
        m_arr = MarkerArray()
        self.id_cnt = 0
        robot_x, robot_y = self.current_position

        # 1) 障碍物直方图 H  —— 放射状灰条
        max_h = max(h) if max(h) != 0 else 1.0
        n_sec = len(h)
        if n_sec <= 0:
            return
        for i in range(n_sec):
            ang = math.radians(self._sector_to_world_angle(i + 1, n_sec))
            L = 0.3 + 1.2 * h[i] / max_h          # 长度 0.3~1.5 m
            dx, dy = L * np.cos(ang), L * np.sin(ang)
            m_arr.markers.append(
                self._line_marker(robot_x, robot_y,
                                robot_x + dx, robot_y + dy,
                                self.c_gray, 0.03))

        # 2) 二值化 hb —— 红色填充扇形
        for i in range(n_sec):
            if hb[i] == 1:
                ang0 = math.radians(self._sector_boundary_to_world_angle(i, n_sec))
                ang1 = math.radians(self._sector_boundary_to_world_angle(i + 1, n_sec))
                m_arr.markers.append(
                    self._wedge_marker(robot_x, robot_y,
                                    ang0, ang1,
                                    1.0, self.c_red, 0.05))

        # 3) Valley 可行谷 —— 绿色粗弧
        if len(valleys) > 0:
            for valley in valleys:
                start = float(valley[0])
                end   = float(valley[1])
                ang0 = math.radians(self._sector_to_world_angle(start, n_sec))
                ang1 = math.radians(self._sector_to_world_angle(end, n_sec))
                m_arr.markers.append(
                    self._filled_wedge_marker(robot_x, robot_y,
                                    ang0, ang1,
                                    1.35, self.c_green_fill))
                m_arr.markers.append(
                    self._wedge_marker(robot_x, robot_y,
                                    ang0, ang1,
                                    1.35, self.c_green, 0.12))

        # 4) 目标方向 —— 黄色细线
        if target_position is not None:
            target_dx = target_position[0] - robot_x
            target_dy = target_position[1] - robot_y
            target_len = math.hypot(target_dx, target_dy)
            if target_len > 1e-6:
                line_len = min(2.0, target_len)
                tdx = line_len * target_dx / target_len
                tdy = line_len * target_dy / target_len
            else:
                tdx = 0.0
                tdy = 0.0
        else:
            target_rad = math.radians(self._sector_to_world_angle(target_sector, n_sec))
            tdx = 1.8 * np.cos(target_rad)
            tdy = 1.8 * np.sin(target_rad)
        m_arr.markers.append(
            self._line_marker(robot_x, robot_y,
                            robot_x + tdx, robot_y + tdy,
                            self.c_yellow, 0.04))

        # 5) VFH* 最终候选角 —— 蓝色粗箭头
        chosen_rad = math.radians(self._sector_to_world_angle(chosen_sector, n_sec))
        cdx = 2.2 * np.cos(chosen_rad)
        cdy = 2.2 * np.sin(chosen_rad)
        m_arr.markers.append(
            self._arrow_marker(robot_x, robot_y,
                            robot_x + cdx, robot_y + cdy,
                            self.c_blue, 0.08))
        label_x = robot_x + cdx + 0.25 * np.cos(chosen_rad + np.pi / 2.0)
        label_y = robot_y + cdy + 0.25 * np.sin(chosen_rad + np.pi / 2.0)
        m_arr.markers.append(
            self._text_marker(label_x, label_y, 0.35,
                              self._format_sector_label(chosen_sector),
                              self.c_blue))

        self.viz_pub.publish(m_arr)

    def _sector_to_relative_angle(self, sector, n_sec):
        """Convert VFH sector number to robot-relative degrees."""
        if n_sec <= 1:
            return 0.0
        sector = min(max(float(sector), 1.0), float(n_sec))
        return 135.0 - (sector - 1.0) * (270.0 / (n_sec - 1.0))

    def _sector_to_world_angle(self, sector, n_sec):
        return self.current_heading + self._sector_to_relative_angle(sector, n_sec)

    def _sector_boundary_to_world_angle(self, boundary, n_sec):
        if n_sec <= 0:
            return self.current_heading
        boundary = min(max(float(boundary), 0.0), float(n_sec))
        relative_angle = 135.0 - boundary * (270.0 / float(n_sec))
        return self.current_heading + relative_angle

    def _format_sector_label(self, sector):
        sector = float(sector)
        if abs(sector - round(sector)) < 1e-3:
            return "sector %d" % int(round(sector))
        return "sector %.1f" % sector


    # ------------- 工具 maker -------------
    def _line_marker(self, x0, y0, x1, y1, color, width):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = rospy.Time.now()
        m.ns = "vfh"
        m.id = self.id_cnt; self.id_cnt += 1
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.scale.x = width
        m.color = color
        m.lifetime = rospy.Duration(0.3)
        # 添加INIT_POSITION偏移
        x0 = x0 - self.INIT_POSITION[0]
        y0 = y0 - self.INIT_POSITION[1]
        x1 = x1 - self.INIT_POSITION[0]
        y1 = y1 - self.INIT_POSITION[1]
        m.points = [Point(x0, y0, 0), Point(x1, y1, 0)]
        return m

    def _arrow_marker(self, x0, y0, x1, y1, color, width):
        m = self._line_marker(x0, y0, x1, y1, color, width)
        m.type = m.ARROW
        m.scale.y = width * 3.0
        m.scale.z = width * 4.0
        return m

    def _text_marker(self, x, y, z, text, color):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = rospy.Time.now()
        m.ns = "vfh"
        m.id = self.id_cnt; self.id_cnt += 1
        m.type = m.TEXT_VIEW_FACING
        m.action = m.ADD
        m.pose.position.x = x - self.INIT_POSITION[0]
        m.pose.position.y = y - self.INIT_POSITION[1]
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.z = 0.38
        m.color = color
        m.text = text
        m.lifetime = rospy.Duration(0.3)
        return m

    def _wedge_marker(self, cx, cy, ang0, ang1, radius, color, width):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = rospy.Time.now()
        m.ns = "vfh"
        m.id = self.id_cnt; self.id_cnt += 1
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.scale.x = width
        m.color = color
        m.lifetime = rospy.Duration(0.3)
        ang0 = float(ang0)
        ang1 = float(ang1)
        # 添加INIT_POSITION偏移
        cx = cx - self.INIT_POSITION[0]
        cy = cy - self.INIT_POSITION[1]
        steps = max(2, int(abs(ang1 - ang0) / 0.05) + 2)
        for i in range(steps):
            a = ang0 + i * (ang1 - ang0) / (steps - 1)
            m.points.append(Point(cx + radius * np.cos(a),
                                cy + radius * np.sin(a), 0))
        m.points.append(Point(cx, cy, 0))  # 闭合
        return m

    def _filled_wedge_marker(self, cx, cy, ang0, ang1, radius, color):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = rospy.Time.now()
        m.ns = "vfh"
        m.id = self.id_cnt; self.id_cnt += 1
        m.type = m.TRIANGLE_LIST
        m.action = m.ADD
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.color = color
        m.lifetime = rospy.Duration(0.3)
        cx = cx - self.INIT_POSITION[0]
        cy = cy - self.INIT_POSITION[1]
        steps = max(3, int(abs(ang1 - ang0) / 0.05) + 2)
        center = Point(cx, cy, 0.01)
        prev = None
        for i in range(steps):
            a = ang0 + i * (ang1 - ang0) / (steps - 1)
            pt = Point(cx + radius * np.cos(a),
                       cy + radius * np.sin(a), 0.01)
            if prev is not None:
                m.points.extend([center, prev, pt])
            prev = pt
        return m

    def _colored_line_strip(self, pts, costs):
        """根据代价渐变颜色"""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp    = rospy.Time.now()
        m.ns = "vfh"
        m.id = self.id_cnt; self.id_cnt += 1
        m.type = m.LINE_STRIP
        m.action = m.ADD
        m.scale.x = 0.04
        m.lifetime = rospy.Duration(0.3)
        max_c = max(costs) if max(costs) != 0 else 1.0
        for i, (x, y) in enumerate(pts):
            # 添加INIT_POSITION偏移
            x_offset = x - self.INIT_POSITION[0]
            y_offset = y - self.INIT_POSITION[1]
            m.points.append(Point(x_offset, y_offset, 0))
            c = costs[i] / max_c
            m.colors.append(ColorRGBA(1 - c, c, 0.0, 0.9))
        return m
    
    def _create_target_marker(self, x, y):
        """创建表示追踪目标点的marker（球体）"""
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = rospy.Time.now()
        m.ns = "lookahead_target"
        m.id = 0
        m.type = m.SPHERE  # 使用球体表示目标点
        m.action = m.ADD
        m.pose.position.x = x - self.INIT_POSITION[0]
        m.pose.position.y = y - self.INIT_POSITION[1]
        m.pose.position.z = 0.5  # 升高z轴便于观察
        m.scale.x = 0.4  # 直径0.4m的球
        m.scale.y = 0.4
        m.scale.z = 0.4
        m.color = ColorRGBA(1.0, 0.5, 0.0, 0.9)  # 橙色
        m.lifetime = rospy.Duration(0.5)
        return m
    
    def path_callback(self, msg):
        """订阅全局路径"""
        self.global_path = msg
        self.last_selected_index = 0
    
    def get_lookahead_target(self):
        """从路径中获取距离当前位置前方 lookahead_distance 米的点"""
        if self.global_path is None or len(self.global_path.poses) == 0:
            return None

        current_x, current_y = self.current_position

        # —— 新增：快到终点时直接锁终点 —— 
        last_pose = self.global_path.poses[-1]
        goal_x = last_pose.pose.position.x + self.INIT_POSITION[0]
        goal_y = last_pose.pose.position.y + self.INIT_POSITION[1]
        # if math.hypot(goal_x - current_x, goal_y - current_y) <= 3.0:
        #     rospy.loginfo(f"距离终点 ≤3 m，直接锁定终点: x={goal_x:.2f}, y={goal_y:.2f}")
        #     return (goal_x, goal_y)
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
        if self.environment_mode == 'static':
            lin_vel_max = 0.9
            linear_velocity = ctrl.Consequent(np.arange(0, lin_vel_max + 0.01, 0.01), 'Linear_Velocity')
            scale = 2.0
            linear_velocity['Very_Low']  = fuzz.trimf(linear_velocity.universe, [-0.175 * scale, 0.2, 0.375 * scale])
            linear_velocity['Low']       = fuzz.trimf(linear_velocity.universe, [0, 0.275 * scale, 0.45 * scale])
            linear_velocity['Medium']    = fuzz.trimf(linear_velocity.universe, [0.25 * scale, 0.4 * scale, 0.625 * scale])
            linear_velocity['High']      = fuzz.trimf(linear_velocity.universe, [0.4 * scale, 0.625 * scale, 0.8 * scale])
            linear_velocity['Very_High'] = fuzz.trimf(linear_velocity.universe, [0.625 * scale, 0.7 * scale, 0.9 * scale])
        else:
            lin_vel_max = 1.8
            linear_velocity = ctrl.Consequent(np.arange(0, lin_vel_max + 0.01, 0.01), 'Linear_Velocity')
            scale = lin_vel_max / 0.7  # approximately 2.57
            linear_velocity['Very_Low']  = fuzz.trimf(linear_velocity.universe, [-0.175 * scale, 0, 0.175 * scale])
            linear_velocity['Low']       = fuzz.trimf(linear_velocity.universe, [0, 0.175 * scale, 0.35 * scale])
            linear_velocity['Medium']    = fuzz.trimf(linear_velocity.universe, [0.275 * scale, 0.35 * scale, 0.625 * scale])
            linear_velocity['High']      = fuzz.trimf(linear_velocity.universe, [0.35 * scale, 0.525 * scale, 0.8 * scale])
            linear_velocity['Very_High'] = fuzz.trimf(linear_velocity.universe, [0.525 * scale, 0.7 * scale, 0.9 * scale])
        
        obstacle_distance = ctrl.Antecedent(np.arange(0, 4.01, 0.01), 'Obstacle_Distance')
        obstacle_distance['Very_Near'] = fuzz.trapmf(obstacle_distance.universe, [-0.9, 0, 0, 0.5])
        obstacle_distance['Near']      = fuzz.trimf(obstacle_distance.universe, [0, 0.5, 1.2])
        obstacle_distance['Midway']    = fuzz.trimf(obstacle_distance.universe, [1, 1.5, 2.5])
        obstacle_distance['Far']       = fuzz.trimf(obstacle_distance.universe, [2.5, 4, 5])
        obstacle_distance['Very_Far']  = fuzz.trapmf(obstacle_distance.universe, [3.1, 4, 4, 6])

        turn_curvature = ctrl.Antecedent(
            np.arange(0, self.max_turn_curvature + 0.01, 0.01),
            'Turn_Curvature')
        curvature_scale = self.max_turn_curvature
        turn_curvature['Small'] = fuzz.trapmf(
            turn_curvature.universe,
            [0, 0, 0.35 * curvature_scale, 0.55 * curvature_scale])
        turn_curvature['Medium'] = fuzz.trimf(
            turn_curvature.universe,
            [0.35 * curvature_scale, 0.60 * curvature_scale, 0.85 * curvature_scale])
        turn_curvature['Large'] = fuzz.trapmf(
            turn_curvature.universe,
            [0.65 * curvature_scale, 0.90 * curvature_scale,
             curvature_scale, curvature_scale])
        
        rule1 = ctrl.Rule(obstacle_distance['Very_Near'], linear_velocity['Very_Low'])
        rule2 = ctrl.Rule(obstacle_distance['Near'],      linear_velocity['Low'])
        rule3 = ctrl.Rule(obstacle_distance['Midway'],    linear_velocity['Medium'])
        rule4 = ctrl.Rule(obstacle_distance['Far'],       linear_velocity['High'])
        rule5 = ctrl.Rule(obstacle_distance['Very_Far'],  linear_velocity['Very_High'])
        rule6 = ctrl.Rule(turn_curvature['Medium'],       linear_velocity['Medium'])
        rule7 = ctrl.Rule(turn_curvature['Large'],        linear_velocity['Low'])
        
        linear_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7])
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

    # ---------- sensor callbacks ----------
    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.flip(ranges)
        ranges[np.isnan(ranges)] = self.max_range
        ranges[ranges > self.max_range] = self.max_range
        self.processed_lidar_ranges = ranges

    # def odom_callback(self, msg):
    #     self.odom_data = msg
    #     x = msg.pose.pose.position.x + self.INIT_POSITION[0]
    #     y = msg.pose.pose.position.y + self.INIT_POSITION[1]
    #     q = msg.pose.pose.orientation
    #     _, _, yaw = t3d_euler.quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')
    #     self.current_position = (x, y)
    #     self.current_heading  = math.degrees(yaw)
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
        self.update_travel_distance(self.current_position)

    # ---------- VFH* ----------
    def vfh_star(self, hb, heading_sector, current_position, current_heading):
        prev = self.prev_heading if self.prev_heading is not None else heading_sector
        return vfh_star_full(current_position, current_heading,
                             heading_sector, self.ds, self.ng,
                             hb, self.threshold, self.robotDim,
                             self.WidevalleyMin, prev)

    def limit_rate(self, target, previous, max_accel, dt):
        if max_accel <= 0.0 or dt <= 0.0:
            return target
        max_delta = max_accel * dt
        delta = target - previous
        if delta > max_delta:
            return previous + max_delta
        if delta < -max_delta:
            return previous - max_delta
        return target

    def apply_acceleration_limits(self, twist):
        now = rospy.Time.now()
        if self.last_cmd_time is None:
            dt = self.cmd_dt
        else:
            dt = (now - self.last_cmd_time).to_sec()
            if dt <= 0.0:
                dt = self.cmd_dt
            else:
                dt = min(dt, self.cmd_dt)

        twist.linear.x = self.limit_rate(
            twist.linear.x, self.prev_cmd_linear, self.max_linear_accel, dt)
        twist.angular.z = self.limit_rate(
            twist.angular.z, self.prev_cmd_angular, self.max_angular_accel, dt)

        self.prev_cmd_linear = twist.linear.x
        self.prev_cmd_angular = twist.angular.z
        self.last_cmd_time = now
        return twist

    def reset_cmd_history(self):
        self.prev_cmd_linear = 0.0
        self.prev_cmd_angular = 0.0
        self.last_cmd_time = None

    def start_run_stats(self):
        if self.stats_start_time is not None:
            return
        self.stats_start_time = rospy.Time.now()
        self.total_distance = 0.0
        self.last_distance_position = self.current_position
        self.stats_recorded = False
        rospy.loginfo("Navigation statistics started.")

    def update_travel_distance(self, position):
        if self.stats_start_time is None or self.stats_recorded:
            return
        if self.is_distance_to_goal:
            return
        if self.last_distance_position is None:
            self.last_distance_position = position
            return

        dx = position[0] - self.last_distance_position[0]
        dy = position[1] - self.last_distance_position[1]
        step_distance = math.hypot(dx, dy)
        if math.isfinite(step_distance):
            self.total_distance += step_distance
            self.last_distance_position = position

    def get_next_experiment_index(self):
        if not os.path.exists(self.stats_file):
            return 1

        experiment_count = 0
        with open(self.stats_file, 'r', encoding='utf-8') as record_file:
            for line in record_file:
                line = line.strip()
                if line.startswith('第') and '次实验' in line:
                    experiment_count += 1
        return experiment_count + 1

    def finish_run_stats(self, reason='shutdown'):
        if self.stats_start_time is None or self.stats_recorded:
            return

        self.update_travel_distance(self.current_position)
        self.stats_end_time = rospy.Time.now()
        elapsed_time = (self.stats_end_time - self.stats_start_time).to_sec()
        if elapsed_time < 0.0:
            elapsed_time = 0.0
        average_speed = self.total_distance / elapsed_time if elapsed_time > 0.0 else 0.0

        stats_dir = os.path.dirname(self.stats_file)
        if stats_dir:
            os.makedirs(stats_dir, exist_ok=True)
        experiment_index = self.get_next_experiment_index()
        record_line = (
            "第%d次实验 行驶里程: %.3f m 行驶时间: %.3f s 平均速度: %.3f m/s\n" %
            (experiment_index, self.total_distance, elapsed_time, average_speed))
        with open(self.stats_file, 'a', encoding='utf-8') as record_file:
            record_file.write(record_line)

        self.stats_recorded = True
        rospy.loginfo(
            "Navigation statistics saved to %s: duration=%.3fs, distance=%.3fm, average_speed=%.3fm/s",
            self.stats_file, elapsed_time, self.total_distance, average_speed)

    def estimate_turn_curvature(self, heading_sector, lookahead_distance):
        lookahead_distance = max(float(lookahead_distance), 0.05)
        relative_angle = math.radians(
            self._sector_to_relative_angle(heading_sector, self.vfh_sector_count))
        curvature = abs(2.0 * math.sin(relative_angle) / lookahead_distance)
        return min(curvature, self.max_turn_curvature)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if (self.processed_lidar_ranges is None or
                self.odom_data is False or
                self.global_path is None or
                self.is_distance_to_goal is True ):   
                rate.sleep()
                continue

            self.start_run_stats()

            # 动态获取前向跟踪目标点
            lookahead_target = self.get_lookahead_target()
            # rospy.loginfo(f"发布目标点: {lookahead_target}")
            if lookahead_target is not None:
                self.target_absolute_position = (
                    lookahead_target[0] ,
                    lookahead_target[1] 
                )
                # 发布目标点的marker进行可视化
                target_marker = self._create_target_marker(lookahead_target[0], lookahead_target[1])
                self.lookahead_target_pub.publish(target_marker)
                # rospy.loginfo(f"VFH")
            else:
                rate.sleep()
                continue
            # 计算终点距离并判断是否到达目标点
            goal_pose = self.global_path.poses[-1]
            goal_x = goal_pose.pose.position.x + self.INIT_POSITION[0]
            goal_y = goal_pose.pose.position.y + self.INIT_POSITION[1]
            distance_to_goal = math.hypot(
                goal_x - self.current_position[0],
                goal_y - self.current_position[1])
            if distance_to_goal <= self.goal_tolerance:
                rospy.loginfo("Reached goal within %.2f m, stopping.", self.goal_tolerance)
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self.reset_cmd_history()
                self.finish_run_stats(reason='goal_reached')
                self.is_distance_to_goal = True
                self.goal = None
                self.target_absolute_position = None
                rate.sleep()
                continue
            # 计算到当前追踪点的距离，用于可视化和跟踪控制
            lookahead_distance = math.hypot(
                self.target_absolute_position[0] - self.current_position[0],
                self.target_absolute_position[1] - self.current_position[1])
            if lookahead_distance < 0.2:
                rospy.loginfo("Near lookahead target, but not yet at goal.")
            
            # VFH*、模糊控制、安全泡泡
            m  = calcDanger(self.processed_lidar_ranges, self.max_range)
            # rospy.loginfo(f"VFH")
            h  = calc_h(m, self.sector_size, self.vfh_sector_count)
            hp = calc_hp(h, self.filter_width)
            hb = calc_Hb(h, self.threshold)
            valleys, _ = find_valleys(hb, self.WidevalleyMin, self.robotDim)
            num_beams = len(self.processed_lidar_ranges)
            center_index = num_beams // 2
            front_indices = slice(max(0, center_index - 60),
                                  min(num_beams, center_index + 61))
            front_readings = self.processed_lidar_ranges[front_indices]
            obstacle_distance = self.max_range
            if len(front_readings) > 0:
                obstacle_distance = float(np.min(front_readings))
            obstacle_distance = min(max(obstacle_distance, 0.0), self.max_range)
            heading_sector = calc_Target(self.target_absolute_position,
                                         self.current_position, self.current_heading,
                                         self.vfh_sector_count)
            sector_angle = -((heading_sector - 1) * (270.0 / 89.0) - 135.0)
            candidate_heading = self.vfh_star(hb, heading_sector,
                                              self.current_position, self.current_heading)
            if self.prev_heading is None:
                smoothed_heading = candidate_heading
            else:
                smoothed_heading = self.alpha * candidate_heading + (1 - self.alpha) * self.prev_heading
            self.prev_heading = smoothed_heading
            smooth_angle = -((heading_sector - 1) * (270.0 / 89.0) - 135.0)
            # rospy.loginfo(f"!!!")
            turn_curvature = self.estimate_turn_curvature(
                smoothed_heading, lookahead_distance)
            self.linear_sim.input['Obstacle_Distance'] = obstacle_distance
            self.linear_sim.input['Turn_Curvature'] = turn_curvature
            self.linear_sim.compute()
            lin_vel = self.linear_sim.output['Linear_Velocity']

            self.angular_sim.input['Angular_Input'] = smoothed_heading
            self.angular_sim.compute()
            ang_vel = self.angular_sim.output['Angular_Output']

            twist = Twist()
            twist.linear.x  = lin_vel
            twist.angular.z = ang_vel
            # rospy.loginfo(f"计算速度")
            # Clear-Path Override & Safety Bubble 保持原样
            # if (np.all(front_readings >= self.max_range) and
            #     abs(smoothed_heading - heading_sector) < 5):
            #     twist.linear.x = 0.5

            # half_width = self.safety_bubble_width // 2
            # start_idx = max(0, center_index - half_width)
            # end_idx   = min(num_beams, center_index + half_width + 1)
            # safety_readings = self.processed_lidar_ranges[start_idx:end_idx]
            # if np.any(safety_readings < self.safety_distance):
            #     twist.linear.x = 0.0
            #     mid = len(safety_readings) // 2
            #     left_clearance  = np.min(safety_readings[:mid]) if mid > 0 else self.max_range
            #     right_clearance = np.min(safety_readings[mid:]) if mid < len(safety_readings) else self.max_range
            #     if left_clearance > right_clearance:
            #         twist.angular.z = 0.5
            #         twist.linear.x  = -0.4
            #     else:
            #         twist.angular.z = -0.5
            #         twist.linear.x  = -0.4
            # dx = self.target_absolute_position[0] - self.current_position[0]
            # dy = self.target_absolute_position[1] - self.current_position[1]
            
            # # 计算目标方向的角度（角度制，与current_heading统一）
            # target_heading_deg = np.degrees(np.arctan2(dy, dx))
            
            # # 计算与当前航向的夹角差，并归一化到 [-180, 180]
            # heading_diff_deg = target_heading_deg - self.current_heading
            # heading_diff_deg = (heading_diff_deg + 180) % 360 - 180  # 归一化到 [-180, 180]
            # heading_diff_deg = abs(heading_diff_deg)
        
            # rospy.loginfo("[Safety Check] target_heading: %.1f°, current_heading: %.1f°, heading_diff: %.1f°", target_heading_deg, self.current_heading, heading_diff_deg)
            
            # if heading_diff_deg > 30.0:
            #     rospy.logwarn("[Safety Check] Heading diff %.1f° > 60°, performing in-place rotation", heading_diff_deg)
                
            #     # 停止前进，原地旋转对准目标
            #     twist.linear.x = 0.0
                
            #     # 根据夹角符号判断转向方向（使用未取绝对值的heading_diff）
            #     heading_diff_raw = target_heading_deg - self.current_heading
            #     heading_diff_raw = (heading_diff_raw + 180) % 360 - 180
                
            #     if heading_diff_raw > 0:
            #         twist.angular.z = 1.0
            #         rospy.loginfo("[Safety Check] Rotating LEFT to target")
            #     else:
            #         twist.angular.z = 1.0
            #         rospy.loginfo("[Safety Check] Rotating RIGHT to target")
            if self.processed_lidar_ranges is not None:

                self.publish_vfh_markers(h, hp, hb, valleys,
                                        heading_sector, smoothed_heading,
                                        self.target_absolute_position)
            twist = self.apply_acceleration_limits(twist)
            self.cmd_vel_pub.publish(twist)
            rate.sleep()


if __name__ == '__main__':
    try:
        rospy.loginfo("====START NAVIGATION====")
        rospy.loginfo("Waiting for 2D Nav Goal from RViz…")
        node = NavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
