#include "astar_planner.hpp"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <boost/bind.hpp>  // 用于绑定回调
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace astar_planner
{
// ... (AStarNodeHash, AStarNodeEqual, PairHash 保持不变)
// 用于AStarNode的哈希和比较
struct AStarNodeHash {
    std::size_t operator()(const AStarNode& node) const {
        return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
    }
};

struct AStarNodeEqual {
    bool operator()(const AStarNode& lhs, const AStarNode& rhs) const {
        return lhs.x == rhs.x && lhs.y == rhs.y;
    }
};

// 用于pair的哈希
struct PairHash {
    std::size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

AStarPlanner::AStarPlanner() : 
    nh_("~"),  // NodeHandle 初始化
    tf_listener_(ros::Duration(10))  // TF Listener 在初始化列表中直接构造（缓存10秒）
{
    // 获取参数（ROS1风格：nh.param自动声明并获取）
    nh_.param("cost_threshold", cost_threshold_, 50.0);
    nh_.param("use_diagonal_movement", use_diagonal_movement_, true);
    nh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    nh_.param("global_frame", global_frame_, std::string("map"));
    double planning_frequency;
    nh_.param("planning_frequency", planning_frequency, 2.0);
    
    // 车辆尺寸参数
    nh_.param("vehicle_width", vehicle_width_, 0.6);
    nh_.param("vehicle_length", vehicle_length_, 0.8);
    nh_.param("footprint_padding", footprint_padding_, 1);
    
    // 创建订阅者 - 订阅静态地图
    costmap_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
        "/esdf_map", 10, &AStarPlanner::costmapCallback, this);
    
    // 订阅rviz的2D Nav Goal（修正为标准主题）
    goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 10, &AStarPlanner::goalCallback, this);

    // 订阅初始位置（可选）
    initial_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/initialpose", 10, &AStarPlanner::initialPoseCallback, this);
    
    // 创建发布者
    path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 25);
    cancel_planner_flag_ = false;
    
    // 创建定时器用于定期重新规划
    ros::Duration timer_period(1.0 / planning_frequency);  // ROS1 Duration
    planning_timer_ = nh_.createTimer(
        timer_period, &AStarPlanner::planningTimerCallback, this);  // 无需bind，ROS1直接传this
    
    ROS_INFO("A* Planner initialized");
    ROS_INFO("Listening to:");
    ROS_INFO("  - Static map: /esdf_map");
    ROS_INFO("  - Goal from rviz: /move_base_simple/goal");
    ROS_INFO("  - Robot pose via TF: %s -> %s", global_frame_.c_str(), robot_base_frame_.c_str());
    ROS_INFO("Cost threshold: %.1f", cost_threshold_);
    ROS_INFO("Vehicle size: %.2fx%.2f meters, padding: %d cells", 
             vehicle_width_, vehicle_length_, footprint_padding_);
}

AStarPlanner::~AStarPlanner()
{
    // ROS1中无需特殊清理，NodeHandle析构时自动清理订阅/发布
}
void AStarPlanner::pub_path(){
if(!current_path.empty()){
    publishPath(current_path);
}
}

void AStarPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    bool is_first_map = (costmap_.data.empty());
    
    // 直接复制地图数据
    costmap_ = *msg;
    
    // ========== 日志输出 ==========
    double map_width_world = msg->info.width * msg->info.resolution;
    double map_height_world = msg->info.height * msg->info.resolution;
    double map_min_x = msg->info.origin.position.x;
    double map_max_x = map_min_x + map_width_world;
    double map_min_y = msg->info.origin.position.y;
    double map_max_y = map_min_y + map_height_world;
    
    if (is_first_map) {
        ROS_INFO("Received first ESDF map:");
        ROS_INFO("  Size: %dx%d cells (%.2fx%.2f meters)",
                msg->info.width, msg->info.height, map_width_world, map_height_world);
        ROS_INFO("  Resolution: %.3f m/cell", msg->info.resolution);
        ROS_INFO("  Origin: (%.2f, %.2f)", 
                msg->info.origin.position.x, msg->info.origin.position.y);
        ROS_INFO("  World bounds: X[%.2f -> %.2f], Y[%.2f -> %.2f]",
                map_min_x, map_max_x, map_min_y, map_max_y);
        ROS_INFO("  ESDF values: 100=obstacle, <100=free (smaller = safer)");
    } else {
        ROS_DEBUG("Updated ESDF map: %dx%d, resolution: %.3f, origin: (%.2f, %.2f)", 
                msg->info.width, msg->info.height, msg->info.resolution,
                msg->info.origin.position.x, msg->info.origin.position.y);
    }
}

void AStarPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    cancel_planner_flag_ = false;
    current_goal_ = *msg;
    ROS_INFO("Received new goal from rviz: (%.2f, %.2f)", 
            msg->pose.position.x, msg->pose.position.y);
    
    // 立即触发一次路径规划 - 只检查地图和目标
    if (!costmap_.data.empty() && !current_goal_.header.frame_id.empty()) {
        performPathPlanning();
    }
}

void AStarPlanner::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("Robot initial pose updated: (%.2f, %.2f)", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);
}

// 移除里程计回调函数 odomCallback

void AStarPlanner::planningTimerCallback(const ros::TimerEvent& event)
{
    // 检查是否有必要的数据
    if (costmap_.data.empty() || current_goal_.header.frame_id.empty()) {
        if (costmap_.data.empty()) {
            ROS_DEBUG("Waiting for map...");
        }
        if (current_goal_.header.frame_id.empty()) {
            ROS_DEBUG("Waiting for goal...");
        }
        return;
    }
    
    // 如果当前没有路径，直接进行规划
    if (current_path.empty()) {
        ROS_DEBUG("No current path, performing initial planning");
        performPathPlanning();
        return;
    }
    
    // 检查当前路径是否有碰撞
    if (checkPathCollision()) {
        ROS_INFO("Path collision detected, replanning...");
        performPathPlanning();
    }
}

// 检查当前路径是否与地图发生碰撞
// bool AStarPlanner::checkPathCollision()
// {
//     if (current_path.empty() || costmap_.data.empty()) {
//         return true; // 没有路径或地图时认为有碰撞
//     }
    
//     for (const auto& node : current_path) {
//         // 检查路径上每个点是否有碰撞
//         if (checkCollision(node.x, node.y)) {
//             ROS_DEBUG("Collision detected at path point: grid(%d, %d)", 
//                     node.x, node.y);
//             return true;
//         }
//     }
    
//     return false; // 路径无碰撞
// }
bool AStarPlanner::checkPathCollision()
{
    // 边界条件：无路径或无地图，直接判定有碰撞
    if (current_path.empty() || costmap_.data.empty()) {
        ROS_WARN("checkPathCollision: Empty path or costmap!");
        return true; // 没有路径或地图时认为有碰撞（路径无效）
    }
    
    // 遍历路径上的每个节点，结合相邻节点确定车辆朝向，进行碰撞检测
    for (size_t i = 0; i < current_path.size(); ++i) {
        const auto& current_node = current_path[i];
        double yaw = 0.0; // 车辆朝向（偏航角，弧度制）
        
        // 1. 确定车辆当前朝向（偏航角）
        if (i < current_path.size() - 1) {
            // 有下一个路点：通过当前节点和下一个节点计算朝向
            const auto& next_node = current_path[i+1];
            int dx_grid = next_node.x - current_node.x;
            int dy_grid = next_node.y - current_node.y;
            yaw = std::atan2(static_cast<double>(dy_grid), static_cast<double>(dx_grid));
        } else if (i > 0) {
            // 最后一个路点：沿用前一个朝向
            const auto& prev_node = current_path[i-1];
            int dx_grid = current_node.x - prev_node.x;
            int dy_grid = current_node.y - prev_node.y;
            yaw = std::atan2(static_cast<double>(dy_grid), static_cast<double>(dx_grid));
        }
        // 若只有一个路点，yaw保持为0（x轴正方向）
        
        // 2. 计算车辆实际物理尺寸的半值（含安全缓冲）
        double half_width = (vehicle_width_ / 2.0) + footprint_padding_;
        double half_length = (vehicle_length_ / 2.0) + footprint_padding_;
        double resolution = costmap_.info.resolution;
        
        // 3-4. 计算车辆四个边角的栅格坐标
        std::vector<std::pair<int, int>> vehicle_corners;
        std::vector<std::pair<double, double>> corner_offsets_physical = {
            { half_length,  half_width },   // 左前
            { half_length, -half_width },   // 右前
            { -half_length,  half_width },  // 左后
            { -half_length, -half_width }   // 右后
        };
        
        for (const auto& offset : corner_offsets_physical) {
            double x_phys = offset.first;
            double y_phys = offset.second;
            
            // 坐标旋转
            double x_rot_phys = x_phys * std::cos(yaw) - y_phys * std::sin(yaw);
            double y_rot_phys = x_phys * std::sin(yaw) + y_phys * std::cos(yaw);
            
            // 物理偏移转栅格偏移
            int x_rot_grid = static_cast<int>(std::round(x_rot_phys / resolution));
            int y_rot_grid = static_cast<int>(std::round(y_rot_phys / resolution));
            
            // 计算最终边角栅格坐标
            int corner_x = current_node.x + x_rot_grid;
            int corner_y = current_node.y + y_rot_grid;
            
            vehicle_corners.emplace_back(corner_x, corner_y);
        }
        
        // 5. 检查四个边角的栅格是否存在碰撞
        for (const auto& corner : vehicle_corners) {
            int corner_x = corner.first;
            int corner_y = corner.second;
            
            // 检查边界和障碍物
            if (!isValidCell(corner_x, corner_y)) {
                ROS_DEBUG("Collision detected at path point %zu: center(%d, %d), corner(%d, %d)", 
                        i, current_node.x, current_node.y, corner_x, corner_y);
                return true; // 🔴 检测到碰撞，返回true
            }
        }
    }
    
    // ✅ 修复：所有路径点检查通过，返回false表示无碰撞
    return false;
}

void AStarPlanner::performPathPlanning()
{
    // 通过TF获取当前机器人位置
    geometry_msgs::PoseStamped current_pose;
    try {
        current_pose = getCurrentPose();
    } catch (const std::exception& e) {
        ROS_WARN("Failed to get robot pose from TF: %s", e.what());  // 直接用 e.what()，无需 .c_str()
        return;
    }
    
    // 转换为栅格坐标
    auto start_grid = worldToGrid(current_pose.pose.position.x, current_pose.pose.position.y);
    auto goal_grid = worldToGrid(current_goal_.pose.position.x, current_goal_.pose.position.y);
    
    // 使用碰撞检测验证起点和终点
    // if (checkCollision(start_grid.first, start_grid.second)) {
    //     ROS_WARN("Robot start position has collision! world(%.2f,%.2f) -> grid(%d,%d). Searching for nearby valid cell...", 
    //              current_pose.pose.position.x, current_pose.pose.position.y,
    //              start_grid.first, start_grid.second);
        
    //     auto nearest_start = findNearestValidCell(start_grid.first, start_grid.second);
    //     if (nearest_start.first == -1) {
    //         ROS_ERROR("Cannot find valid start position nearby!");
    //         return;
    //     }
    //     start_grid = nearest_start;
    //     ROS_INFO("Using nearby valid start position: grid(%d,%d)", 
    //              start_grid.first, start_grid.second);
    // }
    
    if (checkCollision(goal_grid.first, goal_grid.second)) {
        ROS_WARN("Goal position has collision! world(%.2f,%.2f) -> grid(%d,%d). Searching for nearby valid cell...", 
                 current_goal_.pose.position.x, current_goal_.pose.position.y,
                 goal_grid.first, goal_grid.second);
        
        auto nearest_goal = findNearestValidCell(goal_grid.first, goal_grid.second);
        if (nearest_goal.first == -1) {
            ROS_ERROR("Cannot find valid goal position nearby!");
            return;
        }
        goal_grid = nearest_goal;
        ROS_INFO("Using nearby valid goal position: grid(%d,%d)", 
                 goal_grid.first, goal_grid.second);
    }
    
    
    AStarNode start_node(start_grid.first, start_grid.second);
    AStarNode goal_node(goal_grid.first, goal_grid.second);
    
    // 执行A*搜索
    auto start_time = std::chrono::high_resolution_clock::now();
    current_path.clear();
    auto path = findPath(start_node, goal_node);
    current_path = path;
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (!path.empty()) {
        publishPath(path);
        ROS_INFO("New path found with %zu waypoints (planning time: %ld ms)", 
                 path.size(), duration.count());
        ROS_DEBUG("Robot: world(%.2f,%.2f)->grid(%d,%d), Goal: world(%.2f,%.2f)->grid(%d,%d)",
                  current_pose.pose.position.x, current_pose.pose.position.y, start_grid.first, start_grid.second,
                  current_goal_.pose.position.x, current_goal_.pose.position.y, goal_grid.first, goal_grid.second);
    } else {
        ROS_WARN("No path found from (%d,%d) to (%d,%d) (planning time: %ld ms)",
                 start_grid.first, start_grid.second,
                 goal_grid.first, goal_grid.second, duration.count());
    }
}

geometry_msgs::PoseStamped AStarPlanner::getCurrentPose()
{
    tf::StampedTransform transform;  // 使用 tf::StampedTransform 而非 geometry_msgs::TransformStamped
    
    try {
        // 获取最新可用的变换（ROS1: lookupTransform with ros::Time(0) for latest）
        tf_listener_.lookupTransform(
            global_frame_, robot_base_frame_, ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        throw std::runtime_error("TF lookup failed (" + global_frame_ + " -> " + robot_base_frame_ + "): " + std::string(ex.what()));
    }
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = ros::Time::now();
    
    // 从 tf::StampedTransform 提取位置和姿态
    pose.pose.position.x = transform.getOrigin().x();
    pose.pose.position.y = transform.getOrigin().y();
    pose.pose.position.z = transform.getOrigin().z();
    pose.pose.orientation.x = transform.getRotation().x();
    pose.pose.orientation.y = transform.getRotation().y();
    pose.pose.orientation.z = transform.getRotation().z();
    pose.pose.orientation.w = transform.getRotation().w();
    
    return pose;
}
std::pair<int, int> AStarPlanner::findNearestValidCell(int target_x, int target_y, int max_search_radius)
{
    // 首先检查目标点本身是否可通行
    if (!checkCollision(target_x, target_y)) {
        return std::make_pair(target_x, target_y);
    }
    
    // 螺旋式搜索附近的可通行点
    for (int radius = 1; radius <= max_search_radius; ++radius) {
        // 搜索当前半径圆圈上的所有点
        for (int dx = -radius; dx <= radius; ++dx) {
            for (int dy = -radius; dy <= radius; ++dy) {
                // 只检查距离正好为当前半径的点（避免重复检查）
                if (std::abs(dx) == radius || std::abs(dy) == radius) {
                    int check_x = target_x + dx;
                    int check_y = target_y + dy;
                    
                    // 检查边界
                    if (check_x >= 0 && check_x < static_cast<int>(costmap_.info.width) &&
                        check_y >= 0 && check_y < static_cast<int>(costmap_.info.height)) {
                        
                        // 检查是否可通行（无碰撞）
                        if (!checkCollision(check_x, check_y)) {
                            double distance = std::sqrt(dx * dx + dy * dy);
                            auto world_pos = gridToWorld(check_x, check_y);
                            auto orig_world_pos = gridToWorld(target_x, target_y);
                            
                            ROS_INFO("Found valid cell at grid(%d,%d) world(%.2f,%.2f), "
                                     "distance %.2f cells from original grid(%d,%d) world(%.2f,%.2f)",
                                     check_x, check_y, world_pos.first, world_pos.second,
                                     distance, target_x, target_y, 
                                     orig_world_pos.first, orig_world_pos.second);
                            
                            return std::make_pair(check_x, check_y);
                        }
                    }
                }
            }
        }
    }
    
    // 没有找到可通行的点
    ROS_WARN("No valid cell found within %d cells radius of grid(%d,%d)",
             max_search_radius, target_x, target_y);
    return std::make_pair(-1, -1);
}

std::pair<int, int> AStarPlanner::worldToGrid(double x, double y)
{
    if (costmap_.data.empty()) {  // 检查地图是否可用
        ROS_WARN("Costmap not available for coordinate conversion");
        return std::make_pair(-1, -1);
    }
    
    int grid_x = static_cast<int>((x - costmap_.info.origin.position.x) / costmap_.info.resolution);
    int grid_y = static_cast<int>((y - costmap_.info.origin.position.y) / costmap_.info.resolution);
    
    return std::make_pair(grid_x, grid_y);
}

std::pair<double, double> AStarPlanner::gridToWorld(int grid_x, int grid_y)
{
    if (costmap_.data.empty()) {  // 检查地图是否可用
        ROS_WARN("Costmap not available for coordinate conversion");
        return std::make_pair(0.0, 0.0);
    }
    
    double x = grid_x * costmap_.info.resolution + costmap_.info.origin.position.x + costmap_.info.resolution * 0.5;
    double y = grid_y * costmap_.info.resolution + costmap_.info.origin.position.y + costmap_.info.resolution * 0.5;
    
    return std::make_pair(x, y);
}


bool AStarPlanner::checkCollision(int center_x, int center_y)
{
    if (costmap_.data.empty()) {
        return true;  // 没有地图就认为有碰撞
    }
    
    // 根据车辆尺寸和地图分辨率计算要检查的栅格范围
    int radius_x = static_cast<int>(std::ceil(vehicle_width_ / (2.0 * costmap_.info.resolution))) + footprint_padding_;
    int radius_y = static_cast<int>(std::ceil(vehicle_length_ / (2.0 * costmap_.info.resolution))) + footprint_padding_;
    
    // 检查车辆占用的所有栅格
    for (int dx = -radius_x; dx <= radius_x; ++dx) {
        for (int dy = -radius_y; dy <= radius_y; ++dy) {
            int check_x = center_x + dx;
            int check_y = center_y + dy;
            if (!isValidCell(check_x, check_y)) {
                return true;  // 发现碰撞
            }
        }
    }
    
    return false;  // 无碰撞
}

// bool AStarPlanner::isValidCell(int x, int y)
// {
//     // 检查边界
//     if (x < 0 || x >= static_cast<int>(costmap_.info.width) || 
//         y < 0 || y >= static_cast<int>(costmap_.info.height)) {
//         return false;
//     }
    
//     // 检查障碍物
//     int index = y * costmap_.info.width + x;
//     if (index >= 0 && index < static_cast<int>(costmap_.data.size())) {
//         int8_t cell_value = costmap_.data[index];
    
//         if (cell_value == -1) {
//             // 未知区域：允许通过
//             return true;
//         } else if (cell_value == 0) {
//             // 空闲空间：允许通过
//             return true;
//         } else if (cell_value > 0 && cell_value < cost_threshold_) {
//             // 低概率占用区域：允许通过
//             return true;
//         } else {
//             // 高概率占用或障碍物：不允许通过
//             return false;
//         }
//     }
    
//     return false;
// }

bool AStarPlanner::isValidCell(int x, int y)
{
    // 边界检查
    if (x < 0 || x >= static_cast<int>(costmap_.info.width) ||
        y < 0 || y >= static_cast<int>(costmap_.info.height)) {
        return false;
    }

    int index = y * costmap_.info.width + x;
    int8_t cell_value = costmap_.data[index];

    // ESDF 地图语义：0~99 表示安全程度（值越小越安全），100 表示障碍
    // 只有 cell_value < cost_threshold_ 的栅格才视为可通行
    return cell_value < cost_threshold_;
}

double AStarPlanner::calculateHeuristic(const AStarNode& a, const AStarNode& b)
{
    if (use_diagonal_movement_) {
        double dx = std::abs(a.x - b.x);
        double dy = std::abs(a.y - b.y);
        return std::sqrt(dx * dx + dy * dy);
    } else {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y);
    }
}

double AStarPlanner::calculateMoveCost(const AStarNode& from, const AStarNode& to)
{
    double dx = std::abs(from.x - to.x);
    double dy = std::abs(from.y - to.y);
    
    double base_cost;
    if (dx == 1 && dy == 1) {
        base_cost = 1.414; // 对角线移动
    } else {
        base_cost = 1.0; // 水平或垂直移动
    }
    
    // 考虑地图代价
    int index = to.y * costmap_.info.width + to.x;
    if (index >= 0 && index < static_cast<int>(costmap_.data.size())) {
        double cell_cost = costmap_.data[index] / 100.0;
        if (cell_cost < 0) cell_cost = 0;
        base_cost *= (1.0 + cell_cost);
    }
    
    return base_cost;
}   
std::vector<AStarNode> AStarPlanner::findPath(const AStarNode& start, const AStarNode& goal)
{
    std::priority_queue<AStarNode> open_set;
    std::unordered_set<AStarNode, AStarNodeHash, AStarNodeEqual> closed_set;
    std::unordered_map<std::pair<int, int>, std::unique_ptr<AStarNode>, PairHash> node_map;
    
    // 检查起点和终点
    if (!isValidCell(start.x, start.y) || !isValidCell(goal.x, goal.y)) {
        ROS_WARN("Start or goal position has collision");
        return {};
    }
    
    // 初始化起始节点
    auto start_key = std::make_pair(start.x, start.y);
    node_map[start_key] = std::make_unique<AStarNode>(start.x, start.y);
    node_map[start_key]->g_cost = 0;
    node_map[start_key]->h_cost = calculateHeuristic(*node_map[start_key], goal);
    node_map[start_key]->f_cost = node_map[start_key]->g_cost + node_map[start_key]->h_cost;
    
    open_set.push(*node_map[start_key]);
    
    int iterations = 0;
    const int max_iterations = costmap_.info.width * costmap_.info.height;
    
    while (!open_set.empty() && iterations < max_iterations) {
        iterations++;
        
        AStarNode current = open_set.top();
        open_set.pop();
        
        if (closed_set.find(current) != closed_set.end()) {
            continue;
        }
        
        closed_set.insert(current);
        
        // 检查是否到达目标
        if (current.x == goal.x && current.y == goal.y) {
            auto goal_key = std::make_pair(goal.x, goal.y);
            if (node_map.find(goal_key) != node_map.end()) {
                return reconstructPath(node_map[goal_key].get());
            }
        }
        
        // 检查邻居
        auto neighbors = getNeighbors(current);
        
        for (const auto& neighbor : neighbors) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            auto neighbor_key = std::make_pair(neighbor.x, neighbor.y);
            auto current_key = std::make_pair(current.x, current.y);
            double tentative_g_cost = node_map[current_key]->g_cost + 
                                    calculateMoveCost(current, neighbor);
            
            bool is_better = false;
            if (node_map.find(neighbor_key) == node_map.end()) {
                node_map[neighbor_key] = std::make_unique<AStarNode>(neighbor.x, neighbor.y);
                is_better = true;
            } else if (tentative_g_cost < node_map[neighbor_key]->g_cost) {
                is_better = true;
            }
            
            if (is_better) {
                node_map[neighbor_key]->parent = node_map[current_key].get();
                node_map[neighbor_key]->g_cost = tentative_g_cost;
                node_map[neighbor_key]->h_cost = calculateHeuristic(*node_map[neighbor_key], goal);
                node_map[neighbor_key]->f_cost = node_map[neighbor_key]->g_cost + 
                                               node_map[neighbor_key]->h_cost;
                
                open_set.push(*node_map[neighbor_key]);
            }
        }
    }
    
    return {}; // 没有找到路径
}

// 重建路径：从目标节点回溯到起点
std::vector<AStarNode> AStarPlanner::reconstructPath(AStarNode* goal_node)
{
    std::vector<AStarNode> path;
    AStarNode* current = goal_node;
    
    while (current != nullptr) {
        path.push_back(*current);
        current = current->parent;
    }
    
    // 反转路径（从起点到终点）
    std::reverse(path.begin(), path.end());
    
    ROS_DEBUG("Reconstructed path with %zu nodes", path.size());
    return path;
}

// 获取当前节点的邻居（8方向或4方向，取决于 use_diagonal_movement_）
std::vector<AStarNode> AStarPlanner::getNeighbors(const AStarNode& current)
{
    std::vector<AStarNode> neighbors;
    
    // 4方向（上下左右）+ 可选对角线
    std::vector<std::pair<int, int>> directions;
    directions.emplace_back(0, 1);   // 下
    directions.emplace_back(1, 0);   // 右
    directions.emplace_back(0, -1);  // 上
    directions.emplace_back(-1, 0);  // 左
    
    if (use_diagonal_movement_) {
        directions.emplace_back(1, 1);   // 右下
        directions.emplace_back(1, -1);  // 右上
        directions.emplace_back(-1, 1);  // 左下
        directions.emplace_back(-1, -1); // 左上
    }
    
    for (const auto& dir : directions) {
        int nx = current.x + dir.first;
        int ny = current.y + dir.second;
        
        // 检查有效性和无碰撞
        if (isValidCell(nx, ny) && !checkCollision(nx, ny)) {
            neighbors.emplace_back(nx, ny);
        }
    }
    
    ROS_DEBUG("Generated %zu valid neighbors for node (%d, %d)", neighbors.size(), current.x, current.y);
    return neighbors;
}

void AStarPlanner::publishPath(const std::vector<AStarNode>& path)
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = global_frame_;
    
    for (size_t i = 0; i < path.size(); ++i) {
        const auto& node = path[i];
        auto world_pos = gridToWorld(node.x, node.y);
        
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = world_pos.first;
        pose_stamped.pose.position.y = world_pos.second;
        pose_stamped.pose.position.z = 0.0;

        // 如果是路径的最后一个点，使用目标的位姿
        if (i == path.size() - 1 && !current_goal_.header.frame_id.empty()) {
            // 使用目标点的位姿
            pose_stamped.pose.orientation = current_goal_.pose.orientation;
            ROS_DEBUG("Setting goal orientation for final waypoint: qx=%.3f, qy=%.3f, qz=%.3f, qw=%.3f",
                      current_goal_.pose.orientation.x, current_goal_.pose.orientation.y,
                      current_goal_.pose.orientation.z, current_goal_.pose.orientation.w);
        } else {
            // 中间路径点保持默认位姿
            pose_stamped.pose.orientation.w = 1.0;
        }        
        path_msg.poses.push_back(pose_stamped);
    }
    
    path_pub_.publish(path_msg);
}
}