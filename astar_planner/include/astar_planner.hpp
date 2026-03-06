#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>

#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <boost/shared_ptr.hpp>  // 用于ConstPtr等

namespace astar_planner
{

// 重命名为AStarNode避免与rclcpp::Node冲突
struct AStarNode
{
    int x, y;
    double g_cost;
    double h_cost;
    double f_cost;
    AStarNode* parent;
    
    AStarNode(int x = 0, int y = 0) : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    
    bool operator<(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

class AStarPlanner
{
public:
    AStarPlanner();
    ~AStarPlanner();

private:
    // NodeHandle
    ros::NodeHandle nh_;
    
    // 回调函数（ROS1签名：使用ConstPtr）
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void cancelCallback(const std_msgs::Bool::ConstPtr& msg);
    
    // A*算法核心函数
    std::vector<AStarNode> findPath(const AStarNode& start, const AStarNode& goal);
    bool isValidCell(int x, int y);
    double calculateHeuristic(const AStarNode& a, const AStarNode& b);
    double calculateMoveCost(const AStarNode& from, const AStarNode& to);
    std::vector<AStarNode> getNeighbors(const AStarNode& current);
    std::vector<AStarNode> reconstructPath(AStarNode* goal_node);
    void pub_path();
    std::vector<AStarNode> current_path;
    // 简单的碰撞检测函数
    bool checkCollision(int center_x, int center_y);

    std::pair<int, int> findNearestValidCell(int target_x, int target_y, int max_search_radius = 10);
    
    // 坐标转换
    std::pair<int, int> worldToGrid(double x, double y);
    std::pair<double, double> gridToWorld(int grid_x, int grid_y);
    geometry_msgs::PoseStamped getCurrentPose();
    
    // 路径发布
    void publishPath(const std::vector<AStarNode>& path);
    
    // 订阅者
    ros::Subscriber costmap_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Subscriber cancel_sub_;

    // 发布者
    ros::Publisher path_pub_;
    
    // TF
    tf::TransformListener tf_listener_;
    
    // 数据存储
    nav_msgs::OccupancyGrid costmap_;  // 直接存储拷贝（非Ptr）
    geometry_msgs::PoseStamped current_goal_;  // 直接存储拷贝
    
    // 参数
    double cost_threshold_;
    bool use_diagonal_movement_;
    std::string robot_base_frame_;
    std::string global_frame_;
    bool cancel_planner_flag_;
    
    // 车辆尺寸参数
    double vehicle_width_;      // 车辆宽度（米）
    double vehicle_length_;     // 车辆长度（米）
    int footprint_padding_;     // 安全边距（栅格数）
    
    // 定时器用于定期重新规划
    ros::Timer planning_timer_;
    void planningTimerCallback(const ros::TimerEvent& event);

    void performPathPlanning();
    bool checkPathCollision();

};

} // namespace astar_planner

#endif // ASTAR_PLANNER_HPP