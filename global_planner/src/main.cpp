#include <ros/ros.h>
#include "astar_planner.hpp"

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "astar_planner");
    
    astar_planner::AStarPlanner node;
    
    ROS_INFO("Starting A* Planner for Navigation...");
    
    try {
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception caught: %s", e.what());
    }
    
    return 0;
}