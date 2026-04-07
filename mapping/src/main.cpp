#include <ros/ros.h>

#include "mapping/map_ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aaamapping_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  narrow_nav::MapRosNode node(nh, pnh);
  if (node.init() == false)
  {
    ROS_ERROR("Failed to initialize aaamapping node.");
    return 1;
  }

  node.spin();
  return 0;
}
