cmake_minimum_required(VERSION 3.0.2)
project(global_planner)  # 匹配你的包名（从 package.xml）

## Compile as C++14 (for std::unique_ptr 等)
## add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf nav_msgs geometry_msgs std_msgs
  DEPENDS Boost
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable (包含所有源文件)
add_executable(astar_planner_node 
  src/main.cpp 
  src/astar_planner.cpp  # 添加 AStarPlanner 的实现源文件
)

## Specify libraries to link
target_link_libraries(astar_planner_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)