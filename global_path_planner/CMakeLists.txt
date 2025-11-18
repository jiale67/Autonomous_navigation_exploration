cmake_minimum_required(VERSION 3.0.2)
project(global_path_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  octomap_ros
  pcl_ros
  pcl_conversions
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)  # 添加Eigen3依赖

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp rospy std_msgs nav_msgs sensor_msgs visualization_msgs 
    tf2 tf2_ros tf2_geometry_msgs octomap_ros pcl_ros pcl_conversions
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}  # 添加Eigen头文件路径
  ${PCL_INCLUDE_DIRS}
)





## 添加可执行文件
add_executable(rrt_interactive_node src/rrt_interactive_node.cpp)
add_executable(rrt_star_interactive_node src/rrt_star_interactive_node.cpp)
add_executable(astar_interactive_node src/astar_interactive_node.cpp)


## 链接库
target_link_libraries(rrt_interactive_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(rrt_star_interactive_node 
  ${catkin_LIBRARIES}
)

target_link_libraries(astar_interactive_node 
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)