#include "dwa_local_planner/dwa_local_planner.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dwa_local_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  ROS_INFO("Starting DWA Local Planner Node...");
  
  dwa_local_planner::DWALocalPlanner planner;
  
  if (!planner.initialize(nh, nh_private)) {
    ROS_ERROR("Failed to initialize DWA Local Planner!");
    return -1;
  }
  
  planner.run();
  
  return 0;
}









