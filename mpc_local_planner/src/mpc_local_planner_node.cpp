#include "mpc_local_planner/mpc_local_planner.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_local_planner");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  
  ROS_INFO("Starting MPC Local Planner Node...");
  
  mpc_local_planner::MPCLocalPlanner planner;
  
  if (!planner.initialize(nh, nh_private)) {
    ROS_ERROR("Failed to initialize MPC Local Planner!");
    return -1;
  }
  
  planner.run();
  
  return 0;
}
