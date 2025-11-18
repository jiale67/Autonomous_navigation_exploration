/*
*	File: density_exploration_node.cpp
*	---------------
*   基于密度的探索节点
*   Created by Assistant on 2025.10
*/

#include <ros/ros.h>
#include "global_path_planner/density_frontier_explorer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "density_exploration_node");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("   基于密度的前沿探索系统");
    ROS_INFO("   使用 /terrain_map_ext 点云");
    ROS_INFO("========================================");
    
    try {
        exploration::DensityFrontierExplorer explorer(nh);
        
        ROS_INFO("✓ 探索节点初始化成功!");
        ROS_INFO("等待 /state_estimation 和 /terrain_map_ext...");
        
        explorer.run();
        
    } catch (const std::exception& e) {
        ROS_FATAL("✗ 探索器初始化失败: %s", e.what());
        return 1;
    }
    
    return 0;
}



