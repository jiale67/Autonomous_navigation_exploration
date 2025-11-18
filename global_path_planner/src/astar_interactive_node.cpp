/*
*	File: astar_interactive_node.cpp
*	---------------
*   Interactive node for A* global path planner
*   Created on 2025
*/
#include <ros/ros.h>
#include <global_path_planner/aStarOctomap.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <thread>

bool hasOdom = false;
std::vector<double> currentPosition {0, 0, 0};

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    currentPosition[0] = odom->pose.pose.position.x;
    currentPosition[1] = odom->pose.pose.position.y;
    currentPosition[2] = odom->pose.pose.position.z;
    hasOdom = true;
}

bool newGoalMsg = false;
std::vector<double> goalPoint {0, 0, 1.0};

void clickedPointCB(const geometry_msgs::PoseStamped::ConstPtr& cp){
    goalPoint[0] = cp->pose.position.x;
    goalPoint[1] = cp->pose.position.y;
    goalPoint[2] = cp->pose.position.z;
    newGoalMsg = true;
    ROS_INFO("[A* Node]: New goal received: (%.2f, %.2f, %.2f)", 
             goalPoint[0], goalPoint[1], goalPoint[2]);
}

ros::Publisher goalVisPub;
ros::Publisher globalpathPub;
bool initGoal = false;
visualization_msgs::Marker goalMarker;

void publishGoalVis(){
    ros::Rate r(10);
    while (ros::ok()){
        if (initGoal){
            goalVisPub.publish(goalMarker);
        }
        r.sleep();
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "astar_global_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    ROS_INFO("============================================");
    ROS_INFO("[A* Node]: A* Global Path Planner Started");
    ROS_INFO("============================================");

    // 订阅
    ros::Subscriber odomSub = nh.subscribe("/state_estimation", 1000, odomCallback);
    ros::Subscriber clickedPointSub = nh.subscribe("/move_base_simple/goal", 1000, clickedPointCB);
    
    // 发布
    goalVisPub = nh.advertise<visualization_msgs::Marker>("/astar/goal_marker", 10);
    globalpathPub = nh.advertise<nav_msgs::Path>("/astar/global_path", 10);
    
    // 启动目标可视化线程
    std::thread goalVisWorker = std::thread(publishGoalVis);
    goalVisWorker.detach();
    
    // 创建A*规划器
    globalPlanner::AStarOctomap astarPlanner(nh_private);
    
    ROS_INFO("[A* Node]: Waiting for octomap...");
    ros::Duration(2.0).sleep();
    
    int planCount = 0;
    ros::Rate r(10);
    
    while (ros::ok()){
        ros::spinOnce();
        if (!newGoalMsg) {
            r.sleep();
            continue;
        }
        
        ROS_INFO("----------------------------------------------------");
        ROS_INFO("[A* Node]: Planning Request #%d", ++planCount);
        
        if (!hasOdom) {
            ROS_WARN("[A* Node]: Waiting for odometry data...");
            while (ros::ok() && !hasOdom) {
                ros::spinOnce();
                r.sleep();
            }
        }
        
        std::vector<double> start = currentPosition;
        std::vector<double> goal = goalPoint;
        
        astarPlanner.updateStart(start);
        astarPlanner.updateGoal(goal);
        
        ROS_INFO("[A* Node]: Start: (%.2f, %.2f, %.2f)", start[0], start[1], start[2]);
        ROS_INFO("[A* Node]: Goal:  (%.2f, %.2f, %.2f)", goal[0], goal[1], goal[2]);
        
        // 可视化目标点
        initGoal = true;
        goalMarker.header.frame_id = "map";
        goalMarker.header.stamp = ros::Time::now();
        goalMarker.ns = "astar_goal";
        goalMarker.id = 0;
        goalMarker.type = visualization_msgs::Marker::SPHERE;
        goalMarker.action = visualization_msgs::Marker::ADD;
        goalMarker.pose.position.x = goal[0];
        goalMarker.pose.position.y = goal[1];
        goalMarker.pose.position.z = goal[2];
        goalMarker.scale.x = 0.5;
        goalMarker.scale.y = 0.5;
        goalMarker.scale.z = 0.5;
        goalMarker.color.a = 0.8;
        goalMarker.color.r = 0.0;
        goalMarker.color.g = 1.0;
        goalMarker.color.b = 1.0;
        goalMarker.lifetime = ros::Duration(0);
        
        // 执行路径规划
        nav_msgs::Path global_path;
        ros::Time planStartTime = ros::Time::now();
        
        bool success = astarPlanner.makePlan(global_path);
        
        double planningTime = (ros::Time::now() - planStartTime).toSec();
        
        if (success && !global_path.poses.empty()) {
            ROS_INFO("[A* Node]: ✓ Path found with %zu waypoints in %.3fs", 
                     global_path.poses.size(), planningTime);
            
            global_path.header.frame_id = "map";
            global_path.header.stamp = ros::Time::now();
            globalpathPub.publish(global_path);
            
            // 计算路径长度
            double pathLength = 0.0;
            for (size_t i = 1; i < global_path.poses.size(); ++i) {
                double dx = global_path.poses[i].pose.position.x - global_path.poses[i-1].pose.position.x;
                double dy = global_path.poses[i].pose.position.y - global_path.poses[i-1].pose.position.y;
                double dz = global_path.poses[i].pose.position.z - global_path.poses[i-1].pose.position.z;
                pathLength += std::sqrt(dx*dx + dy*dy + dz*dz);
            }
            ROS_INFO("[A* Node]: Path length: %.2fm", pathLength); 
            // 提取关键拐点并发布MarkerArray给local_planner
            std::vector<globalPlanner::Point3D> fullPath;
            for (const auto& pose : global_path.poses) {
                fullPath.emplace_back(pose.pose.position.x, 
                                     pose.pose.position.y, 
                                     pose.pose.position.z);
            }
            
            std::vector<globalPlanner::Point3D> keyWaypoints = astarPlanner.extractKeyWaypoints(fullPath);
            
            // 将最后一个航点替换为精确的点击终点，确保导航精准
            if (!keyWaypoints.empty()) {
                keyWaypoints.back() = globalPlanner::Point3D(goalPoint[0], goalPoint[1], goalPoint[2]);
                ROS_INFO("[A* Node]: Replaced last waypoint with exact goal (%.2f, %.2f, %.2f)", 
                         goalPoint[0], goalPoint[1], goalPoint[2]);
            }
            
            astarPlanner.publishWaypointsAsMarkerArray(keyWaypoints);
        } else {
            ROS_ERROR("[A* Node]: ✗ Path planning failed!");
        }
        
        newGoalMsg = false;
        ROS_INFO("----------------------------------------------------");
    }
    
    return 0;
}

