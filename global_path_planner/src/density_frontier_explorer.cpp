/*
*	File: density_frontier_explorer.cpp
*	---------------
*   基于点云密度的前沿探索实现
*   Created by Assistant on 2025.10
*/

#include "global_path_planner/density_frontier_explorer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <algorithm>
#include <cmath>

namespace exploration {

DensityFrontierExplorer::DensityFrontierExplorer(ros::NodeHandle& nh)
    : nh_(nh),
      terrain_cloud_(new pcl::PointCloud<pcl::PointXYZI>()),
      odom_received_(false),
      cloud_received_(false),
      has_goal_(false),
      goal_reached_(false) {
    
    // 加载参数 - 扇区划分
    nh_.param<int>("num_sectors", num_sectors_, 36);
    nh_.param<double>("min_sector_radius", min_sector_radius_, 2.0);
    nh_.param<double>("max_sector_radius", max_sector_radius_, 12.0);
    nh_.param<double>("sector_radial_resolution", sector_radial_resolution_, 1.0);
    
    // 密度计算参数
    nh_.param<double>("density_radius", density_radius_, 1.0);
    nh_.param<int>("dense_threshold", dense_threshold_, 50);
    nh_.param<int>("sparse_threshold", sparse_threshold_, 10);
    
    // 目标选择参数
    nh_.param<double>("goal_reached_threshold", goal_reached_threshold_, 1.2);
    nh_.param<double>("min_goal_distance", min_goal_distance_, 3.0);
    nh_.param<double>("max_goal_distance", max_goal_distance_, 12.0);
    
    // 评分权重
    nh_.param<double>("weight_sparsity", weight_sparsity_, 0.5);
    nh_.param<double>("weight_distance", weight_distance_, 0.3);
    nh_.param<double>("weight_direction", weight_direction_, 0.2);
    
    nh_.param<double>("exploration_rate", exploration_rate_, 1.0);
    nh_.param<bool>("enable_visualization", enable_visualization_, true);
    nh_.param<bool>("use_global_path", use_global_path_, true);
    nh_.param<double>("goal_switch_distance", goal_switch_distance_, 2.0);  // 目标切换距离阈值
    nh_.param<double>("goal_reevaluation_interval", goal_reevaluation_interval_, 5.0);  // 5秒重新评估目标
    nh_.param<double>("path_replan_interval", path_replan_interval_, 3.0);  // 3秒重规划路径
    
    // RRT*参数
    nh_.param<double>("rrt_step_size", rrt_step_size_, 0.5);
    nh_.param<double>("rrt_goal_bias", rrt_goal_bias_, 0.1);
    nh_.param<int>("rrt_max_iterations", rrt_max_iterations_, 3000);
    nh_.param<double>("path_smoothing_factor", path_smoothing_factor_, 0.3);
    
    // 碰撞检测参数
    nh_.param<double>("collision_check_step", collision_check_step_, 0.1);
    nh_.param<double>("robot_radius", robot_radius_, 0.5);
    nh_.param<double>("obstacle_height_threshold", obstacle_height_threshold_, 0.25);
    nh_.param<double>("max_path_point_distance", max_path_point_distance_, 0.5);
    
    // 已探索区域记录参数
    nh_.param<double>("exploration_grid_resolution", exploration_grid_resolution_, 2.0);
    nh_.param<double>("exploration_radius", exploration_radius_, 3.0);
    
    // 订阅器
    odom_sub_ = nh_.subscribe("/state_estimation", 10,
                              &DensityFrontierExplorer::odomCallback, this);
    terrain_cloud_sub_ = nh_.subscribe("/terrain_map_ext", 10,
                                      &DensityFrontierExplorer::terrainCloudCallback, this);
    
    // 发布器
    frontier_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/frontier_candidates", 1);
    best_frontier_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/best_frontier", 1);
    exploration_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "/move_base_simple/goal", 1);
    density_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/density_visualization", 1);
    sector_lines_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/sector_lines", 1);
    global_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/global_planned_path", 1);
    
    // 定时器
    exploration_timer_ = nh_.createTimer(
        ros::Duration(1.0 / exploration_rate_),
        &DensityFrontierExplorer::explorationTimerCallback,
        this);
    
    robot_position_ = Eigen::Vector3d::Zero();
    robot_yaw_ = 0.0;
    current_goal_ = Eigen::Vector3d::Zero();
    last_goal_update_time_ = ros::Time::now();
    last_path_replan_time_ = ros::Time::now();
    
    ROS_INFO("========================================");
    ROS_INFO("[DensityFrontierExplorer] Density-based Exploration System Initialized!");
    ROS_INFO("[DensityFrontierExplorer] Number of sectors: %d", num_sectors_);
    ROS_INFO("[DensityFrontierExplorer] Search radius: %.1f - %.1f m", 
             min_sector_radius_, max_sector_radius_);
    ROS_INFO("[DensityFrontierExplorer] Density threshold: sparse<%d, dense>%d", 
             sparse_threshold_, dense_threshold_);
    ROS_INFO("[DensityFrontierExplorer] Use global path: %s", use_global_path_ ? "YES" : "NO");
    ROS_INFO("[DensityFrontierExplorer] Exploration grid resolution: %.1fm", exploration_grid_resolution_);
    ROS_INFO("[DensityFrontierExplorer] Exploration radius: %.1fm", exploration_radius_);
    ROS_INFO("========================================");
}

DensityFrontierExplorer::~DensityFrontierExplorer() {
}

void DensityFrontierExplorer::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_position_.x() = msg->pose.pose.position.x;
    robot_position_.y() = msg->pose.pose.position.y;
    robot_position_.z() = msg->pose.pose.position.z;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_yaw_);
    
    // 标记机器人当前位置及周围区域为已探索
    markRegionAsExplored(robot_position_);
    
    odom_received_ = true;
}

void DensityFrontierExplorer::terrainCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg) {
    
    terrain_cloud_->clear();
    pcl::fromROSMsg(*msg, *terrain_cloud_);
    
    if (!terrain_cloud_->empty()) {
        // 构建KD树
        kdtree_.setInputCloud(terrain_cloud_);
        cloud_received_ = true;
    }
}

void DensityFrontierExplorer::explorationTimerCallback(const ros::TimerEvent& event) {
    if (!odom_received_) {
        ROS_WARN_THROTTLE(5.0, "[DensityFrontierExplorer] Waiting for odometry data...");
        return;
    }
    
    if (!cloud_received_ || terrain_cloud_->empty()) {
        ROS_WARN_THROTTLE(5.0, "[DensityFrontierExplorer] Waiting for terrain_map_ext point cloud...");
        return;
    }
    
    // ===== 1. 持续发布当前路径和目标（实时更新） =====
    if (has_goal_ && !current_global_path_.empty()) {
        publishGlobalPath(current_global_path_);
        visualizeBestFrontier(current_goal_);
    }
    
    // 可视化扇区线
    if (enable_visualization_) {
        visualizeSectorLines();
    }
    
    ros::Time current_time = ros::Time::now();
    double time_since_goal_update = (current_time - last_goal_update_time_).toSec();
    double time_since_path_replan = (current_time - last_path_replan_time_).toSec();
    
    // ===== 2. 判断是否需要更新目标 =====
    bool should_update_goal = false;
    bool should_replan_path = false;
    std::string update_reason = "";
    
    if (!has_goal_) {
        // Case 1: No goal, update immediately
        should_update_goal = true;
        update_reason = "No goal";
        ROS_DEBUG("[DensityFrontierExplorer] No goal, searching for new goal");
    } else {
        // 有目标的情况下，检查多个条件
        double distance_to_goal = getDistance2D(robot_position_, current_goal_);
        
        if (distance_to_goal < goal_reached_threshold_) {
            // Case 2: Goal reached
            ROS_INFO("[DensityFrontierExplorer] Goal reached! Distance: %.2fm", distance_to_goal);
            goal_reached_ = true;
            has_goal_ = false;
            visited_goals_.insert(pointToString(current_goal_));
            should_update_goal = true;
            update_reason = "Goal reached";
            
        } else if (distance_to_goal < goal_switch_distance_) {
            // Case 3: Approaching goal, plan next one early
            ROS_INFO("[DensityFrontierExplorer] Approaching goal (%.2fm), planning next goal", 
                     distance_to_goal);
            should_update_goal = true;
            update_reason = "Approaching goal";
            visited_goals_.insert(pointToString(current_goal_));
            
        } else if (time_since_goal_update > goal_reevaluation_interval_) {
            // Case 4: Periodic goal reevaluation (core improvement)
            ROS_INFO("[DensityFrontierExplorer] Periodic goal reevaluation (%.1fs since last)", 
                     time_since_goal_update);
            should_update_goal = true;
            update_reason = "Periodic reevaluation";
            
        } else if (time_since_path_replan > path_replan_interval_) {
            // Case 5: Periodic path replanning (verify path still valid)
            ROS_INFO("[DensityFrontierExplorer] Periodic path replanning (%.1fs since last)", 
                     time_since_path_replan);
            should_replan_path = true;
        }
    }
    
    // ===== 3. Replan path to current goal only =====
    if (should_replan_path && has_goal_) {
        std::vector<Eigen::Vector3d> new_path;
        if (planGlobalPath(robot_position_, current_goal_, new_path)) {
            current_global_path_ = new_path;
            last_path_replan_time_ = current_time;
            ROS_INFO("[DensityFrontierExplorer] Path replanning succeeded! Path points: %zu", new_path.size());
            publishGlobalPath(new_path);
        } else {
            ROS_WARN("[DensityFrontierExplorer] Path replanning failed, current goal unreachable, selecting new goal");
            should_update_goal = true;
            update_reason = "Path unreachable";
            has_goal_ = false;
            visited_goals_.insert(pointToString(current_goal_));
        }
    }
    
    // ===== 4. Reevaluate and select new goal =====
    if (should_update_goal) {
        ROS_INFO("[DensityFrontierExplorer] Starting goal update, reason: %s", update_reason.c_str());
        
        std::vector<FrontierCandidate> candidates;
        
        // Detect frontiers based on sector density
        detectFrontiersBySectorDensity(candidates);
        
        if (candidates.empty()) {
            ROS_WARN_THROTTLE(3.0, "[DensityFrontierExplorer] No frontier candidates detected!");
            ROS_INFO_THROTTLE(3.0, "[DensityFrontierExplorer] Terrain cloud size: %zu", 
                             terrain_cloud_->size());
            return;
        }
        
        ROS_INFO("[DensityFrontierExplorer] Detected %zu frontier candidates", candidates.size());
        
        // Select best frontier (ensure reachable)
        Eigen::Vector3d new_goal;
        bool found_reachable_goal = false;
        
        // Try multiple candidates until finding a reachable one
        for (int attempt = 0; attempt < 5 && !found_reachable_goal; ++attempt) {
            if (selectBestFrontier(candidates, new_goal)) {
                // Check path reachability first
                if (use_global_path_) {
                    std::vector<Eigen::Vector3d> test_path;
                    if (planGlobalPath(robot_position_, new_goal, test_path)) {
                        found_reachable_goal = true;
                        current_goal_ = new_goal;
                        current_global_path_ = test_path;
                        has_goal_ = true;
                        goal_reached_ = false;
                        last_goal_update_time_ = current_time;
                        last_path_replan_time_ = current_time;
                        
                        ROS_INFO("[DensityFrontierExplorer] Found reachable goal! Path points: %zu", 
                                 test_path.size());
                        publishGlobalPath(test_path);
                        break;
                    } else {
                        // Goal unreachable, mark as visited and try next
                        ROS_WARN("[DensityFrontierExplorer] Goal unreachable, trying next...");
                        visited_goals_.insert(pointToString(new_goal));
                        
                        // 从候选列表中移除此点
                        for (auto& candidate : candidates) {
                            if (getDistance2D(candidate.position, new_goal) < 0.5) {
                                candidate.is_valid = false;
                            }
                        }
                    }
                } else {
                    // Not using global path, publish directly
                    current_goal_ = new_goal;
                    has_goal_ = true;
                    goal_reached_ = false;
                    last_goal_update_time_ = current_time;
                    publishExplorationGoal(new_goal);
                    found_reachable_goal = true;
                    break;
                }
            } else {
                break;
            }
        }
        
        if (!found_reachable_goal) {
            ROS_WARN("[DensityFrontierExplorer] No reachable frontier found!");
        } else {
            ROS_INFO("[DensityFrontierExplorer] New goal: (%.2f, %.2f, %.2f)",
                     current_goal_.x(), current_goal_.y(), current_goal_.z());
        }
        
        // 可视化
        if (enable_visualization_) {
            visualizeFrontiers(candidates);
            if (has_goal_) {
                visualizeBestFrontier(new_goal);
            }
        }
    }
    
    printExplorationStatus();
}

void DensityFrontierExplorer::detectFrontiersBySectorDensity(
    std::vector<FrontierCandidate>& candidates) {
    
    candidates.clear();
    
    // 对每个扇区和每个径向距离采样
    for (int sector_idx = 0; sector_idx < num_sectors_; ++sector_idx) {
        // 计算扇区角度
        double angle = (2.0 * M_PI * sector_idx) / num_sectors_ - M_PI;
        
        // 在该扇区的不同半径处采样
        for (double radius = min_sector_radius_; 
             radius <= max_sector_radius_; 
             radius += sector_radial_resolution_) {
            
            // 计算采样点位置
            Eigen::Vector3d sample_pos;
            sample_pos.x() = robot_position_.x() + radius * std::cos(angle);
            sample_pos.y() = robot_position_.y() + radius * std::sin(angle);
            sample_pos.z() = robot_position_.z();
            
            // 计算该点的密度
            double density = calculateDensity(sample_pos);
            
            // 判断是否是前沿点（稀疏区域）
            if (isFrontierPoint(sample_pos, density)) {
                FrontierCandidate candidate;
                candidate.position = sample_pos;
                candidate.density = density;
                candidate.distance = radius;
                candidate.is_valid = true;
                
                // 检查是否访问过（目标点级别）
                std::string pos_key = pointToString(sample_pos);
                if (visited_goals_.find(pos_key) != visited_goals_.end()) {
                    candidate.is_valid = false;
                }
                
                // 检查距离范围
                if (radius < min_goal_distance_ || radius > max_goal_distance_) {
                    candidate.is_valid = false;
                }
                
                // 检查是否在已探索区域（区域级别）
                // 注意：不直接设置为无效，而是在评分时惩罚，允许在必要时重访
                int exploration_count = getExplorationCount(sample_pos);
                if (exploration_count >= 3) {
                    // 已探索3次以上，直接过滤
                    candidate.is_valid = false;
                }
                
                candidates.push_back(candidate);
            }
        }
    }
    
    // 评估所有候选点
    evaluateCandidates(candidates);
    
    // 过滤无效候选
    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
                      [](const FrontierCandidate& c) { return !c.is_valid; }),
        candidates.end()
    );
}

double DensityFrontierExplorer::calculateDensity(const Eigen::Vector3d& position) {
    if (terrain_cloud_->empty()) return 0.0;
    
    pcl::PointXYZI search_point;
    search_point.x = position.x();
    search_point.y = position.y();
    search_point.z = position.z();
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    // 在density_radius_范围内搜索点的数量
    int num_points = kdtree_.radiusSearch(search_point, density_radius_, 
                                          indices, distances);
    
    return static_cast<double>(num_points);
}

bool DensityFrontierExplorer::isFrontierPoint(const Eigen::Vector3d& position, 
                                              double density) {
    // 前沿判断：点云密度在稀疏阈值以下
    // 但又不是完全没有点（说明在地图边缘）
    return (density > 0 && density < sparse_threshold_);
}

void DensityFrontierExplorer::evaluateCandidates(
    std::vector<FrontierCandidate>& candidates) {
    
    for (auto& candidate : candidates) {
        if (!candidate.is_valid) continue;
        
        // 1. 稀疏性得分（密度越低越好）
        double sparsity_score = 1.0 - (candidate.density / sparse_threshold_);
        sparsity_score = std::max(0.0, std::min(1.0, sparsity_score));
        
        // 2. 改进的距离得分（优先近距离目标）
        // 使用指数衰减，近距离目标得分更高
        double distance_score;
        if (candidate.distance < min_goal_distance_) {
            distance_score = 0.0;  // 太近
        } else if (candidate.distance < min_goal_distance_ * 1.5) {
            // 理想距离范围：min_goal_distance ~ min_goal_distance*1.5
            distance_score = 1.0;
        } else {
            // 距离越远，得分指数衰减
            double decay_factor = 0.3;  // 衰减系数
            double extra_distance = candidate.distance - min_goal_distance_ * 1.5;
            distance_score = std::exp(-decay_factor * extra_distance);
        }
        distance_score = std::max(0.0, std::min(1.0, distance_score));
        
        // 3. 方向得分（与当前朝向一致）
        Eigen::Vector3d direction = candidate.position - robot_position_;
        double target_yaw = std::atan2(direction.y(), direction.x());
        double yaw_diff = std::abs(target_yaw - robot_yaw_);
        while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
        yaw_diff = std::abs(yaw_diff);
        double direction_score = 1.0 - (yaw_diff / M_PI);
        
        // 4. 已探索惩罚（避免重复探索）
        int exploration_count = getExplorationCount(candidate.position);
        double exploration_penalty = 1.0 / (1.0 + exploration_count * 0.5);  // 探索次数越多惩罚越大
        
        candidate.direction_score = direction_score;
        
        // 综合评分（增加探索惩罚）
        candidate.score = (weight_sparsity_ * sparsity_score +
                          weight_distance_ * distance_score +
                          weight_direction_ * direction_score) * exploration_penalty;
        
        // 如果区域已被探索多次，直接标记为无效
        if (exploration_count >= 3) {
            candidate.is_valid = false;
        }
    }
}

bool DensityFrontierExplorer::selectBestFrontier(
    const std::vector<FrontierCandidate>& candidates,
    Eigen::Vector3d& goal) {
    
    double best_score = -1.0;
    bool found = false;
    
    for (const auto& candidate : candidates) {
        if (!candidate.is_valid) continue;
        
        if (candidate.score > best_score) {
            best_score = candidate.score;
            goal = candidate.position;
            found = true;
        }
    }
    
    if (found) {
        ROS_INFO("[DensityFrontierExplorer] Best frontier score: %.3f", best_score);
    }
    
    return found;
}

bool DensityFrontierExplorer::isGoalReached() {
    if (!has_goal_) return false;
    
    double distance = getDistance2D(robot_position_, current_goal_);
    return distance < goal_reached_threshold_;
}

void DensityFrontierExplorer::publishExplorationGoal(const Eigen::Vector3d& goal) {
    geometry_msgs::PoseStamped goal_msg;
    goal_msg.header.frame_id = "map";
    goal_msg.header.stamp = ros::Time::now();
    goal_msg.pose.position.x = goal.x();
    goal_msg.pose.position.y = goal.y();
    goal_msg.pose.position.z = goal.z();
    
    Eigen::Vector3d direction = goal - robot_position_;
    double yaw = std::atan2(direction.y(), direction.x());
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    goal_msg.pose.orientation = tf2::toMsg(q);
    
    exploration_goal_pub_.publish(goal_msg);
    
    ROS_INFO("[DensityFrontierExplorer] Published exploration goal");
}

void DensityFrontierExplorer::visualizeFrontiers(
    const std::vector<FrontierCandidate>& candidates) {
    
    visualization_msgs::MarkerArray marker_array;
    
    int id = 0;
    for (const auto& candidate : candidates) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "frontier_candidates";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = candidate.position.x();
        marker.pose.position.y = candidate.position.y();
        marker.pose.position.z = candidate.position.z();
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        if (candidate.is_valid) {
            // 根据得分设置颜色（绿到黄）
            marker.color.r = candidate.score;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.7;
        } else {
            marker.color.r = 0.3;
            marker.color.g = 0.3;
            marker.color.b = 0.3;
            marker.color.a = 0.3;
        }
        
        marker.lifetime = ros::Duration(1.5);
        marker_array.markers.push_back(marker);
    }
    
    frontier_vis_pub_.publish(marker_array);
}

void DensityFrontierExplorer::visualizeBestFrontier(const Eigen::Vector3d& goal) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "best_frontier";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = goal.x();
    marker.pose.position.y = goal.y();
    marker.pose.position.z = goal.z() + 0.5;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 1.2;
    marker.scale.y = 1.2;
    marker.scale.z = 1.2;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9;
    
    marker.lifetime = ros::Duration(1.5);
    
    best_frontier_pub_.publish(marker);
}

void DensityFrontierExplorer::visualizeSectorLines() {
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "map";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "sector_lines";
    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.action = visualization_msgs::Marker::ADD;
    
    line_list.scale.x = 0.05;
    line_list.color.r = 0.0;
    line_list.color.g = 0.8;
    line_list.color.b = 1.0;
    line_list.color.a = 0.3;
    
    line_list.lifetime = ros::Duration(0.5);
    
    // 绘制扇区线
    for (int i = 0; i < num_sectors_; ++i) {
        double angle = (2.0 * M_PI * i) / num_sectors_ - M_PI;
        
        geometry_msgs::Point p1, p2;
        p1.x = robot_position_.x();
        p1.y = robot_position_.y();
        p1.z = robot_position_.z() + 0.1;
        
        p2.x = robot_position_.x() + max_sector_radius_ * std::cos(angle);
        p2.y = robot_position_.y() + max_sector_radius_ * std::sin(angle);
        p2.z = robot_position_.z() + 0.1;
        
        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }
    
    sector_lines_pub_.publish(line_list);
}

double DensityFrontierExplorer::getDistance(const Eigen::Vector3d& p1, 
                                           const Eigen::Vector3d& p2) {
    return (p1 - p2).norm();
}

double DensityFrontierExplorer::getDistance2D(const Eigen::Vector3d& p1,
                                             const Eigen::Vector3d& p2) {
    double dx = p1.x() - p2.x();
    double dy = p1.y() - p2.y();
    return std::sqrt(dx*dx + dy*dy);
}

std::string DensityFrontierExplorer::pointToString(const Eigen::Vector3d& point) {
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "%.1f_%.1f", point.x(), point.y());
    return std::string(buffer);
}

void DensityFrontierExplorer::printExplorationStatus() {
    ROS_INFO_THROTTLE(5.0, "======== Exploration Status ========");
    ROS_INFO_THROTTLE(5.0, "Robot: (%.2f, %.2f, %.2f)",
                     robot_position_.x(), robot_position_.y(), robot_position_.z());
    ROS_INFO_THROTTLE(5.0, "Cloud size: %zu", terrain_cloud_->size());
    ROS_INFO_THROTTLE(5.0, "Has goal: %s", has_goal_ ? "YES" : "NO");
    if (has_goal_) {
        ROS_INFO_THROTTLE(5.0, "Goal: (%.2f, %.2f)", 
                         current_goal_.x(), current_goal_.y());
        ROS_INFO_THROTTLE(5.0, "Distance: %.2f m",
                         getDistance2D(robot_position_, current_goal_));
        int goal_exploration_count = getExplorationCount(current_goal_);
        ROS_INFO_THROTTLE(5.0, "Goal region exploration count: %d", goal_exploration_count);
    }
    ROS_INFO_THROTTLE(5.0, "Visited goals: %zu", visited_goals_.size());
    ROS_INFO_THROTTLE(5.0, "Explored region grids: %zu", explored_regions_.size());
    ROS_INFO_THROTTLE(5.0, "====================================");
}

bool DensityFrontierExplorer::planGlobalPath(const Eigen::Vector3d& start,
                                            const Eigen::Vector3d& goal,
                                            std::vector<Eigen::Vector3d>& path) {
    path.clear();
    
    // 简化的RRT*实现
    struct Node {
        Eigen::Vector3d position;
        int parent_idx;
        double cost;
    };
    
    std::vector<Node> tree;
    Node start_node;
    start_node.position = start;
    start_node.parent_idx = -1;
    start_node.cost = 0.0;
    tree.push_back(start_node);
    
    int goal_node_idx = -1;
    double best_cost = std::numeric_limits<double>::max();
    
    for (int iter = 0; iter < rrt_max_iterations_; ++iter) {
        // 采样
        Eigen::Vector3d sample;
        if (static_cast<double>(rand()) / RAND_MAX < rrt_goal_bias_) {
            sample = goal;
        } else {
            sample.x() = start.x() + (static_cast<double>(rand()) / RAND_MAX - 0.5) * 
                        2.0 * max_goal_distance_;
            sample.y() = start.y() + (static_cast<double>(rand()) / RAND_MAX - 0.5) * 
                        2.0 * max_goal_distance_;
            sample.z() = start.z();
        }
        
        // 找最近节点
        int nearest_idx = 0;
        double min_dist = getDistance(tree[0].position, sample);
        for (size_t i = 1; i < tree.size(); ++i) {
            double dist = getDistance(tree[i].position, sample);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        // 扩展
        Eigen::Vector3d direction = sample - tree[nearest_idx].position;
        double dist = direction.norm();
        if (dist > rrt_step_size_) {
            direction = direction / dist * rrt_step_size_;
        }
        
        Eigen::Vector3d new_pos = tree[nearest_idx].position + direction;
        
        // 碰撞检测
        if (!isPathCollisionFree(tree[nearest_idx].position, new_pos)) {
            continue;
        }
        
        // 添加新节点
        Node new_node;
        new_node.position = new_pos;
        new_node.parent_idx = nearest_idx;
        new_node.cost = tree[nearest_idx].cost + direction.norm();
        tree.push_back(new_node);
        
        // 检查是否到达目标
        if (getDistance(new_pos, goal) < rrt_step_size_) {
            if (isPathCollisionFree(new_pos, goal)) {
                if (new_node.cost < best_cost) {
                    best_cost = new_node.cost;
                    goal_node_idx = tree.size() - 1;
                }
            }
        }
        
        // 如果找到路径且迭代足够，提前退出
        if (goal_node_idx >= 0 && iter > rrt_max_iterations_ / 3) {
            break;
        }
    }
    
    // 回溯路径
    if (goal_node_idx < 0) {
        return false;
    }
    
    path.push_back(goal);
    int current_idx = goal_node_idx;
    while (current_idx >= 0) {
        path.push_back(tree[current_idx].position);
        current_idx = tree[current_idx].parent_idx;
    }
    
    std::reverse(path.begin(), path.end());
    
    ROS_INFO("[DensityFrontierExplorer] RRT* original path: %zu points", path.size());
    
    // Smooth path
    smoothPath(path);
    ROS_INFO("[DensityFrontierExplorer] After smoothing: %zu points", path.size());
    
    // Densify path (ensure point spacing not too large)
    densifyPath(path);
    ROS_INFO("[DensityFrontierExplorer] After densification: %zu points", path.size());
    
    // Verify final path point spacing
    double max_dist = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        double dist = getDistance(path[i], path[i+1]);
        max_dist = std::max(max_dist, dist);
    }
    ROS_INFO("[DensityFrontierExplorer] Max point spacing: %.2f m", max_dist);
    
    return true;
}

bool DensityFrontierExplorer::isPathCollisionFree(const Eigen::Vector3d& p1,
                                                  const Eigen::Vector3d& p2) {
    if (terrain_cloud_->empty()) return true;
    
    Eigen::Vector3d direction = p2 - p1;
    double distance = direction.norm();
    if (distance < 0.01) return true;
    
    direction.normalize();
    
    // 使用更小的检查步长以提高精度（问题1解决方案）
    int num_steps = static_cast<int>(distance / collision_check_step_) + 1;
    
    for (int i = 0; i <= num_steps; ++i) {
        Eigen::Vector3d check_point = p1 + direction * (i * collision_check_step_);
        
        // 检查该点是否安全
        if (!isPointCollisionFree(check_point)) {
            return false;
        }
    }
    
    return true;
}

bool DensityFrontierExplorer::isPointCollisionFree(const Eigen::Vector3d& point) {
    if (terrain_cloud_->empty()) return true;
    
    pcl::PointXYZI search_point;
    search_point.x = point.x();
    search_point.y = point.y();
    search_point.z = point.z();
    
    std::vector<int> indices;
    std::vector<float> distances;
    
    // 在机器人半径范围内搜索（考虑机器人尺寸 + 安全边界）
    int num_nearby = kdtree_.radiusSearch(search_point, robot_radius_, indices, distances);
    
    if (num_nearby == 0) {
        // 没有点云数据，可能是未探索区域，保守地认为可通过
        return true;
    }
    
    // 检查周围点的高度差
    for (size_t i = 0; i < indices.size(); ++i) {
        const auto& terrain_point = terrain_cloud_->points[indices[i]];
        
        // 1. 检查垂直高度差（障碍物检测）
        double height_diff = terrain_point.z - point.z();
        
        // 如果地形点明显高于路径点，是障碍物
        if (height_diff > obstacle_height_threshold_) {
            return false;
        }
        
        // 如果地形点明显低于路径点，可能是悬崖或沟壑
        if (height_diff < -obstacle_height_threshold_ * 2.0) {
            return false;
        }
        
        // 2. 使用intensity信息（如果terrain_map_ext提供了地形特征）
        // intensity > 100 通常表示障碍物或不可通行区域
        if (terrain_point.intensity > 100.0) {
            return false;
        }
    }
    
    // 额外检查：确保路径点周围有足够的地面支撑
    // 在水平方向上检查多个方向
    const int num_directions = 8;
    const double check_radius = robot_radius_ * 0.7;
    
    for (int dir = 0; dir < num_directions; ++dir) {
        double angle = 2.0 * M_PI * dir / num_directions;
        Eigen::Vector3d offset_point = point;
        offset_point.x() += check_radius * std::cos(angle);
        offset_point.y() += check_radius * std::sin(angle);
        
        pcl::PointXYZI offset_search;
        offset_search.x = offset_point.x();
        offset_search.y = offset_point.y();
        offset_search.z = offset_point.z();
        
        std::vector<int> offset_indices;
        std::vector<float> offset_distances;
        
        int offset_count = kdtree_.radiusSearch(offset_search, robot_radius_ * 0.3, 
                                               offset_indices, offset_distances);
        
        if (offset_count > 0) {
            for (size_t i = 0; i < offset_indices.size(); ++i) {
                const auto& terrain_point = terrain_cloud_->points[offset_indices[i]];
                double height_diff = terrain_point.z - point.z();
                
                // 周围方向上也不能有障碍物
                if (height_diff > obstacle_height_threshold_) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

void DensityFrontierExplorer::smoothPath(std::vector<Eigen::Vector3d>& path) {
    if (path.size() < 3) return;
    
    // 改进的路径平滑：跳过中间点但保持合理密度
    std::vector<Eigen::Vector3d> smoothed_path;
    smoothed_path.push_back(path[0]);
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t j = path.size() - 1;
        bool found = false;
        
        // 从最远点开始尝试直接连接，但限制最大跳跃距离
        while (j > i + 1) {
            double dist = getDistance(path[i], path[j]);
            
            // 限制：如果距离太远，不要跳过太多点（保持密度）
            if (dist <= max_path_point_distance_ * 3.0) {  // 允许3倍最大间距进行平滑
                if (isPathCollisionFree(path[i], path[j])) {
                    smoothed_path.push_back(path[j]);
                    i = j;
                    found = true;
                    break;
                }
            }
            j--;
        }
        
        if (!found) {
            smoothed_path.push_back(path[i + 1]);
            i++;
        }
    }
    
    path = smoothed_path;
}

void DensityFrontierExplorer::densifyPath(std::vector<Eigen::Vector3d>& path) {
    if (path.size() < 2) return;
    
    // 路径密集化：在点与点之间插入中间点（问题2解决方案）
    std::vector<Eigen::Vector3d> densified_path;
    densified_path.push_back(path[0]);
    
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Eigen::Vector3d& p1 = path[i];
        const Eigen::Vector3d& p2 = path[i + 1];
        
        double distance = getDistance(p1, p2);
        
        // 如果两点间距超过阈值，插入中间点
        if (distance > max_path_point_distance_) {
            int num_segments = static_cast<int>(std::ceil(distance / max_path_point_distance_));
            
            for (int j = 1; j < num_segments; ++j) {
                double ratio = static_cast<double>(j) / num_segments;
                Eigen::Vector3d intermediate_point = p1 + (p2 - p1) * ratio;
                densified_path.push_back(intermediate_point);
            }
        }
        
        densified_path.push_back(p2);
    }
    
    path = densified_path;
    
    ROS_DEBUG("[DensityFrontierExplorer] Path densification completed: %zu points", densified_path.size());
}

void DensityFrontierExplorer::publishGlobalPath(const std::vector<Eigen::Vector3d>& path) {
    visualization_msgs::MarkerArray marker_array;
    
    // 发布路径点
    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "global_path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = path[i].x();
        marker.pose.position.y = path[i].y();
        marker.pose.position.z = path[i].z();
        marker.pose.orientation.w = 1.0;
        
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;
        
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 0.8;
        
        marker.lifetime = ros::Duration(5.0);
        marker_array.markers.push_back(marker);
    }
    
    // 发布路径线
    if (path.size() > 1) {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "global_path_line";
        line_marker.id = path.size();
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        
        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0;
        line_marker.color.g = 1.0;
        line_marker.color.b = 1.0;
        line_marker.color.a = 0.8;
        
        for (const auto& point : path) {
            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = point.z();
            line_marker.points.push_back(p);
        }
        
        line_marker.lifetime = ros::Duration(5.0);
        marker_array.markers.push_back(line_marker);
    }
    
    global_path_pub_.publish(marker_array);
    ROS_DEBUG("[DensityFrontierExplorer] Published global path to /global_planned_path");
}

void DensityFrontierExplorer::run() {
    ros::spin();
}

// ========== 已探索区域管理 ==========

std::string DensityFrontierExplorer::positionToGridKey(const Eigen::Vector3d& position) {
    // 将位置转换为网格坐标
    int grid_x = static_cast<int>(std::floor(position.x() / exploration_grid_resolution_));
    int grid_y = static_cast<int>(std::floor(position.y() / exploration_grid_resolution_));
    
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "grid_%d_%d", grid_x, grid_y);
    return std::string(buffer);
}

void DensityFrontierExplorer::markRegionAsExplored(const Eigen::Vector3d& position) {
    // 标记机器人周围exploration_radius_范围内的所有网格为已探索
    int num_grids = static_cast<int>(std::ceil(exploration_radius_ / exploration_grid_resolution_));
    
    for (int dx = -num_grids; dx <= num_grids; ++dx) {
        for (int dy = -num_grids; dy <= num_grids; ++dy) {
            Eigen::Vector3d offset_pos = position;
            offset_pos.x() += dx * exploration_grid_resolution_;
            offset_pos.y() += dy * exploration_grid_resolution_;
            
            // 检查是否在探索半径内
            double distance = getDistance2D(position, offset_pos);
            if (distance <= exploration_radius_) {
                std::string grid_key = positionToGridKey(offset_pos);
                explored_regions_[grid_key]++;
            }
        }
    }
}

bool DensityFrontierExplorer::isRegionExplored(const Eigen::Vector3d& position) {
    std::string grid_key = positionToGridKey(position);
    return explored_regions_.find(grid_key) != explored_regions_.end();
}

int DensityFrontierExplorer::getExplorationCount(const Eigen::Vector3d& position) {
    std::string grid_key = positionToGridKey(position);
    auto it = explored_regions_.find(grid_key);
    if (it != explored_regions_.end()) {
        return it->second;
    }
    return 0;
}

} // namespace exploration



