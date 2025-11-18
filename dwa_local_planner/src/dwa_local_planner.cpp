#include "dwa_local_planner/dwa_local_planner.h"

namespace dwa_local_planner
{

DWALocalPlanner::DWALocalPlanner()
  : odom_received_(false)
  , pointcloud_received_(false)
  , global_path_received_(false)
  , robot_x_(0.0)
  , robot_y_(0.0)
  , robot_yaw_(0.0)
  , robot_roll_(0.0)
  , robot_pitch_(0.0)
  , current_linear_vel_(0.0)
  , current_angular_vel_(0.0)
  , odom_time_(0.0)
  , current_goal_index_(1)
  , goal_x_(0.0)
  , goal_y_(0.0)
{
  raw_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cropped_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  transformed_pointcloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  
  last_global_path_endpoint_ = std::make_pair(NAN, NAN);
}

DWALocalPlanner::~DWALocalPlanner()
{
}

bool DWALocalPlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  nh_ = nh;
  nh_private_ = nh_private;
  
  // 加载参数 - 机器人物理属性
  nh_private_.param("robot_radius", robot_radius_, 0.3);
  nh_private_.param("sensor_offset_x", sensor_offset_x_, 0.0);
  nh_private_.param("sensor_offset_y", sensor_offset_y_, 0.0);
  
  // 加载参数 - 运动学约束
  nh_private_.param("max_speed", max_speed_, 2.0);
  nh_private_.param("min_speed", min_speed_, 0.0);
  nh_private_.param("max_yaw_rate", max_yaw_rate_, 90.0);  // 度/秒
  nh_private_.param("max_accel", max_accel_, 2.5);
  nh_private_.param("max_yaw_accel", max_yaw_accel_, 120.0);  // 度/秒^2
  
  // 转换角度到弧度
  max_yaw_rate_ = max_yaw_rate_ * PI / 180.0;
  max_yaw_accel_ = max_yaw_accel_ * PI / 180.0;
  
  // 加载参数 - DWA 采样
  nh_private_.param("sim_time", sim_time_, 2.0);
  nh_private_.param("sim_dt", sim_dt_, 0.1);
  nh_private_.param("vx_samples", vx_samples_, 8);
  nh_private_.param("wz_samples", wz_samples_, 16);
  
  // 加载参数 - 传感器
  nh_private_.param("use_terrain_analysis", use_terrain_analysis_, true);
  nh_private_.param("adjacent_range", adjacent_range_, 4.25);
  nh_private_.param("voxel_size", voxel_size_, 0.05);
  nh_private_.param("obstacle_height_thre", obstacle_height_thre_, 0.15);
  
  // 加载参数 - 全局路径跟随
  nh_private_.param("path_lookahead_dist", path_lookahead_dist_, 3.0);
  nh_private_.param("goal_update_distance", goal_update_distance_, 1.0);
  nh_private_.param("goal_tolerance", goal_tolerance_, 0.3);
  
  // 加载参数 - 代价函数权重
  nh_private_.param("heading_cost_weight", heading_cost_weight_, 1.0);
  nh_private_.param("velocity_cost_weight", velocity_cost_weight_, 0.5);
  nh_private_.param("clearance_cost_weight", clearance_cost_weight_, 1.5);
  nh_private_.param("path_distance_cost_weight", path_distance_cost_weight_, 1.2);
  
  // 加载参数 - 安全
  nh_private_.param("stop_clearance", stop_clearance_, 0.5);
  nh_private_.param("safety_margin", safety_margin_, 0.1);
  
  // 加载参数 - 其他
  nh_private_.param("control_rate", control_rate_, 20.0);
  nh_private_.param("two_way_drive", two_way_drive_, true);
  
  // 初始化订阅者
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, 
                                                  &DWALocalPlanner::odomCallback, this);
  
  if (use_terrain_analysis_) {
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, 
                                                               &DWALocalPlanner::pointCloudCallback, this);
  } else {
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, 
                                                               &DWALocalPlanner::pointCloudCallback, this);
  }
  
  global_path_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/global_planned_path", 5, 
                                                                     &DWALocalPlanner::globalPathCallback, this);
  
  // 初始化发布者
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);
  local_plan_pub_ = nh_.advertise<nav_msgs::Path>("/dwa_local_plan", 5);
  candidates_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dwa_candidates", 5);
  goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_marker", 5);
  
  // 配置体素滤波器
  voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  
  ROS_INFO("DWA Local Planner initialized successfully");
  ROS_INFO("  Robot radius: %.2f m", robot_radius_);
  ROS_INFO("  Max speed: %.2f m/s, Max yaw rate: %.2f deg/s", max_speed_, max_yaw_rate_ * 180.0 / PI);
  ROS_INFO("  Simulation time: %.2f s, dt: %.2f s", sim_time_, sim_dt_);
  ROS_INFO("  Velocity samples: %d, Angular samples: %d", vx_samples_, wz_samples_);
  ROS_INFO("  Use terrain analysis: %s", use_terrain_analysis_ ? "true" : "false");
  
  return true;
}

void DWALocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_time_ = msg->header.stamp.toSec();
  
  // 提取位姿
  tf::Quaternion q(msg->pose.pose.orientation.x,
                   msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z,
                   msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(robot_roll_, robot_pitch_, robot_yaw_);
  
  // 考虑传感器偏移
  robot_x_ = msg->pose.pose.position.x - cos(robot_yaw_) * sensor_offset_x_ + sin(robot_yaw_) * sensor_offset_y_;
  robot_y_ = msg->pose.pose.position.y - sin(robot_yaw_) * sensor_offset_x_ - cos(robot_yaw_) * sensor_offset_y_;
  
  // 提取速度
  current_linear_vel_ = msg->twist.twist.linear.x;
  current_angular_vel_ = msg->twist.twist.angular.z;
  
  odom_received_ = true;
}

void DWALocalPlanner::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // 转换点云
  raw_pointcloud_->clear();
  pcl::fromROSMsg(*msg, *raw_pointcloud_);
  
  pointcloud_received_ = true;
}

void DWALocalPlanner::globalPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  // 提取并排序路径点
  std::vector<std::pair<int, std::pair<double, double>>> temp_points;
  for (const auto& marker : msg->markers) {
    if (marker.type == visualization_msgs::Marker::SPHERE) {
      double x = marker.pose.position.x;
      double y = marker.pose.position.y;
      temp_points.push_back(std::make_pair(marker.id, std::make_pair(x, y)));
    }
  }
  
  if (temp_points.empty()) return;
  
  // 按 ID 排序
  std::sort(temp_points.begin(), temp_points.end(),
            [](const std::pair<int, std::pair<double, double>>& a,
               const std::pair<int, std::pair<double, double>>& b) {
              return a.first < b.first;
            });
  
  // 检查是否是新路径
  std::pair<double, double> new_endpoint = temp_points.back().second;
  bool is_new_path = false;
  if (std::isnan(last_global_path_endpoint_.first) ||
      std::abs(new_endpoint.first - last_global_path_endpoint_.first) > 0.01 ||
      std::abs(new_endpoint.second - last_global_path_endpoint_.second) > 0.01) {
    is_new_path = true;
    last_global_path_endpoint_ = new_endpoint;
  }
  
  if (is_new_path) {
    global_path_points_.clear();
    for (const auto& point : temp_points) {
      global_path_points_.push_back(point.second);
    }
    
    current_goal_index_ = 1;
    if (current_goal_index_ >= global_path_points_.size()) {
      current_goal_index_ = global_path_points_.size() - 1;
    }
    
    goal_x_ = global_path_points_[current_goal_index_].first;
    goal_y_ = global_path_points_[current_goal_index_].second;
    
    ROS_INFO("Received new global path with %lu points. Goal: (%.2f, %.2f)",
             global_path_points_.size(), goal_x_, goal_y_);
  }
  
  global_path_received_ = true;
}

void DWALocalPlanner::updateGlobalPathGoal()
{
  if (global_path_points_.empty()) return;
  
  // 计算到当前目标的距离
  double dist_to_goal = std::sqrt(std::pow(robot_x_ - goal_x_, 2) + std::pow(robot_y_ - goal_y_, 2));
  
  // 如果接近当前目标，推进到下一个目标点
  if (dist_to_goal < goal_update_distance_ && current_goal_index_ < global_path_points_.size() - 1) {
    current_goal_index_++;
    goal_x_ = global_path_points_[current_goal_index_].first;
    goal_y_ = global_path_points_[current_goal_index_].second;
    ROS_INFO("Advancing to goal point %d: (%.2f, %.2f)", current_goal_index_, goal_x_, goal_y_);
  }
}

void DWALocalPlanner::transformPointCloud()
{
  // 裁剪点云到机器人附近
  cropped_pointcloud_->clear();
  for (const auto& point : raw_pointcloud_->points) {
    double dist = std::sqrt(std::pow(point.x - robot_x_, 2) + std::pow(point.y - robot_y_, 2));
    if (dist < adjacent_range_) {
      // 对于地形分析，检查障碍物高度
      if (use_terrain_analysis_) {
        if (point.intensity > obstacle_height_thre_) {
          cropped_pointcloud_->push_back(point);
        }
      } else {
        cropped_pointcloud_->push_back(point);
      }
    }
  }
  
  // 体素降采样
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_filter_.setInputCloud(cropped_pointcloud_);
  voxel_filter_.filter(*downsampled);
  
  // 转换到机器人坐标系
  transformed_pointcloud_->clear();
  for (const auto& point : downsampled->points) {
    pcl::PointXYZI transformed;
    double dx = point.x - robot_x_;
    double dy = point.y - robot_y_;
    transformed.x = dx * std::cos(robot_yaw_) + dy * std::sin(robot_yaw_);
    transformed.y = -dx * std::sin(robot_yaw_) + dy * std::cos(robot_yaw_);
    transformed.z = point.z;
    transformed.intensity = point.intensity;
    transformed_pointcloud_->push_back(transformed);
  }
  
  // 构建 KD 树用于快速距离查询
  if (transformed_pointcloud_->points.size() > 0) {
    kdtree_.setInputCloud(transformed_pointcloud_);
  }
}

std::vector<Trajectory> DWALocalPlanner::generateTrajectories()
{
  std::vector<Trajectory> trajectories;
  
  // 计算动态窗口 - 使用控制周期
  double control_dt = 1.0 / control_rate_;
  
  // 计算基于加速度约束的速度范围
  double accel_min_v = current_linear_vel_ - max_accel_ * control_dt;
  double accel_max_v = current_linear_vel_ + max_accel_ * control_dt;
  double accel_min_w = current_angular_vel_ - max_yaw_accel_ * control_dt;
  double accel_max_w = current_angular_vel_ + max_yaw_accel_ * control_dt;
  
  // 与物理限制取交集
  double min_v = std::max(min_speed_, accel_min_v);
  double max_v = std::min(max_speed_, accel_max_v);
  double min_w = std::max(-max_yaw_rate_, accel_min_w);
  double max_w = std::min(max_yaw_rate_, accel_max_w);
  
  // 确保速度窗口不会太小（至少有一定的采样范围）
  // 线速度窗口至少 0.6 m/s 范围
  if (max_v - min_v < 0.6) {
    double mid_v = (max_v + min_v) / 2.0;
    min_v = std::max(min_speed_, mid_v - 0.3);
    max_v = std::min(max_speed_, mid_v + 0.3);
  }
  
  // 角速度窗口至少 0.5 rad/s (约28度/秒) 范围
  if (max_w - min_w < 0.5) {
    double mid_w = (max_w + min_w) / 2.0;
    min_w = std::max(-max_yaw_rate_, mid_w - 0.25);
    max_w = std::min(max_yaw_rate_, mid_w + 0.25);
  }
  
  // 采样速度空间
  double dv = (max_v - min_v) / std::max(1, vx_samples_ - 1);
  double dw = (max_w - min_w) / std::max(1, wz_samples_ - 1);
  
  for (int i = 0; i < vx_samples_; ++i) {
    double v = min_v + i * dv;
    
    for (int j = 0; j < wz_samples_; ++j) {
      double w = min_w + j * dw;
      
      // 模拟轨迹
      Trajectory traj = simulateTrajectory(v, w);
      trajectories.push_back(traj);
    }
  }
  
  return trajectories;
}

Trajectory DWALocalPlanner::simulateTrajectory(double v, double w)
{
  Trajectory traj;
  traj.linear_vel = v;
  traj.angular_vel = w;
  
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  
  // 前向模拟
  int num_steps = static_cast<int>(sim_time_ / sim_dt_);
  for (int i = 0; i < num_steps; ++i) {
    // 运动学模型：差分驱动
    x += v * std::cos(theta) * sim_dt_;
    y += v * std::sin(theta) * sim_dt_;
    theta += w * sim_dt_;
    
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = 0.0;
    traj.points.push_back(point);
  }
  
  return traj;
}

bool DWALocalPlanner::checkCollision(const Trajectory& traj)
{
  if (transformed_pointcloud_->points.empty()) {
    return false;  // 无点云数据，假设无碰撞
  }
  
  double collision_radius = robot_radius_ + safety_margin_;
  
  // 检查轨迹上的每个点
  for (const auto& point : traj.points) {
    pcl::PointXYZI search_point;
    search_point.x = point.x;
    search_point.y = point.y;
    search_point.z = 0.0;
    
    // KD 树半径搜索
    std::vector<int> point_indices;
    std::vector<float> point_distances;
    
    if (kdtree_.radiusSearch(search_point, collision_radius, point_indices, point_distances) > 0) {
      return true;  // 发现碰撞
    }
  }
  
  return false;  // 无碰撞
}

double DWALocalPlanner::getDistanceToObstacle(const geometry_msgs::Point& point)
{
  if (transformed_pointcloud_->points.empty()) {
    return adjacent_range_;  // 无点云，返回最大距离
  }
  
  pcl::PointXYZI search_point;
  search_point.x = point.x;
  search_point.y = point.y;
  search_point.z = 0.0;
  
  std::vector<int> point_indices(1);
  std::vector<float> point_distances(1);
  
  if (kdtree_.nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
    return std::sqrt(point_distances[0]);
  }
  
  return adjacent_range_;
}

double DWALocalPlanner::getDistanceToGlobalPath(const geometry_msgs::Point& point)
{
  if (global_path_points_.empty()) {
    return 0.0;
  }
  
  // 将局部坐标转换回全局坐标
  double global_x = robot_x_ + point.x * std::cos(robot_yaw_) - point.y * std::sin(robot_yaw_);
  double global_y = robot_y_ + point.x * std::sin(robot_yaw_) + point.y * std::cos(robot_yaw_);
  
  // 找到全局路径上最近的点
  double min_dist = std::numeric_limits<double>::max();
  for (const auto& path_point : global_path_points_) {
    double dist = std::sqrt(std::pow(global_x - path_point.first, 2) + 
                           std::pow(global_y - path_point.second, 2));
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  
  return min_dist;
}

double DWALocalPlanner::evaluateTrajectory(Trajectory& traj)
{
  if (traj.points.empty()) {
    return -1.0;
  }
  
  // 轨迹终点
  geometry_msgs::Point end_point = traj.points.back();
  
  // 1. 朝向代价：计算在局部坐标系中目标的方向
  // 将目标转换到机器人局部坐标系
  double goal_x_local = (goal_x_ - robot_x_) * std::cos(robot_yaw_) + (goal_y_ - robot_y_) * std::sin(robot_yaw_);
  double goal_y_local = -(goal_x_ - robot_x_) * std::sin(robot_yaw_) + (goal_y_ - robot_y_) * std::cos(robot_yaw_);
  
  // 轨迹终点到目标的方向（在局部坐标系中）
  double dx_to_goal = goal_x_local - end_point.x;
  double dy_to_goal = goal_y_local - end_point.y;
  double angle_to_goal_local = std::atan2(dy_to_goal, dx_to_goal);
  
  // 轨迹终点的运动方向（在局部坐标系中）
  double end_theta_local = 0.0;
  if (traj.points.size() >= 2) {
    size_t last_idx = traj.points.size() - 1;
    double dx = traj.points[last_idx].x - traj.points[last_idx-1].x;
    double dy = traj.points[last_idx].y - traj.points[last_idx-1].y;
    if (std::abs(dx) > 1e-6 || std::abs(dy) > 1e-6) {
      end_theta_local = std::atan2(dy, dx);
    }
  }
  
  double heading_diff = std::abs(normalizeAngle(angle_to_goal_local - end_theta_local));
  traj.heading_cost = 1.0 - (heading_diff / PI);  // 归一化到 [0, 1]，越大越好
  
  // 2. 速度代价：鼓励高线速度
  traj.velocity_cost = std::abs(traj.linear_vel) / max_speed_;
  
  // 角速度惩罚：只惩罚原地转圈（线速度很小但角速度大）
  // 当朝向误差大时，允许大角速度
  double angular_penalty = 0.0;
  if (std::abs(traj.linear_vel) < 0.2 && std::abs(traj.angular_vel) > 0.3) {
    // 纯旋转：给予惩罚
    angular_penalty = -0.3;
  }
  // 否则不惩罚角速度，允许快速转向
  
  // 3. 障碍物距离代价：轨迹上所有点的最小障碍物距离
  double min_clearance = adjacent_range_;
  for (const auto& point : traj.points) {
    double clearance = getDistanceToObstacle(point);
    if (clearance < min_clearance) {
      min_clearance = clearance;
    }
  }
  traj.clearance_cost = std::min(1.0, min_clearance / (robot_radius_ * 3.0));  // 归一化
  
  // 4. 路径距离代价：轨迹终点到全局路径的距离
  double path_dist = getDistanceToGlobalPath(end_point);
  traj.path_distance_cost = 1.0 - std::min(1.0, path_dist / 2.0);  // 归一化，越接近路径越好
  
  // 综合代价
  traj.cost = heading_cost_weight_ * traj.heading_cost +
              velocity_cost_weight_ * traj.velocity_cost +
              clearance_cost_weight_ * traj.clearance_cost +
              path_distance_cost_weight_ * traj.path_distance_cost +
              angular_penalty;  // 惩罚纯旋转和过大角速度
  
  return traj.cost;
}

Trajectory DWALocalPlanner::computeVelocityCommands()
{
  // 更新全局路径目标
  updateGlobalPathGoal();
  
  // 转换点云到机器人坐标系
  transformPointCloud();
  
  // 调试：输出目标信息
  double dist_to_goal = std::sqrt(std::pow(robot_x_ - goal_x_, 2) + std::pow(robot_y_ - goal_y_, 2));
  double angle_to_goal_global = std::atan2(goal_y_ - robot_y_, goal_x_ - robot_x_);
  double heading_error = normalizeAngle(angle_to_goal_global - robot_yaw_);
  ROS_INFO_THROTTLE(2.0, "Goal: (%.2f, %.2f), Dist: %.2f m, Heading error: %.1f deg", 
                    goal_x_, goal_y_, dist_to_goal, heading_error * 180.0 / PI);
  
  // 生成候选轨迹
  std::vector<Trajectory> trajectories = generateTrajectories();
  
  // 评估轨迹
  Trajectory best_traj;
  best_traj.feasible = false;
  double best_cost = -1.0;
  
  for (auto& traj : trajectories) {
    // 碰撞检测
    if (checkCollision(traj)) {
      traj.feasible = false;
      continue;
    }
    
    // 评估代价
    double cost = evaluateTrajectory(traj);
    
    if (cost > best_cost) {
      best_cost = cost;
      best_traj = traj;
      best_traj.feasible = true;
    }
  }
  
  // 调试：输出最优轨迹信息
  int feasible_count = 0;
  for (const auto& traj : trajectories) {
    if (traj.feasible) feasible_count++;
  }
  
  if (best_traj.feasible) {
    ROS_INFO_THROTTLE(2.0, "Best traj: v=%.2f m/s, w=%.2f deg/s, cost=%.3f (h:%.2f v:%.2f c:%.2f p:%.2f) [%d/%lu feasible]",
                      best_traj.linear_vel, best_traj.angular_vel * 180.0 / PI, best_cost,
                      best_traj.heading_cost, best_traj.velocity_cost, 
                      best_traj.clearance_cost, best_traj.path_distance_cost,
                      feasible_count, trajectories.size());
  } else {
    ROS_WARN_THROTTLE(2.0, "No feasible trajectory! [0/%lu trajectories collision-free]", trajectories.size());
  }
  
  // 可视化
  publishVisualization(trajectories, best_traj);
  
  // 检查是否到达目标点
  if (best_traj.feasible) {
    // 如果非常接近目标点，停车
    if (dist_to_goal < goal_tolerance_) {
      ROS_INFO_THROTTLE(1.0, "Reached goal! Distance: %.2f m", dist_to_goal);
      best_traj.linear_vel = 0.0;
      best_traj.angular_vel = 0.0;
      return best_traj;
    }
    
    // 安全检查：如果前方障碍物过近，强制停车
    double front_clearance = getDistanceToObstacle(geometry_msgs::Point());
    if (front_clearance < stop_clearance_) {
      ROS_WARN("Obstacle too close (%.2f m), stopping!", front_clearance);
      best_traj.linear_vel = 0.0;
      best_traj.angular_vel = 0.0;
    }
  }
  
  // 如果没有可行轨迹，停车
  if (!best_traj.feasible) {
    ROS_WARN("No feasible trajectory found, stopping!");
    best_traj.linear_vel = 0.0;
    best_traj.angular_vel = 0.0;
  }
  
  return best_traj;
}

void DWALocalPlanner::publishVelocityCommand(const Trajectory& traj)
{
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = ros::Time().fromSec(odom_time_);
  cmd_vel.header.frame_id = "vehicle";
  cmd_vel.twist.linear.x = traj.linear_vel;
  cmd_vel.twist.angular.z = traj.angular_vel;
  
  cmd_vel_pub_.publish(cmd_vel);
}

void DWALocalPlanner::publishVisualization(const std::vector<Trajectory>& trajectories, 
                                           const Trajectory& best_traj)
{
  // 发布最佳轨迹
  if (best_traj.feasible && !best_traj.points.empty()) {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "vehicle";
    
    for (const auto& point : best_traj.points) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    
    local_plan_pub_.publish(path);
  }
  
  // 发布候选轨迹（可选，用于调试）
  visualization_msgs::MarkerArray markers;
  int marker_id = 0;
  
  for (const auto& traj : trajectories) {
    if (!traj.feasible || traj.points.empty()) continue;
    
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "vehicle";
    marker.ns = "dwa_candidates";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.3;
    marker.lifetime = ros::Duration(0.1);
    
    for (const auto& point : traj.points) {
      marker.points.push_back(point);
    }
    
    markers.markers.push_back(marker);
    
    // 限制可视化数量
    if (marker_id > 50) break;
  }
  
  candidates_pub_.publish(markers);
}

void DWALocalPlanner::publishGoalMarker()
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "map";
  marker.ns = "goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = goal_x_;
  marker.pose.position.y = goal_y_;
  marker.pose.position.z = 1.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.8;
  marker.scale.y = 0.8;
  marker.scale.z = 0.8;
  marker.color.r = 1.0;
  marker.color.g = 0.6;
  marker.color.b = 0.6;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.5);
  
  goal_marker_pub_.publish(marker);
}

double DWALocalPlanner::normalizeAngle(double angle)
{
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

void DWALocalPlanner::run()
{
  ros::Rate rate(control_rate_);
  
  while (ros::ok()) {
    ros::spinOnce();
    
    // 检查是否收到必要数据
    if (!odom_received_) {
      ROS_WARN_THROTTLE(5.0, "Waiting for odometry data...");
      rate.sleep();
      continue;
    }
    
    if (!pointcloud_received_) {
      ROS_WARN_THROTTLE(5.0, "Waiting for point cloud data...");
      rate.sleep();
      continue;
    }
    
    if (!global_path_received_) {
      ROS_WARN_THROTTLE(5.0, "Waiting for global path...");
      rate.sleep();
      continue;
    }
    
    // 发布目标标记
    publishGoalMarker();
    
    // 计算速度指令
    Trajectory cmd = computeVelocityCommands();
    
    // 发布速度指令
    publishVelocityCommand(cmd);
    
    rate.sleep();
  }
}

} // namespace dwa_local_planner

