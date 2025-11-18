#include "mpc_local_planner/mpc_local_planner.h"

namespace mpc_local_planner
{

MPCLocalPlanner::MPCLocalPlanner()
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

MPCLocalPlanner::~MPCLocalPlanner()
{
}

bool MPCLocalPlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  nh_ = nh;
  nh_private_ = nh_private;
  
  // 加载参数 - 机器人物理属性
  nh_private_.param("robot_radius", robot_radius_, 0.3);
  nh_private_.param("sensor_offset_x", sensor_offset_x_, 0.0);
  nh_private_.param("sensor_offset_y", sensor_offset_y_, 0.0);
  
  // 加载参数 - 运动学约束
  nh_private_.param("max_speed", max_speed_, 2.0);
  nh_private_.param("min_speed", min_speed_, -0.5);
  nh_private_.param("max_yaw_rate", max_yaw_rate_, 90.0);  // 度/秒
  nh_private_.param("max_accel", max_accel_, 2.5);
  nh_private_.param("max_yaw_accel", max_yaw_accel_, 120.0);  // 度/秒^2
  
  // 转换角度到弧度
  max_yaw_rate_ = max_yaw_rate_ * PI / 180.0;
  max_yaw_accel_ = max_yaw_accel_ * PI / 180.0;
  
  // 加载参数 - MPC配置
  nh_private_.param("prediction_horizon", prediction_horizon_, 10);
  nh_private_.param("prediction_dt", prediction_dt_, 0.2);
  nh_private_.param("reference_speed", reference_speed_, 1.5);
  
  // 加载参数 - 传感器
  nh_private_.param("use_terrain_analysis", use_terrain_analysis_, true);
  nh_private_.param("adjacent_range", adjacent_range_, 4.25);
  nh_private_.param("voxel_size", voxel_size_, 0.05);
  nh_private_.param("obstacle_height_thre", obstacle_height_thre_, 0.15);
  
  // 加载参数 - 全局路径跟随
  nh_private_.param("path_lookahead_dist", path_lookahead_dist_, 3.0);
  nh_private_.param("goal_update_distance", goal_update_distance_, 1.0);
  nh_private_.param("goal_tolerance", goal_tolerance_, 0.3);
  nh_private_.param("path_interp_spacing", path_interp_spacing_, 0.1);
  
  // 加载参数 - MPC代价权重
  nh_private_.param("weight_x", weight_x_, 1.0);
  nh_private_.param("weight_y", weight_y_, 1.0);
  nh_private_.param("weight_theta", weight_theta_, 0.5);
  nh_private_.param("weight_v", weight_v_, 0.3);
  nh_private_.param("weight_omega", weight_omega_, 0.1);
  nh_private_.param("weight_delta_v", weight_delta_v_, 0.5);
  nh_private_.param("weight_delta_omega", weight_delta_omega_, 0.3);
  nh_private_.param("weight_obstacle", weight_obstacle_, 2.0);
  
  // 加载参数 - 安全
  nh_private_.param("stop_clearance", stop_clearance_, 0.5);
  nh_private_.param("safety_margin", safety_margin_, 0.1);
  nh_private_.param("min_obstacle_distance", min_obstacle_distance_, 0.5);
  
  // 加载参数 - 其他
  nh_private_.param("control_rate", control_rate_, 20.0);
  nh_private_.param("two_way_drive", two_way_drive_, true);
  nh_private_.param("enable_obstacle_avoidance", enable_obstacle_avoidance_, true);
  
  // 初始化订阅者
  odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/state_estimation", 5, 
                                                  &MPCLocalPlanner::odomCallback, this);
  
  if (use_terrain_analysis_) {
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 5, 
                                                               &MPCLocalPlanner::pointCloudCallback, this);
  } else {
    pointcloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/registered_scan", 5, 
                                                               &MPCLocalPlanner::pointCloudCallback, this);
  }
  
  global_path_sub_ = nh_.subscribe<visualization_msgs::MarkerArray>("/global_planned_path", 5, 
                                                                     &MPCLocalPlanner::globalPathCallback, this);
  
  // 初始化发布者
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 5);
  local_plan_pub_ = nh_.advertise<nav_msgs::Path>("/mpc_local_plan", 5);
  predicted_traj_pub_ = nh_.advertise<nav_msgs::Path>("/mpc_predicted_trajectory", 5);
  goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_marker", 5);
  
  // 配置体素滤波器
  voxel_filter_.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
  
  ROS_INFO("MPC Local Planner initialized successfully");
  ROS_INFO("  Robot radius: %.2f m", robot_radius_);
  ROS_INFO("  Max speed: %.2f m/s, Max yaw rate: %.2f deg/s", max_speed_, max_yaw_rate_ * 180.0 / PI);
  ROS_INFO("  Prediction horizon: %d steps, dt: %.2f s", prediction_horizon_, prediction_dt_);
  ROS_INFO("  Reference speed: %.2f m/s", reference_speed_);
  ROS_INFO("  Use terrain analysis: %s", use_terrain_analysis_ ? "true" : "false");
  
  return true;
}

void MPCLocalPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
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

void MPCLocalPlanner::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // 转换点云
  raw_pointcloud_->clear();
  pcl::fromROSMsg(*msg, *raw_pointcloud_);
  
  pointcloud_received_ = true;
}

void MPCLocalPlanner::globalPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
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
    
    // 插值全局路径
    interpolatePath(global_path_points_, interpolated_path_x_, interpolated_path_y_, path_interp_spacing_);
    
    current_goal_index_ = 1;
    if (current_goal_index_ >= global_path_points_.size()) {
      current_goal_index_ = global_path_points_.size() - 1;
    }
    
    goal_x_ = global_path_points_[current_goal_index_].first;
    goal_y_ = global_path_points_[current_goal_index_].second;
    
    ROS_INFO("Received new global path with %lu points. Interpolated to %lu points. Goal: (%.2f, %.2f)",
             global_path_points_.size(), interpolated_path_x_.size(), goal_x_, goal_y_);
  }
  
  global_path_received_ = true;
}

void MPCLocalPlanner::updateGlobalPathGoal()
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

void MPCLocalPlanner::transformPointCloud()
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

void MPCLocalPlanner::interpolatePath(const std::vector<std::pair<double, double>>& waypoints,
                                      std::vector<double>& x_interp,
                                      std::vector<double>& y_interp,
                                      double spacing)
{
  x_interp.clear();
  y_interp.clear();
  
  if (waypoints.size() < 2) {
    if (waypoints.size() == 1) {
      x_interp.push_back(waypoints[0].first);
      y_interp.push_back(waypoints[0].second);
    }
    return;
  }
  
  // 在相邻航点之间插值
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    double x1 = waypoints[i].first;
    double y1 = waypoints[i].second;
    double x2 = waypoints[i + 1].first;
    double y2 = waypoints[i + 1].second;
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    double segment_length = std::sqrt(dx * dx + dy * dy);
    
    int num_points = std::max(2, static_cast<int>(segment_length / spacing));
    
    for (int j = 0; j < num_points; ++j) {
      double t = static_cast<double>(j) / num_points;
      x_interp.push_back(x1 + t * dx);
      y_interp.push_back(y1 + t * dy);
    }
  }
  
  // 添加最后一个点
  x_interp.push_back(waypoints.back().first);
  y_interp.push_back(waypoints.back().second);
}

int MPCLocalPlanner::findClosestPoint(double x, double y,
                                      const std::vector<double>& path_x,
                                      const std::vector<double>& path_y)
{
  if (path_x.empty()) return -1;
  
  int closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < path_x.size(); ++i) {
    double dist = std::sqrt(std::pow(x - path_x[i], 2) + std::pow(y - path_y[i], 2));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  
  return closest_idx;
}

void MPCLocalPlanner::generateReferenceTrajectory(std::vector<double>& x_ref,
                                                   std::vector<double>& y_ref,
                                                   std::vector<double>& theta_ref,
                                                   std::vector<double>& v_ref)
{
  x_ref.clear();
  y_ref.clear();
  theta_ref.clear();
  v_ref.clear();
  
  if (interpolated_path_x_.empty()) {
    // 如果没有路径，生成直线前进的参考轨迹
    for (int i = 0; i < prediction_horizon_; ++i) {
      x_ref.push_back(0.0);
      y_ref.push_back(0.0);
      theta_ref.push_back(0.0);
      v_ref.push_back(reference_speed_);
    }
    return;
  }
  
  // 找到最近的路径点
  int start_idx = findClosestPoint(robot_x_, robot_y_, interpolated_path_x_, interpolated_path_y_);
  
  // 从最近点开始，沿路径生成参考轨迹
  for (int i = 0; i < prediction_horizon_; ++i) {
    int path_idx = start_idx + i;
    
    if (path_idx < interpolated_path_x_.size()) {
      // 转换到机器人局部坐标系
      double x_global = interpolated_path_x_[path_idx];
      double y_global = interpolated_path_y_[path_idx];
      
      double dx = x_global - robot_x_;
      double dy = y_global - robot_y_;
      double x_local = dx * std::cos(robot_yaw_) + dy * std::sin(robot_yaw_);
      double y_local = -dx * std::sin(robot_yaw_) + dy * std::cos(robot_yaw_);
      
      x_ref.push_back(x_local);
      y_ref.push_back(y_local);
      
      // 计算参考航向（指向下一个点）
      if (path_idx + 1 < interpolated_path_x_.size()) {
        double next_x = interpolated_path_x_[path_idx + 1];
        double next_y = interpolated_path_y_[path_idx + 1];
        double theta_global = std::atan2(next_y - y_global, next_x - x_global);
        double theta_local = normalizeAngle(theta_global - robot_yaw_);
        theta_ref.push_back(theta_local);
      } else {
        theta_ref.push_back(0.0);
      }
      
      v_ref.push_back(reference_speed_);
    } else {
      // 路径结束，保持最后一个点
      if (!x_ref.empty()) {
        x_ref.push_back(x_ref.back());
        y_ref.push_back(y_ref.back());
        theta_ref.push_back(theta_ref.back());
        v_ref.push_back(0.0);  // 到达终点后停止
      } else {
        x_ref.push_back(0.0);
        y_ref.push_back(0.0);
        theta_ref.push_back(0.0);
        v_ref.push_back(0.0);
      }
    }
  }
}

void MPCLocalPlanner::updateState(double& x, double& y, double& theta, double v, double omega, double dt)
{
  // 差分驱动运动学模型
  x += v * std::cos(theta) * dt;
  y += v * std::sin(theta) * dt;
  theta += omega * dt;
  theta = normalizeAngle(theta);
}

double MPCLocalPlanner::getDistanceToObstacle(double x_local, double y_local)
{
  if (transformed_pointcloud_->points.empty()) {
    return adjacent_range_;  // 无点云，返回最大距离
  }
  
  pcl::PointXYZI search_point;
  search_point.x = x_local;
  search_point.y = y_local;
  search_point.z = 0.0;
  
  std::vector<int> point_indices(1);
  std::vector<float> point_distances(1);
  
  if (kdtree_.nearestKSearch(search_point, 1, point_indices, point_distances) > 0) {
    return std::sqrt(point_distances[0]);
  }
  
  return adjacent_range_;
}

double MPCLocalPlanner::getDistanceToGlobalPath(double x_local, double y_local)
{
  if (interpolated_path_x_.empty()) {
    return 0.0;
  }
  
  // 将局部坐标转换回全局坐标
  double x_global = robot_x_ + x_local * std::cos(robot_yaw_) - y_local * std::sin(robot_yaw_);
  double y_global = robot_y_ + x_local * std::sin(robot_yaw_) + y_local * std::cos(robot_yaw_);
  
  // 找到全局路径上最近的点
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < interpolated_path_x_.size(); ++i) {
    double dist = std::sqrt(std::pow(x_global - interpolated_path_x_[i], 2) + 
                           std::pow(y_global - interpolated_path_y_[i], 2));
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  
  return min_dist;
}

bool MPCLocalPlanner::solveMPC(double& v_cmd, double& w_cmd)
{
  // 生成参考轨迹
  std::vector<double> x_ref, y_ref, theta_ref, v_ref;
  generateReferenceTrajectory(x_ref, y_ref, theta_ref, v_ref);
  
  // 简化的MPC求解器：使用梯度下降法优化控制序列
  // 这是一个基础实现，实际应用中可以使用更高级的QP求解器
  
  int N = prediction_horizon_;
  std::vector<double> v_seq(N, current_linear_vel_);
  std::vector<double> w_seq(N, current_angular_vel_);
  
  // 迭代优化 - 增加迭代次数和使用自适应学习率
  int max_iterations = 30;
  double base_learning_rate = 0.15;  // 基础学习率
  double prev_cost = std::numeric_limits<double>::max();
  
  for (int iter = 0; iter < max_iterations; ++iter) {
    // 自适应学习率：随迭代衰减
    double learning_rate = base_learning_rate * (1.0 - 0.5 * iter / max_iterations);
    // 前向模拟预测轨迹
    std::vector<double> x_pred(N + 1), y_pred(N + 1), theta_pred(N + 1);
    x_pred[0] = 0.0;  // 在局部坐标系中，初始位置为原点
    y_pred[0] = 0.0;
    theta_pred[0] = 0.0;
    
    for (int i = 0; i < N; ++i) {
      x_pred[i + 1] = x_pred[i];
      y_pred[i + 1] = y_pred[i];
      theta_pred[i + 1] = theta_pred[i];
      updateState(x_pred[i + 1], y_pred[i + 1], theta_pred[i + 1], 
                  v_seq[i], w_seq[i], prediction_dt_);
    }
    
    // 计算代价梯度
    std::vector<double> grad_v(N, 0.0);
    std::vector<double> grad_w(N, 0.0);
    
    for (int i = 0; i < N; ++i) {
      // 位置误差梯度
      double dx = x_pred[i + 1] - x_ref[i];
      double dy = y_pred[i + 1] - y_ref[i];
      double dtheta = normalizeAngle(theta_pred[i + 1] - theta_ref[i]);
      
      // 简化梯度计算（数值梯度）
      grad_v[i] = weight_x_ * dx * std::cos(theta_pred[i]) * prediction_dt_ +
                  weight_y_ * dy * std::sin(theta_pred[i]) * prediction_dt_;
      
      grad_w[i] = weight_theta_ * dtheta * prediction_dt_;
      
      // 速度代价梯度
      grad_v[i] += weight_v_ * (v_seq[i] - v_ref[i]);
      grad_w[i] += weight_omega_ * w_seq[i];
      
      // 控制变化梯度
      if (i > 0) {
        grad_v[i] += weight_delta_v_ * (v_seq[i] - v_seq[i - 1]);
        grad_w[i] += weight_delta_omega_ * (w_seq[i] - w_seq[i - 1]);
      } else {
        grad_v[i] += weight_delta_v_ * (v_seq[i] - current_linear_vel_);
        grad_w[i] += weight_delta_omega_ * (w_seq[i] - current_angular_vel_);
      }
      
      // 障碍物代价梯度（增强版）
      if (enable_obstacle_avoidance_) {
        double obs_dist = getDistanceToObstacle(x_pred[i + 1], y_pred[i + 1]);
        if (obs_dist < min_obstacle_distance_ * 2.0) {
          // 使用指数惩罚，距离越近惩罚越大
          double dist_ratio = obs_dist / (min_obstacle_distance_ * 2.0);
          double penalty = weight_obstacle_ * (1.0 - dist_ratio) * (1.0 - dist_ratio);
          
          // 减速梯度
          grad_v[i] += penalty * std::cos(theta_pred[i]) * prediction_dt_;
          
          // 检查左右两侧，引导转向
          double left_dist = getDistanceToObstacle(x_pred[i + 1], y_pred[i + 1] + robot_radius_);
          double right_dist = getDistanceToObstacle(x_pred[i + 1], y_pred[i + 1] - robot_radius_);
          
          if (left_dist > right_dist) {
            // 左侧空间更大，增加左转梯度
            grad_w[i] -= penalty * 0.5;
          } else {
            // 右侧空间更大，增加右转梯度
            grad_w[i] += penalty * 0.5;
          }
        }
      }
    }
    
    // 计算当前代价（用于早停判断）
    double current_cost = 0.0;
    for (int i = 0; i < N; ++i) {
      double dx = x_pred[i + 1] - x_ref[i];
      double dy = y_pred[i + 1] - y_ref[i];
      double dtheta = normalizeAngle(theta_pred[i + 1] - theta_ref[i]);
      current_cost += weight_x_ * dx * dx + weight_y_ * dy * dy + weight_theta_ * dtheta * dtheta;
    }
    
    // 早停：如果代价收敛则提前退出
    if (iter > 5 && std::abs(current_cost - prev_cost) < 0.01) {
      break;
    }
    prev_cost = current_cost;
    
    // 更新控制序列
    for (int i = 0; i < N; ++i) {
      v_seq[i] -= learning_rate * grad_v[i];
      w_seq[i] -= learning_rate * grad_w[i];
      
      // 约束限制
      v_seq[i] = std::max(min_speed_, std::min(max_speed_, v_seq[i]));
      w_seq[i] = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, w_seq[i]));
    }
  }
  
  // 应用控制约束（加速度限制）
  double control_dt = 1.0 / control_rate_;
  double max_dv = max_accel_ * control_dt;
  double max_dw = max_yaw_accel_ * control_dt;
  
  v_cmd = v_seq[0];
  w_cmd = w_seq[0];
  
  // 限制加速度
  double dv = v_cmd - current_linear_vel_;
  double dw = w_cmd - current_angular_vel_;
  
  if (std::abs(dv) > max_dv) {
    v_cmd = current_linear_vel_ + (dv > 0 ? max_dv : -max_dv);
  }
  
  if (std::abs(dw) > max_dw) {
    w_cmd = current_angular_vel_ + (dw > 0 ? max_dw : -max_dw);
  }
  
  // 急转弯优化：当需要大角度转向时，适当降低线速度以提高转向灵活性
  if (std::abs(w_cmd) > max_yaw_rate_ * 0.5) {
    double turn_ratio = std::abs(w_cmd) / max_yaw_rate_;
    double speed_reduction = 1.0 - 0.4 * turn_ratio;  // 最多降低40%速度
    v_cmd *= speed_reduction;
  }
  
  // 发布预测轨迹用于可视化
  MPCTrajectory pred_traj;
  pred_traj.x.push_back(0.0);
  pred_traj.y.push_back(0.0);
  pred_traj.theta.push_back(0.0);
  
  for (int i = 0; i < N; ++i) {
    double x = pred_traj.x.back();
    double y = pred_traj.y.back();
    double theta = pred_traj.theta.back();
    updateState(x, y, theta, v_seq[i], w_seq[i], prediction_dt_);
    pred_traj.x.push_back(x);
    pred_traj.y.push_back(y);
    pred_traj.theta.push_back(theta);
  }
  
  pred_traj.feasible = true;
  publishVisualization(pred_traj);
  
  return true;
}

bool MPCLocalPlanner::computeVelocityCommands(double& v_cmd, double& w_cmd)
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
  
  // 检查是否到达目标
  if (dist_to_goal < goal_tolerance_) {
    ROS_INFO_THROTTLE(1.0, "Reached goal! Distance: %.2f m", dist_to_goal);
    v_cmd = 0.0;
    w_cmd = 0.0;
    return true;
  }
  
  // 智能障碍物处理：检查多个方向的障碍物距离
  double front_clearance = getDistanceToObstacle(0.0, 0.0);
  double left_clearance = getDistanceToObstacle(0.0, robot_radius_ * 1.5);
  double right_clearance = getDistanceToObstacle(0.0, -robot_radius_ * 1.5);
  
  // 如果前方非常接近障碍物（小于机器人半径），尝试原地转向
  if (front_clearance < robot_radius_ * 1.2) {
    ROS_WARN_THROTTLE(1.0, "Obstacle very close (%.2f m), rotating to find escape route!", front_clearance);
    
    // 选择障碍物较少的方向转向
    if (left_clearance > right_clearance) {
      v_cmd = 0.0;
      w_cmd = max_yaw_rate_ * 0.5;  // 向左转
      ROS_INFO_THROTTLE(1.0, "Turning left (left: %.2f m, right: %.2f m)", left_clearance, right_clearance);
    } else {
      v_cmd = 0.0;
      w_cmd = -max_yaw_rate_ * 0.5;  // 向右转
      ROS_INFO_THROTTLE(1.0, "Turning right (left: %.2f m, right: %.2f m)", left_clearance, right_clearance);
    }
    return true;
  }
  
  // 如果前方障碍物较近但还有空间，允许慢速前进并转向
  if (front_clearance < stop_clearance_) {
    ROS_WARN_THROTTLE(1.0, "Obstacle close (%.2f m), slow navigation with turning!", front_clearance);
    
    // 求解MPC但限制最大速度
    bool success = solveMPC(v_cmd, w_cmd);
    if (success) {
      // 限制速度为较低值
      double max_safe_speed = 0.3;
      v_cmd = std::max(-max_safe_speed, std::min(max_safe_speed, v_cmd));
      
      // 增强转向能力以避开障碍
      if (left_clearance > right_clearance && left_clearance > front_clearance) {
        w_cmd += max_yaw_rate_ * 0.3;  // 增加左转
      } else if (right_clearance > front_clearance) {
        w_cmd -= max_yaw_rate_ * 0.3;  // 增加右转
      }
      
      // 限制角速度
      w_cmd = std::max(-max_yaw_rate_, std::min(max_yaw_rate_, w_cmd));
      
      ROS_INFO_THROTTLE(1.0, "Adjusted cmd: v=%.2f m/s, w=%.2f deg/s", v_cmd, w_cmd * 180.0 / PI);
    } else {
      // MPC失败，尝试原地转向
      v_cmd = 0.0;
      w_cmd = (left_clearance > right_clearance) ? max_yaw_rate_ * 0.3 : -max_yaw_rate_ * 0.3;
    }
    return true;
  }
  
  // 正常情况：求解MPC
  bool success = solveMPC(v_cmd, w_cmd);
  
  if (success) {
    ROS_INFO_THROTTLE(2.0, "MPC solution: v=%.2f m/s, w=%.2f deg/s", 
                      v_cmd, w_cmd * 180.0 / PI);
  } else {
    ROS_WARN("MPC solver failed, stopping!");
    v_cmd = 0.0;
    w_cmd = 0.0;
  }
  
  return success;
}

void MPCLocalPlanner::publishVelocityCommand(double v, double w)
{
  geometry_msgs::TwistStamped cmd_vel;
  cmd_vel.header.stamp = ros::Time().fromSec(odom_time_);
  cmd_vel.header.frame_id = "vehicle";
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.angular.z = w;
  
  cmd_vel_pub_.publish(cmd_vel);
}

void MPCLocalPlanner::publishVisualization(const MPCTrajectory& traj)
{
  if (!traj.feasible || traj.x.empty()) return;
  
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "vehicle";
  
  for (size_t i = 0; i < traj.x.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = traj.x[i];
    pose.pose.position.y = traj.y[i];
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  
  predicted_traj_pub_.publish(path);
}

void MPCLocalPlanner::publishGoalMarker()
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
  marker.color.b = 0.2;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(0.5);
  
  goal_marker_pub_.publish(marker);
}

double MPCLocalPlanner::normalizeAngle(double angle)
{
  while (angle > PI) angle -= 2.0 * PI;
  while (angle < -PI) angle += 2.0 * PI;
  return angle;
}

void MPCLocalPlanner::run()
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
    double v_cmd = 0.0, w_cmd = 0.0;
    computeVelocityCommands(v_cmd, w_cmd);
    
    // 发布速度指令
    publishVelocityCommand(v_cmd, w_cmd);
    
    rate.sleep();
  }
}

} // namespace mpc_local_planner
