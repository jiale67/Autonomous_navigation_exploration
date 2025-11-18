#ifndef DWA_LOCAL_PLANNER_H
#define DWA_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <algorithm>
#include <cmath>

namespace dwa_local_planner
{

// 轨迹结构体
struct Trajectory
{
  double linear_vel;   // 线速度
  double angular_vel;  // 角速度
  std::vector<geometry_msgs::Point> points;  // 轨迹点
  double cost;         // 总代价
  bool feasible;       // 是否可行（无碰撞）
  
  // 各项代价分量
  double heading_cost;
  double velocity_cost;
  double clearance_cost;
  double path_distance_cost;
  
  Trajectory() : linear_vel(0.0), angular_vel(0.0), cost(0.0), feasible(true),
                 heading_cost(0.0), velocity_cost(0.0), clearance_cost(0.0), 
                 path_distance_cost(0.0) {}
};

// DWA 局部规划器类
class DWALocalPlanner
{
public:
  DWALocalPlanner();
  ~DWALocalPlanner();
  
  // 初始化
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  
  // 主循环
  void run();

private:
  // 回调函数
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void globalPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  
  // DWA 核心算法
  Trajectory computeVelocityCommands();
  std::vector<Trajectory> generateTrajectories();
  Trajectory simulateTrajectory(double v, double w);
  bool checkCollision(const Trajectory& traj);
  double evaluateTrajectory(Trajectory& traj);
  
  // 辅助函数
  void updateGlobalPathGoal();
  void publishVelocityCommand(const Trajectory& traj);
  void publishVisualization(const std::vector<Trajectory>& trajectories, const Trajectory& best_traj);
  void publishGoalMarker();
  double normalizeAngle(double angle);
  double getDistanceToObstacle(const geometry_msgs::Point& point);
  double getDistanceToGlobalPath(const geometry_msgs::Point& point);
  void transformPointCloud();
  
  // ROS 相关
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  
  // 订阅者
  ros::Subscriber odom_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber global_path_sub_;
  
  // 发布者
  ros::Publisher cmd_vel_pub_;
  ros::Publisher local_plan_pub_;
  ros::Publisher candidates_pub_;
  ros::Publisher goal_marker_pub_;
  
  // 参数 - 机器人物理属性
  double robot_radius_;
  double sensor_offset_x_;
  double sensor_offset_y_;
  
  // 参数 - 运动学约束
  double max_speed_;
  double min_speed_;
  double max_yaw_rate_;
  double max_accel_;
  double max_yaw_accel_;
  
  // 参数 - DWA 采样
  double sim_time_;
  double sim_dt_;
  int vx_samples_;
  int wz_samples_;
  
  // 参数 - 传感器
  bool use_terrain_analysis_;
  double adjacent_range_;
  double voxel_size_;
  double obstacle_height_thre_;
  
  // 参数 - 全局路径跟随
  double path_lookahead_dist_;
  double goal_update_distance_;
  double goal_tolerance_;
  
  // 参数 - 代价函数权重
  double heading_cost_weight_;
  double velocity_cost_weight_;
  double clearance_cost_weight_;
  double path_distance_cost_weight_;
  
  // 参数 - 安全
  double stop_clearance_;
  double safety_margin_;
  
  // 参数 - 其他
  double control_rate_;
  bool two_way_drive_;
  
  // 状态变量
  bool odom_received_;
  bool pointcloud_received_;
  bool global_path_received_;
  
  // 机器人当前状态
  double robot_x_;
  double robot_y_;
  double robot_yaw_;
  double robot_roll_;
  double robot_pitch_;
  double current_linear_vel_;
  double current_angular_vel_;
  double odom_time_;
  
  // 全局路径
  std::vector<std::pair<double, double>> global_path_points_;
  int current_goal_index_;
  double goal_x_;
  double goal_y_;
  std::pair<double, double> last_global_path_endpoint_;
  
  // 点云数据
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pointcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cropped_pointcloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pointcloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
  
  // 常量
  static constexpr double PI = 3.1415926535897932;
};

} // namespace dwa_local_planner

#endif // DWA_LOCAL_PLANNER_H








