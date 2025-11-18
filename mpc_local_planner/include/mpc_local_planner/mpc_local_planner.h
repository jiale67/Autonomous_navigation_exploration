#ifndef MPC_LOCAL_PLANNER_H
#define MPC_LOCAL_PLANNER_H

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

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <vector>
#include <algorithm>
#include <cmath>

namespace mpc_local_planner
{

// MPC 轨迹结构体
struct MPCTrajectory
{
  std::vector<double> x;           // X位置序列
  std::vector<double> y;           // Y位置序列
  std::vector<double> theta;       // 航向序列
  std::vector<double> v;           // 线速度序列
  std::vector<double> omega;       // 角速度序列
  double cost;                     // 总代价
  bool feasible;                   // 是否可行
  
  MPCTrajectory() : cost(0.0), feasible(false) {}
};

// MPC 局部规划器类
class MPCLocalPlanner
{
public:
  MPCLocalPlanner();
  ~MPCLocalPlanner();
  
  // 初始化
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  
  // 主循环
  void run();

private:
  // 回调函数
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void globalPathCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
  
  // MPC 核心算法
  bool computeVelocityCommands(double& v_cmd, double& w_cmd);
  bool solveMPC(double& v_cmd, double& w_cmd);
  void generateReferenceTrajectory(std::vector<double>& x_ref, 
                                   std::vector<double>& y_ref,
                                   std::vector<double>& theta_ref,
                                   std::vector<double>& v_ref);
  
  // 运动学模型
  void updateState(double& x, double& y, double& theta, double v, double omega, double dt);
  
  // 辅助函数
  void updateGlobalPathGoal();
  void publishVelocityCommand(double v, double w);
  void publishVisualization(const MPCTrajectory& traj);
  void publishGoalMarker();
  double normalizeAngle(double angle);
  double getDistanceToObstacle(double x_local, double y_local);
  double getDistanceToGlobalPath(double x_local, double y_local);
  void transformPointCloud();
  
  // 路径插值
  void interpolatePath(const std::vector<std::pair<double, double>>& waypoints,
                      std::vector<double>& x_interp,
                      std::vector<double>& y_interp,
                      double spacing);
  
  // 寻找最近路径点
  int findClosestPoint(double x, double y, 
                       const std::vector<double>& path_x,
                       const std::vector<double>& path_y);
  
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
  ros::Publisher predicted_traj_pub_;
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
  
  // 参数 - MPC配置
  int prediction_horizon_;        // 预测步数 N
  double prediction_dt_;          // 预测时间步长
  double reference_speed_;        // 参考速度
  
  // 参数 - 传感器
  bool use_terrain_analysis_;
  double adjacent_range_;
  double voxel_size_;
  double obstacle_height_thre_;
  
  // 参数 - 全局路径跟随
  double path_lookahead_dist_;
  double goal_update_distance_;
  double goal_tolerance_;
  double path_interp_spacing_;    // 路径插值间距
  
  // 参数 - MPC代价权重
  double weight_x_;               // X位置误差权重
  double weight_y_;               // Y位置误差权重
  double weight_theta_;           // 航向误差权重
  double weight_v_;               // 速度跟踪权重
  double weight_omega_;           // 角速度控制权重
  double weight_delta_v_;         // 加速度变化权重
  double weight_delta_omega_;     // 角加速度变化权重
  double weight_obstacle_;        // 障碍物距离权重
  
  // 参数 - 安全
  double stop_clearance_;
  double safety_margin_;
  double min_obstacle_distance_;  // 最小安全距离
  
  // 参数 - 其他
  double control_rate_;
  bool two_way_drive_;
  bool enable_obstacle_avoidance_;
  
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
  std::vector<double> interpolated_path_x_;
  std::vector<double> interpolated_path_y_;
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
  static constexpr double EPSILON = 1e-6;
};

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_H
