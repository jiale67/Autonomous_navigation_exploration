/*
*	File: density_frontier_explorer.h
*	---------------
*   使用terrain_map_ext点云，密集区域=已探索，稀疏边界=前沿
*   Created by rjl on 2025.10
*/

#ifndef DENSITY_FRONTIER_EXPLORER_H
#define DENSITY_FRONTIER_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <vector>
#include <set>

namespace exploration {

// 前沿候选点结构
struct FrontierCandidate {
    Eigen::Vector3d position;
    double density;           // 点云密度
    double distance;          // 到机器人距离
    double direction_score;   // 方向得分
    double score;             // 综合得分
    bool is_valid;
    
    FrontierCandidate() : density(0), distance(0), direction_score(0), 
                         score(0), is_valid(true) {}
};

class DensityFrontierExplorer {
public:
    DensityFrontierExplorer(ros::NodeHandle& nh);
    ~DensityFrontierExplorer();
    
    void run();
    
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber terrain_cloud_sub_;
    
    ros::Publisher frontier_vis_pub_;
    ros::Publisher best_frontier_pub_;
    ros::Publisher exploration_goal_pub_;
    ros::Publisher density_vis_pub_;
    ros::Publisher sector_lines_pub_;
    ros::Publisher global_path_pub_;           // 发布全局路径
    
    ros::Timer exploration_timer_;
    
    // 数据存储
    pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_;
    
    // 机器人状态
    Eigen::Vector3d robot_position_;
    double robot_yaw_;
    bool odom_received_;
    bool cloud_received_;
    
    // 探索状态
    Eigen::Vector3d current_goal_;
    bool has_goal_;
    bool goal_reached_;
    std::set<std::string> visited_goals_;
    std::vector<Eigen::Vector3d> current_global_path_;  // 当前全局路径
    ros::Time last_goal_update_time_;                   // 上次目标更新时间
    ros::Time last_path_replan_time_;                   // 上次路径重规划时间
    
    // 已探索区域记录（解决重复探索问题）
    std::map<std::string, int> explored_regions_;       // 键：网格坐标字符串，值：探索次数
    double exploration_grid_resolution_;                 // 探索区域网格分辨率
    double exploration_radius_;                          // 探索半径（机器人周围多大范围算已探索）
    
    // 参数 - 扇区划分
    int num_sectors_;                    // 扇区数量
    double min_sector_radius_;           // 最小扇区半径
    double max_sector_radius_;           // 最大扇区半径
    double sector_radial_resolution_;    // 径向分辨率
    
    // 参数 - 密度计算
    double density_radius_;              // 密度计算半径
    int dense_threshold_;                // 密集阈值
    int sparse_threshold_;               // 稀疏阈值
    
    // 参数 - 目标选择
    double goal_reached_threshold_;
    double min_goal_distance_;
    double max_goal_distance_;
    
    // 参数 - 评分权重
    double weight_sparsity_;             // 稀疏性权重
    double weight_distance_;             // 距离权重
    double weight_direction_;            // 方向权重
    
    double exploration_rate_;
    bool enable_visualization_;
    bool use_global_path_;                     // 是否使用全局路径规划
    double goal_switch_distance_;              // 目标切换距离阈值
    double goal_reevaluation_interval_;        // 目标重新评估时间间隔(秒)
    double path_replan_interval_;              // 路径重规划时间间隔(秒)
    
    // RRT*路径规划参数
    double rrt_step_size_;
    double rrt_goal_bias_;
    int rrt_max_iterations_;
    double path_smoothing_factor_;
    
    // 碰撞检测参数
    double collision_check_step_;           // 碰撞检测步长
    double robot_radius_;                   // 机器人半径（安全边界）
    double obstacle_height_threshold_;      // 障碍物高度阈值
    double max_path_point_distance_;        // 路径点最大间距
    
    // 回调函数
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void terrainCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void explorationTimerCallback(const ros::TimerEvent& event);
    
    // 核心功能 - 基于密度的前沿检测
    void detectFrontiersBySectorDensity(std::vector<FrontierCandidate>& candidates);
    double calculateDensity(const Eigen::Vector3d& position);
    bool isFrontierPoint(const Eigen::Vector3d& position, double density);
    
    // 目标选择与评估
    void evaluateCandidates(std::vector<FrontierCandidate>& candidates);
    bool selectBestFrontier(const std::vector<FrontierCandidate>& candidates, 
                           Eigen::Vector3d& goal);
    bool isGoalReached();
    void publishExplorationGoal(const Eigen::Vector3d& goal);
    
    // 可视化
    void visualizeFrontiers(const std::vector<FrontierCandidate>& candidates);
    void visualizeBestFrontier(const Eigen::Vector3d& goal);
    void visualizeDensityMap();
    void visualizeSectorLines();
    
    // 全局路径规划
    bool planGlobalPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                       std::vector<Eigen::Vector3d>& path);
    bool isPathCollisionFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    bool isPointCollisionFree(const Eigen::Vector3d& point);
    void smoothPath(std::vector<Eigen::Vector3d>& path);
    void densifyPath(std::vector<Eigen::Vector3d>& path);  // 路径密集化
    void publishGlobalPath(const std::vector<Eigen::Vector3d>& path);
    
    // 已探索区域管理
    void markRegionAsExplored(const Eigen::Vector3d& position);
    bool isRegionExplored(const Eigen::Vector3d& position);
    int getExplorationCount(const Eigen::Vector3d& position);
    std::string positionToGridKey(const Eigen::Vector3d& position);
    
    // 辅助函数
    double getDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    double getDistance2D(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    std::string pointToString(const Eigen::Vector3d& point);
    void printExplorationStatus();
};

} // namespace exploration

#endif // DENSITY_FRONTIER_EXPLORER_H



