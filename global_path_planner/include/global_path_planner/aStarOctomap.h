/*
*	File: aStarOctomap.h
*	---------------
*   A* planner class based on Octomap for globally optimal path planning.
*   Created on 2025
*/
#ifndef ASTAROCTOMAP_H
#define ASTAROCTOMAP_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <algorithm>

namespace globalPlanner {

// 3D点结构
struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    
    bool operator==(const Point3D& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
    
    double distanceTo(const Point3D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        double dz = z - other.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
};

// 哈希函数
struct Point3DHash {
    std::size_t operator()(const Point3D& p) const {
        int ix = static_cast<int>(std::round(p.x / 0.1));
        int iy = static_cast<int>(std::round(p.y / 0.1));
        int iz = static_cast<int>(std::round(p.z / 0.1));
        return std::hash<int>()(ix) ^ (std::hash<int>()(iy) << 1) ^ (std::hash<int>()(iz) << 2);
    }
};

// A*节点
struct AStarNode {
    Point3D point;
    double g_cost;
    double h_cost;
    double f_cost;
    Point3D parent;
    
    AStarNode() : g_cost(0), h_cost(0), f_cost(0) {}
    AStarNode(const Point3D& p) : point(p), g_cost(0), h_cost(0), f_cost(0) {}
    
    bool operator>(const AStarNode& other) const {
        return f_cost > other.f_cost;
    }
};

class AStarOctomap {
private:
    ros::NodeHandle nh_;
    ros::Subscriber mapSub_;
    ros::Publisher pathPub_;
    ros::Publisher pathVisPub_;
    ros::Publisher searchSpacePub_;
    
    std::shared_ptr<octomap::OcTree> octree_;
    bool mapReceived_;
    
    double mapResolution_;
    double searchResolution_;
    std::vector<double> collisionBox_;
    std::vector<double> envBox_;
    double heuristicWeight_;
    double maxPlanningTime_;
    bool use3D_;
    double fixedHeight_;
    bool visSearchSpace_;
    
    Point3D start_;
    Point3D goal_;
    
    std::vector<Point3D> neighbors_;
    
    // 拐点提取相关参数
    ros::Publisher waypointsPub_;  // MarkerArray发布器
    double turningAngleThreshold_;  // 拐点角度阈值（度）
    double minWaypointDistance_;    // 相邻航点最小距离
    
public:
    AStarOctomap(const ros::NodeHandle& nh);
    
    void mapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
    void updateStart(const std::vector<double>& start);
    void updateGoal(const std::vector<double>& goal);
    bool makePlan(nav_msgs::Path& path);
    bool isCollisionFree(const Point3D& point) const;
    bool isLineCollisionFree(const Point3D& p1, const Point3D& p2) const;
    double heuristic(const Point3D& p1, const Point3D& p2) const;
    std::vector<Point3D> getNeighbors(const Point3D& point) const;
    void smoothPath(std::vector<Point3D>& path);
    void visualizePath(const std::vector<Point3D>& path);
    void visualizeSearchSpace(const std::unordered_set<Point3D, Point3DHash>& closedSet);
    void initializeNeighbors();
    Point3D snapToGrid(const Point3D& point) const;
    bool isInBounds(const Point3D& point) const;
    
    // 拐点提取和MarkerArray发布
    std::vector<Point3D> extractKeyWaypoints(const std::vector<Point3D>& path);
    void publishWaypointsAsMarkerArray(const std::vector<Point3D>& waypoints);
    double calculateAngle(const Point3D& p1, const Point3D& p2, const Point3D& p3) const;
};

// ============= 实现 ===================

AStarOctomap::AStarOctomap(const ros::NodeHandle& nh) : nh_(nh), mapReceived_(false) {
    nh_.param("map_resolution", mapResolution_, 0.2);
    nh_.param("search_resolution", searchResolution_, 0.3);
    nh_.param("heuristic_weight", heuristicWeight_, 1.0);
    nh_.param("max_planning_time", maxPlanningTime_, 5.0);
    nh_.param("use_3d_search", use3D_, false);
    nh_.param("fixed_height", fixedHeight_, 1.0);
    nh_.param("visualize_search_space", visSearchSpace_, true);
    
    if (!nh_.getParam("collision_box", collisionBox_)) {
        collisionBox_ = {1.0, 1.0, 0.6};
    }
    
    if (!nh_.getParam("env_box", envBox_)) {
        envBox_ = {-100, 100, -100, 100, 0, 3.0};
    }
    
    mapSub_ = nh_.subscribe("/octomap_binary", 1, &AStarOctomap::mapCallback, this);
    pathPub_ = nh_.advertise<nav_msgs::Path>("/astar/global_path", 10);
    pathVisPub_ = nh_.advertise<visualization_msgs::Marker>("/astar/path_marker", 10);
    searchSpacePub_ = nh_.advertise<visualization_msgs::MarkerArray>("/astar/search_space", 10);
    
    // 拐点提取参数
    nh_.param("turning_angle_threshold", turningAngleThreshold_, 15.0);
    nh_.param("min_waypoint_distance", minWaypointDistance_, 1.5);
    
    // 发布MarkerArray给local_planner
    waypointsPub_ = nh_.advertise<visualization_msgs::MarkerArray>("/global_planned_path", 10);
    
    initializeNeighbors();
    
    ROS_INFO("[A* Planner]: Initialized with search resolution: %.2fm", searchResolution_);
    ROS_INFO("[A* Planner]: 3D search: %s", use3D_ ? "ENABLED" : "DISABLED");
    ROS_INFO("[A* Planner]: Waypoint extraction - angle threshold: %.1f deg, min distance: %.2fm", 
             turningAngleThreshold_, minWaypointDistance_);
}

void AStarOctomap::initializeNeighbors() {
    neighbors_.clear();
    
    if (use3D_) {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    neighbors_.emplace_back(dx * searchResolution_, 
                                           dy * searchResolution_, 
                                           dz * searchResolution_);
                }
            }
        }
    } else {
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                neighbors_.emplace_back(dx * searchResolution_, 
                                       dy * searchResolution_, 
                                       0);
            }
        }
    }
}

void AStarOctomap::mapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    if (tree) {
        octree_.reset(dynamic_cast<octomap::OcTree*>(tree));
        mapResolution_ = octree_->getResolution();
        mapReceived_ = true;
        ROS_INFO_ONCE("[A* Planner]: Octomap received with resolution: %.3fm", mapResolution_);
    }
}

void AStarOctomap::updateStart(const std::vector<double>& start) {
    start_ = Point3D(start[0], start[1], use3D_ ? start[2] : fixedHeight_);
    start_ = snapToGrid(start_);
}

void AStarOctomap::updateGoal(const std::vector<double>& goal) {
    goal_ = Point3D(goal[0], goal[1], use3D_ ? goal[2] : fixedHeight_);
    goal_ = snapToGrid(goal_);
}

Point3D AStarOctomap::snapToGrid(const Point3D& point) const {
    double x = std::round(point.x / searchResolution_) * searchResolution_;
    double y = std::round(point.y / searchResolution_) * searchResolution_;
    double z = use3D_ ? std::round(point.z / searchResolution_) * searchResolution_ : fixedHeight_;
    return Point3D(x, y, z);
}

bool AStarOctomap::isInBounds(const Point3D& point) const {
    return point.x >= envBox_[0] && point.x <= envBox_[1] &&
           point.y >= envBox_[2] && point.y <= envBox_[3] &&
           point.z >= envBox_[4] && point.z <= envBox_[5];
}

bool AStarOctomap::isCollisionFree(const Point3D& point) const {
    if (!mapReceived_ || !octree_) return true;
    if (!isInBounds(point)) return false;
    
    double half_x = collisionBox_[0] / 2.0;
    double half_y = collisionBox_[1] / 2.0;
    double half_z = collisionBox_[2] / 2.0;
    
    int steps = 3;
    for (int ix = 0; ix < steps; ++ix) {
        for (int iy = 0; iy < steps; ++iy) {
            for (int iz = 0; iz < steps; ++iz) {
                double x = point.x + (ix / (steps - 1.0) - 0.5) * 2 * half_x;
                double y = point.y + (iy / (steps - 1.0) - 0.5) * 2 * half_y;
                double z = point.z + (iz / (steps - 1.0) - 0.5) * 2 * half_z;
                
                octomap::OcTreeNode* node = octree_->search(x, y, z);
                if (node && octree_->isNodeOccupied(node)) {
                    return false;
                }
            }
        }
    }
    return true;
}

bool AStarOctomap::isLineCollisionFree(const Point3D& p1, const Point3D& p2) const {
    double dist = p1.distanceTo(p2);
    int steps = std::max(3, static_cast<int>(dist / (searchResolution_ / 2.0)));
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        Point3D p(p1.x + t * (p2.x - p1.x),
                  p1.y + t * (p2.y - p1.y),
                  p1.z + t * (p2.z - p1.z));
        if (!isCollisionFree(p)) {
            return false;
        }
    }
    return true;
}

double AStarOctomap::heuristic(const Point3D& p1, const Point3D& p2) const {
    return p1.distanceTo(p2) * heuristicWeight_;
}

std::vector<Point3D> AStarOctomap::getNeighbors(const Point3D& point) const {
    std::vector<Point3D> validNeighbors;
    for (const auto& offset : neighbors_) {
        Point3D neighbor(point.x + offset.x, 
                        point.y + offset.y, 
                        point.z + offset.z);
        
        if (isInBounds(neighbor) && isCollisionFree(neighbor)) {
            validNeighbors.push_back(neighbor);
        }
    }
    return validNeighbors;
}

bool AStarOctomap::makePlan(nav_msgs::Path& path) {
    path.poses.clear();
    
    if (!mapReceived_) {
        ROS_WARN("[A* Planner]: No map received yet!");
        return false;
    }
    
    if (!isCollisionFree(start_)) {
        ROS_ERROR("[A* Planner]: Start position is in collision!");
        return false;
    }
    if (!isCollisionFree(goal_)) {
        ROS_ERROR("[A* Planner]: Goal position is in collision!");
        return false;
    }
    
    ROS_INFO("[A* Planner]: Planning from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)",
             start_.x, start_.y, start_.z, goal_.x, goal_.y, goal_.z);
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
    std::unordered_set<Point3D, Point3DHash> closedSet;
    std::unordered_map<Point3D, Point3D, Point3DHash> cameFrom;
    std::unordered_map<Point3D, double, Point3DHash> gScore;
    
    AStarNode startNode(start_);
    startNode.g_cost = 0;
    startNode.h_cost = heuristic(start_, goal_);
    startNode.f_cost = startNode.h_cost;
    
    openSet.push(startNode);
    gScore[start_] = 0;
    
    ros::Time startTime = ros::Time::now();
    int iterations = 0;
    
    while (!openSet.empty() && ros::ok()) {
        if ((ros::Time::now() - startTime).toSec() > maxPlanningTime_) {
            ROS_WARN("[A* Planner]: Planning timeout after %.2fs", maxPlanningTime_);
            break;
        }
        
        AStarNode current = openSet.top();
        openSet.pop();
        
        if (current.point.distanceTo(goal_) < searchResolution_) {
            ROS_INFO("[A* Planner]: Goal reached! Iterations: %d, Time: %.3fs", 
                     iterations, (ros::Time::now() - startTime).toSec());
            
            std::vector<Point3D> rawPath;
            Point3D curr = current.point;
            while (!(curr == start_)) {
                rawPath.push_back(curr);
                curr = cameFrom[curr];
            }
            rawPath.push_back(start_);
            std::reverse(rawPath.begin(), rawPath.end());
            
            smoothPath(rawPath);
            
            path.header.frame_id = "map";
            path.header.stamp = ros::Time::now();
            for (const auto& p : rawPath) {
                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position.x = p.x;
                pose.pose.position.y = p.y;
                pose.pose.position.z = p.z;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
            }
            
            visualizePath(rawPath);
            if (visSearchSpace_) {
                visualizeSearchSpace(closedSet);
            }
            
            return true;
        }
        
        if (closedSet.count(current.point)) continue;
        closedSet.insert(current.point);
        
        for (const auto& neighbor : getNeighbors(current.point)) {
            if (closedSet.count(neighbor)) continue;
            
            double tentative_g = gScore[current.point] + current.point.distanceTo(neighbor);
            
            if (!gScore.count(neighbor) || tentative_g < gScore[neighbor]) {
                cameFrom[neighbor] = current.point;
                gScore[neighbor] = tentative_g;
                
                AStarNode neighborNode(neighbor);
                neighborNode.g_cost = tentative_g;
                neighborNode.h_cost = heuristic(neighbor, goal_);
                neighborNode.f_cost = neighborNode.g_cost + neighborNode.h_cost;
                
                openSet.push(neighborNode);
            }
        }
        
        ++iterations;
    }
    
    ROS_ERROR("[A* Planner]: Failed to find path after %d iterations", iterations);
    return false;
}

void AStarOctomap::smoothPath(std::vector<Point3D>& path) {
    if (path.size() < 3) return;
    
    std::vector<Point3D> smoothed;
    smoothed.push_back(path.front());
    
    size_t i = 0;
    while (i < path.size() - 1) {
        size_t farthest = i + 1;
        for (size_t j = i + 2; j < path.size(); ++j) {
            if (isLineCollisionFree(path[i], path[j])) {
                farthest = j;
            } else {
                break;
            }
        }
        smoothed.push_back(path[farthest]);
        i = farthest;
    }
    
    path = smoothed;
    ROS_INFO("[A* Planner]: Path smoothed to %zu waypoints", smoothed.size());
}

void AStarOctomap::visualizePath(const std::vector<Point3D>& path) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "astar_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    for (const auto& p : path) {
        geometry_msgs::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = p.z;
        marker.points.push_back(point);
    }
    
    pathVisPub_.publish(marker);
}

void AStarOctomap::visualizeSearchSpace(const std::unordered_set<Point3D, Point3DHash>& closedSet) {
    visualization_msgs::MarkerArray markerArray;
    
    int id = 0;
    for (const auto& p : closedSet) {
        if (id > 1000) break;  // 限制可视化数量
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "astar_search";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = p.z;
        marker.scale.x = searchResolution_ * 0.8;
        marker.scale.y = searchResolution_ * 0.8;
        marker.scale.z = searchResolution_ * 0.8;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.3;
        marker.lifetime = ros::Duration(5.0);
        
        markerArray.markers.push_back(marker);
    }
    
    searchSpacePub_.publish(markerArray);
}

// 计算三点之间的转角（度）
double AStarOctomap::calculateAngle(const Point3D& p1, const Point3D& p2, const Point3D& p3) const {
    // 向量 p1->p2 和 p2->p3
    double dx1 = p2.x - p1.x;
    double dy1 = p2.y - p1.y;
    double dx2 = p3.x - p2.x;
    double dy2 = p3.y - p2.y;
    
    // 2D方向角度
    double angle1 = std::atan2(dy1, dx1);
    double angle2 = std::atan2(dy2, dx2);
    
    // 计算角度差
    double angleDiff = std::abs(angle2 - angle1) * 180.0 / M_PI;
    if (angleDiff > 180.0) {
        angleDiff = 360.0 - angleDiff;
    }
    
    return angleDiff;
}

// 提取关键拐点（减少航点数量，只保留重要拐点）
std::vector<Point3D> AStarOctomap::extractKeyWaypoints(const std::vector<Point3D>& path) {
    if (path.size() <= 2) {
        return path;  // 路径太短，直接返回
    }
    
    std::vector<Point3D> keyWaypoints;
    keyWaypoints.push_back(path.front());  // 起点总是关键点
    
    size_t lastAddedIdx = 0;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // 计算转角
        double angle = calculateAngle(path[lastAddedIdx], path[i], path[i + 1]);
        
        // 计算距离上一个关键点的距离
        double dist = path[lastAddedIdx].distanceTo(path[i]);
        
        // 判断是否为拐点：角度变化大 或 距离足够远
        bool isTurningPoint = (angle > turningAngleThreshold_);
        bool isDistantEnough = (dist > minWaypointDistance_);
        
        if (isTurningPoint && isDistantEnough) {
            keyWaypoints.push_back(path[i]);
            lastAddedIdx = i;
        }
    }
    
    // 终点总是关键点
    if (path.back().distanceTo(keyWaypoints.back()) > minWaypointDistance_ * 0.5) {
        keyWaypoints.push_back(path.back());
    } else {
        keyWaypoints.back() = path.back();  // 替换最后一个点为真正的终点
    }
    
    double compressionRate = 100.0 * (1.0 - (double)keyWaypoints.size() / path.size());
    ROS_INFO("[A* Planner]: Extracted %zu key waypoints from %zu path points (compression: %.1f%%)", 
             keyWaypoints.size(), path.size(), compressionRate);
    
    return keyWaypoints;
}

// 将航点发布为MarkerArray（供local_planner使用）
void AStarOctomap::publishWaypointsAsMarkerArray(const std::vector<Point3D>& waypoints) {
    visualization_msgs::MarkerArray markerArray;
    
    for (size_t i = 0; i < waypoints.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "global_waypoints";
        marker.id = static_cast<int>(i);  // ID按顺序，local_planner会按ID排序
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
        marker.pose.position.x = waypoints[i].x;
        marker.pose.position.y = waypoints[i].y;
        marker.pose.position.z = waypoints[i].z;
        marker.pose.orientation.w = 1.0;
        
        // 设置航点大小和颜色
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        
        // 渐变色：起点绿色，终点红色
        double t = (waypoints.size() > 1) ? static_cast<double>(i) / (waypoints.size() - 1) : 0.0;
        marker.color.r = t;
        marker.color.g = 1.0 - t;
        marker.color.b = 0.2;
        marker.color.a = 0.9;
        
        marker.lifetime = ros::Duration(0);  // 永久显示
        
        markerArray.markers.push_back(marker);
    }
    
    waypointsPub_.publish(markerArray);
    ROS_INFO("[A* Planner]: Published %zu waypoints to /rrt_planned_path for local_planner", 
             waypoints.size());
}

} // namespace globalPlanner

#endif // ASTAROCTOMAP_H

