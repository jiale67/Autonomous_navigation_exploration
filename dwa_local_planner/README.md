# DWA Local Planner

基于动态窗口法（Dynamic Window Approach）的局部路径规划器，用于自主导航。

## 功能特性

- **DWA 算法**：通过采样线速度和角速度生成多条候选轨迹，选择最优路径
- **避障功能**：基于点云数据进行碰撞检测和障碍物距离评估
- **全局路径跟随**：订阅 `/global_planned_path` 话题，实现点到点导航
- **多代价函数**：综合考虑朝向、速度、障碍物距离和路径跟随
- **实时可视化**：发布候选轨迹和最优轨迹用于调试

## 话题接口

### 订阅话题
- `/state_estimation` (nav_msgs::Odometry) - 机器人位姿和速度
- `/global_planned_path` (visualization_msgs::MarkerArray) - 全局路径规划结果
- `/terrain_map` 或 `/registered_scan` (sensor_msgs::PointCloud2) - 障碍物点云

### 发布话题
- `/cmd_vel` (geometry_msgs::TwistStamped) - 速度控制指令
- `/dwa_local_plan` (nav_msgs::Path) - 选中的最优轨迹
- `/dwa_candidates` (visualization_msgs::MarkerArray) - 候选轨迹（调试用）
- `/goal_marker` (visualization_msgs::Marker) - 当前跟踪目标点

## 参数说明

主要参数在 `config/params.yaml` 中配置：

### 机器人参数
- `robot_radius`: 机器人半径 (m)
- `max_speed`: 最大线速度 (m/s)
- `max_yaw_rate`: 最大角速度 (度/秒)
- `max_accel`: 最大线加速度 (m/s²)

### DWA 采样参数
- `sim_time`: 前向仿真时间 (秒)
- `sim_dt`: 仿真时间步长 (秒)
- `vx_samples`: 线速度采样数
- `wz_samples`: 角速度采样数

### 代价函数权重
- `heading_cost_weight`: 朝向代价权重
- `velocity_cost_weight`: 速度代价权重
- `clearance_cost_weight`: 障碍物距离代价权重
- `path_distance_cost_weight`: 路径跟随代价权重

## 使用方法

### 编译
```bash
cd ~/tare_ws
catkin_make --pkg dwa_local_planner
source devel/setup.bash
```

### 运行（A* 全局规划）
```bash
roslaunch dwa_local_planner astar_local_dwa.launch
```

可选参数：
- `world_name`: 仿真世界名称（默认：substation）
- `maxSpeed`: 最大速度（默认：2.0）
- `vehicleX`, `vehicleY`: 初始位置

### 运行（RRT* 全局规划）
```bash
roslaunch dwa_local_planner rrtstar_local_dwa.launch
```

可选参数：
- `world_name`: 仿真世界名称（默认：maze）
- `maxSpeed`: 最大速度（默认：2.0）

## 算法流程

1. **数据准备**：接收里程计、点云和全局路径
2. **动态窗口采样**：根据当前速度和加速度约束生成可行速度空间
3. **轨迹仿真**：对每个速度对 (v, w) 进行前向仿真
4. **碰撞检测**：使用 KD-Tree 快速检测轨迹与障碍物碰撞
5. **代价评估**：综合评估朝向、速度、避障和路径跟随代价
6. **最优选择**：选择代价最高的可行轨迹并输出控制指令

## 调试建议

1. 在 RViz 中添加以下显示：
   - `/dwa_local_plan` - 最优轨迹（Path）
   - `/dwa_candidates` - 候选轨迹（MarkerArray）
   - `/goal_marker` - 目标点（Marker）

2. 调整代价函数权重以优化行为：
   - 增大 `clearance_cost_weight` 使机器人更保守避障
   - 增大 `path_distance_cost_weight` 使机器人更紧密跟随全局路径
   - 增大 `velocity_cost_weight` 鼓励更高速度

3. 调整采样参数：
   - 增加 `vx_samples` 和 `wz_samples` 提高精度但降低实时性
   - 调整 `sim_time` 改变规划视野

## 与原 local_planner 的区别

| 特性 | 原 local_planner | dwa_local_planner |
|------|-----------------|-------------------|
| 算法 | 预定义路径选择 | DWA 动态采样 |
| 灵活性 | 依赖预设路径文件 | 实时生成轨迹 |
| 避障 | 基于路径评分 | 基于轨迹仿真 |
| 计算复杂度 | 低 | 中等 |
| 适应性 | 有限 | 更强 |

## 故障排查

- **没有速度输出**：检查是否接收到全局路径和点云数据
- **频繁停车**：降低 `clearance_cost_weight` 或 `stop_clearance`
- **不跟随路径**：增大 `path_distance_cost_weight`
- **速度过慢**：增大 `velocity_cost_weight`

## 作者

自主导航系统 - DWA 局部规划器模块

## 许可

BSD License









