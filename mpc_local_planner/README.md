# MPC Local Planner

基于模型预测控制（Model Predictive Control, MPC）的局部路径规划器，用于自主导航系统。

## 概述

MPC Local Planner 是一个实时局部路径规划器，可以配合 A* 或 RRT* 全局规划器使用。它通过求解有限时域最优控制问题，生成平滑、安全的局部轨迹，实现对全局路径的精确跟踪和动态避障。

## 主要特性

- **模型预测控制**：使用差分驱动运动学模型预测未来轨迹
- **实时优化**：通过梯度下降法实时求解最优控制序列
- **多目标优化**：综合考虑路径跟踪、速度控制、平滑性和避障
- **障碍物规避**：基于点云的实时障碍检测与规避
- **全局路径集成**：无缝对接 A* 和 RRT* 全局规划器输出
- **可配置参数**：丰富的参数接口，便于调优

## 系统架构

### 输入
- `/state_estimation` (nav_msgs/Odometry) - 机器人位姿与速度
- `/terrain_map` 或 `/registered_scan` (sensor_msgs/PointCloud2) - 环境点云
- `/global_planned_path` (visualization_msgs/MarkerArray) - 全局路径关键点

### 输出
- `/cmd_vel` (geometry_msgs/TwistStamped) - 速度控制指令
- `/mpc_local_plan` (nav_msgs/Path) - 当前最优局部路径
- `/mpc_predicted_trajectory` (nav_msgs/Path) - MPC 预测轨迹（可视化）
- `/goal_marker` (visualization_msgs/Marker) - 当前目标点标记

## 算法原理

### MPC 框架

MPC 在每个控制周期求解以下优化问题：

```
min  Σ [ w_x(x_i - x_ref)² + w_y(y_i - y_ref)² + w_θ(θ_i - θ_ref)² 
         + w_v(v_i - v_ref)² + w_ω·ω_i² 
         + w_Δv(v_i - v_{i-1})² + w_Δω(ω_i - ω_{i-1})²
         + w_obs·penalty(d_obs) ]

s.t.  x_{i+1} = x_i + v_i·cos(θ_i)·dt
      y_{i+1} = y_i + v_i·sin(θ_i)·dt
      θ_{i+1} = θ_i + ω_i·dt
      v_min ≤ v_i ≤ v_max
      ω_min ≤ ω_i ≤ ω_max
```

### 核心模块

1. **参考轨迹生成**：从插值后的全局路径提取前视地平线参考点
2. **状态预测**：根据运动学模型前向模拟未来状态
3. **优化求解**：使用梯度下降迭代更新控制序列
4. **约束处理**：应用速度、加速度约束确保可执行性
5. **碰撞检测**：基于 KD-Tree 的快速障碍距离查询

## 参数说明

### 机器人参数
- `robot_radius`: 机器人半径（默认 0.3 m）
- `sensor_offset_x/y`: 传感器相对机器人中心的偏移

### 运动学约束
- `max_speed`: 最大线速度（默认 2.0 m/s）
- `min_speed`: 最小线速度（默认 -0.5 m/s，支持倒车）
- `max_yaw_rate`: 最大角速度（默认 90 deg/s）
- `max_accel`: 最大线加速度（默认 2.5 m/s²）
- `max_yaw_accel`: 最大角加速度（默认 120 deg/s²）

### MPC 配置
- `prediction_horizon`: 预测步数（默认 10）
- `prediction_dt`: 预测时间步长（默认 0.2 s）
- `reference_speed`: 参考速度（默认 1.5 m/s）

### 代价权重
- `weight_x/y`: 位置误差权重（默认 1.0）
- `weight_theta`: 航向误差权重（默认 0.5）
- `weight_v`: 速度跟踪权重（默认 0.3）
- `weight_omega`: 角速度控制权重（默认 0.1）
- `weight_delta_v/omega`: 控制变化平滑权重（默认 0.5/0.3）
- `weight_obstacle`: 障碍物距离权重（默认 2.0）

### 安全参数
- `stop_clearance`: 紧急停车距离（默认 0.5 m）
- `min_obstacle_distance`: 最小安全距离（默认 0.5 m）
- `safety_margin`: 碰撞检测安全裕度（默认 0.1 m）

## 使用方法

### 与 A* 全局规划器配合使用

```bash
roslaunch mpc_local_planner astar_local_mpc.launch world_name:=maze
```

### 与 RRT* 全局规划器配合使用

```bash
roslaunch mpc_local_planner rrtstar_local_mpc.launch world_name:=substation
```

### 支持的场景
- `maze` - 迷宫环境
- `substation` - 变电站场景
- `cylinder` - 圆柱障碍环境
- `corridor` - 走廊场景

### Launch 参数覆盖

```bash
roslaunch mpc_local_planner astar_local_mpc.launch \
  world_name:=maze \
  maxSpeed:=1.5 \
  vehicleX:=-2.0 \
  vehicleY:=-3.0 \
  gazebo_gui:=true
```

## 性能调优

### 提升跟踪精度
- 增加 `weight_x`、`weight_y`、`weight_theta`
- 减小 `prediction_dt`（更细粒度预测）
- 增加 `prediction_horizon`（更长远规划）

### 提升平滑性
- 增加 `weight_delta_v`、`weight_delta_omega`
- 降低 `max_accel`、`max_yaw_accel`

### 提升避障能力
- 增加 `weight_obstacle`
- 增大 `min_obstacle_distance`
- 启用 `enable_obstacle_avoidance`

### 提升实时性
- 减少 `prediction_horizon`
- 增大 `prediction_dt`
- 降低 `control_rate`

## 依赖项

- ROS (Noetic/Melodic)
- Eigen3
- PCL (Point Cloud Library)
- 标准 ROS 包：roscpp, tf, nav_msgs, geometry_msgs, sensor_msgs, visualization_msgs

## 编译

```bash
cd ~/tare_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES="mpc_local_planner"
# 或编译整个工作空间
catkin_make
source devel/setup.bash
```

## 与 DWA 对比

| 特性 | DWA | MPC |
|------|-----|-----|
| 算法类型 | 采样优化 | 模型预测 |
| 预测模型 | 简单前向模拟 | 显式运动学模型 |
| 优化方法 | 穷举评估 | 梯度优化 |
| 轨迹平滑性 | 较好 | 优秀 |
| 计算复杂度 | O(N_samples) | O(N_horizon × iterations) |
| 避障能力 | 强（采样密度决定） | 强（软约束可调） |
| 参数调优 | 相对简单 | 较复杂（权重调优） |

## 调试与可视化

在 RViz 中添加以下 Topic：
- `/mpc_predicted_trajectory` (Path) - 查看 MPC 预测轨迹
- `/mpc_local_plan` (Path) - 查看实际执行路径
- `/goal_marker` (Marker) - 查看当前跟踪目标
- `/global_planned_path` (MarkerArray) - 查看全局路径

查看日志：
```bash
rostopic echo /rosout | grep mpc
```

## 已知限制

1. 当前实现采用简化的梯度下降求解器，未使用专业 QP 求解器（如 OSQP、qpOASES）
2. 障碍物约束为软约束，极端情况可能穿透
3. 未实现动态障碍物预测
4. 参数调优依赖经验，缺少自适应机制

## 未来改进

- [ ] 集成 OSQP/qpOASES 高效 QP 求解器
- [ ] 实现非线性 MPC（NMPC）处理更复杂运动学
- [ ] 添加动态障碍物预测与规避
- [ ] 参数自适应调整机制
- [ ] 支持多模式切换（快速/保守/平衡）
- [ ] 添加轨迹后处理平滑算法

## 许可证

BSD

## 作者

User

## 参考文献

1. Dynamic Window Approach to Mobile Robot Motion Control, Fox et al., 1997
2. Model Predictive Control for Trajectory Tracking of Unmanned Ground Vehicles, Zhang et al., 2019
3. Nonlinear Model Predictive Control for Mobile Robot Navigation, Kanjanawanishkul et al., 2009
