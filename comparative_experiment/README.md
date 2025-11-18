# Comparative Experiment Package

用于对比评估 `local_planner`、`dwa_local_planner` 和 `mpc_local_planner` 三种局部规划器在不同场景下性能的实验包。

## 主要特性

- **自动目标发送**：系统启动后 5 秒自动发送预设目标点，无需手动在 RViz 中点击
- **标准化场景**：三个场景具有固定的初始位姿和目标点，确保对比公平性
- **灵活配置**：支持参数化选择全局规划器（A*/RRT*）和局部规划器
- **控制变量**：所有实验使用相同的初始条件和目标位置
- **三种局部规划器**：支持 Local Planner（基于栅格地图）、DWA（动态窗口法）和 MPC（模型预测控制）

## 三种局部规划器对比

### 1. Local Planner（原始局部规划器）
- **算法原理**：基于栅格地图的路径跟踪
- **特点**：
  - 针对地形分析优化
  - 适合结构化环境
  - 平滑度好
- **适用场景**：走廊、结构化环境

### 2. DWA (Dynamic Window Approach)
- **算法原理**：动态窗口法，基于速度空间采样
- **特点**：
  - 快速响应
  - 考虑动力学约束
  - 实时性好
- **适用场景**：动态环境、需要快速避障

### 3. MPC (Model Predictive Control)
- **算法原理**：模型预测控制，优化未来轨迹
- **特点**：
  - 轨迹平滑
  - 考虑多步预测
  - 优化性能好
- **适用场景**：需要高质量轨迹的场景

### 对比维度

实验应从以下维度对比三种局部规划器的性能：

1. **路径跟踪精度**：与全局路径的偏差
2. **运行效率**：计算时间、控制频率
3. **避障能力**：最小障碍物距离、碰撞次数
4. **轨迹平滑度**：速度/加速度变化率
5. **任务完成时间**：从起点到终点的总时间
6. **能耗效率**：控制输入的平方和
7. **鲁棒性**：在不同场景下的稳定性

## 实验场景

本包提供三个标准化场景，每个场景具有固定的初始位姿和目标点，以确保对比实验的公平性：

### 1. Substation（L型走廊场景）
- **场景描述**：L型走廊环境，考察规划器在结构化环境中的表现
- **初始位姿**：(-5.0, -3.0, yaw=0°)
- **目标点**：(15.0, 10.0)
- **Launch文件**：`substation.launch`

### 2. Cylinder（柱形障碍场景）
- **场景描述**：散布柱形障碍物的开放空间，测试避障能力
- **初始位姿**：(-8.0, 0.0, yaw=0°)
- **目标点**：(8.0, 0.0)
- **Launch文件**：`cylinder.launch`

### 3. Maze（迷宫场景）
- **场景描述**：复杂迷宫环境，考察规划器在复杂拓扑下的性能
- **初始位姿**：(-1.0, -3.0, yaw=0°)
- **目标点**：(2.0, 12.0)
- **Launch文件**：`maze.launch`

## 使用方法

### 基本用法

每个场景的 launch 文件支持以下参数：

```bash
roslaunch comparative_experiment <scenario>.launch \
    global_planner:=<astar|rrtstar> \
    local_planner:=<local_planner|dwa|mpc> \
    maxSpeed:=<速度值>
```

### 参数说明

| 参数 | 可选值 | 默认值 | 说明 |
|------|--------|--------|------|
| `global_planner` | `astar`, `rrtstar` | `astar` | 全局路径规划器 |
| `local_planner` | `local_planner`, `dwa`, `mpc` | `local_planner` | 局部路径规划器 |
| `maxSpeed` | 浮点数 (m/s) | `2.0` | 最大速度 |
| `gazebo_gui` | `true`, `false` | `false` | 是否显示 Gazebo GUI |
| `goalX` | 浮点数 (m) | 场景相关 | 目标点 X 坐标 (自动发送) |
| `goalY` | 浮点数 (m) | 场景相关 | 目标点 Y 坐标 (自动发送) |

### 实验示例

#### 示例 1：Substation 场景 + A* + Local Planner（默认配置）
```bash
roslaunch comparative_experiment substation.launch
```

#### 示例 2：Cylinder 场景 + A* + DWA
```bash
roslaunch comparative_experiment cylinder.launch local_planner:=dwa
```

#### 示例 3：Maze 场景 + RRT* + MPC
```bash
roslaunch comparative_experiment maze.launch \
    global_planner:=rrtstar \
    local_planner:=mpc
```

#### 示例 4：完整参数设置
```bash
roslaunch comparative_experiment substation.launch \
    global_planner:=astar \
    local_planner:=dwa \
    maxSpeed:=1.5 \
    gazebo_gui:=true
```

## 实验流程建议

### 1. 准备阶段
```bash
# 确保环境已编译
cd ~/tare_ws
catkin_make
source devel/setup.bash
```

### 2. 运行实验

对于每个场景，依次运行三种局部规划器：

**Substation 场景：**
```bash
# 实验 1: Local Planner
roslaunch comparative_experiment substation.launch local_planner:=local_planner

# 实验 2: DWA
roslaunch comparative_experiment substation.launch local_planner:=dwa

# 实验 3: MPC
roslaunch comparative_experiment substation.launch local_planner:=mpc
```

**Cylinder 场景：**
```bash
# 实验 4: Local Planner
roslaunch comparative_experiment cylinder.launch local_planner:=local_planner

# 实验 5: DWA
roslaunch comparative_experiment cylinder.launch local_planner:=dwa

# 实验 6: MPC
roslaunch comparative_experiment cylinder.launch local_planner:=mpc
```

**Maze 场景：**
```bash
# 实验 7: Local Planner
roslaunch comparative_experiment maze.launch local_planner:=local_planner

# 实验 8: DWA
roslaunch comparative_experiment maze.launch local_planner:=dwa

# 实验 9: MPC
roslaunch comparative_experiment maze.launch local_planner:=mpc
```

### 3. 数据采集

#### 方法 1：使用 rosbag 录制
```bash
# 在另一个终端录制关键话题
rosbag record -O experiment_substation_local.bag \
    /state_estimation \
    /cmd_vel \
    /global_planned_path \
    /terrain_map \
    /local_path
```

#### 方法 2：实时监控
```bash
# 监控速度指令
rostopic echo /cmd_vel

# 监控位姿
rostopic echo /state_estimation

# 查看话题列表
rostopic list
```

### 4. 设置目标点

在 RViz 中：
1. 点击顶部工具栏的 "2D Nav Goal"
2. 在地图上点击目标位置（建议使用预设目标点以保证一致性）
3. 观察机器人导航过程

**预设目标点（通过命令行发布）：**

```bash
# Substation 场景目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
    "header: {frame_id: 'map'}
     pose: {position: {x: 15.0, y: 10.0, z: 0.0}}"

# Cylinder 场景目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
    "header: {frame_id: 'map'}
     pose: {position: {x: 8.0, y: 0.0, z: 0.0}}"

# Maze 场景目标
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
    "header: {frame_id: 'map'}
     pose: {position: {x: 2.0, y: 12.0, z: 0.0}}"
```

## 数据分析

### 关键评估指标

1. **路径跟踪误差**
   - 话题：`/state_estimation` vs `/global_planned_path`
   - 计算横向和纵向偏差

2. **速度与加速度曲线**
   - 话题：`/cmd_vel`
   - 分析平滑性和稳定性

3. **障碍物安全距离**
   - 话题：`/terrain_map`, `/state_estimation`
   - 计算最近障碍距离

4. **任务完成时间**
   - 从启动到到达目标的总时间

5. **控制频率**
   - 话题发布频率统计

### Python 数据处理脚本示例

```python
import rosbag
import pandas as pd
import matplotlib.pyplot as plt

# 读取 bag 文件
bag = rosbag.Bag('experiment_substation_local.bag')

# 提取速度数据
velocities = []
timestamps = []
for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
    timestamps.append(t.to_sec())
    velocities.append(msg.twist.linear.x)

# 绘图
plt.figure(figsize=(10, 6))
plt.plot(timestamps, velocities, label='Linear Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile - Local Planner - Substation')
plt.legend()
plt.grid(True)
plt.savefig('velocity_profile.png', dpi=300)
plt.show()

bag.close()
```

## 实验矩阵

完整的对比实验矩阵（9个实验）：

| 场景 | 全局规划器 | 局部规划器 | 实验编号 | 命令 |
|------|-----------|-----------|---------|------|
| Substation | A* | Local Planner | E1 | `roslaunch comparative_experiment substation.launch local_planner:=local_planner` |
| Substation | A* | DWA | E2 | `roslaunch comparative_experiment substation.launch local_planner:=dwa` |
| Substation | A* | MPC | E3 | `roslaunch comparative_experiment substation.launch local_planner:=mpc` |
| Cylinder | A* | Local Planner | E4 | `roslaunch comparative_experiment cylinder.launch local_planner:=local_planner` |
| Cylinder | A* | DWA | E5 | `roslaunch comparative_experiment cylinder.launch local_planner:=dwa` |
| Cylinder | A* | MPC | E6 | `roslaunch comparative_experiment cylinder.launch local_planner:=mpc` |
| Maze | A* | Local Planner | E7 | `roslaunch comparative_experiment maze.launch local_planner:=local_planner` |
| Maze | A* | DWA | E8 | `roslaunch comparative_experiment maze.launch local_planner:=dwa` |
| Maze | A* | MPC | E9 | `roslaunch comparative_experiment maze.launch local_planner:=mpc` |

**扩展实验矩阵**（可选，使用 RRT* 全局规划器）：

| 场景 | 全局规划器 | 局部规划器 | 实验编号 | 命令 |
|------|-----------|-----------|---------|------|
| Substation | RRT* | Local Planner | E10 | `roslaunch comparative_experiment substation.launch global_planner:=rrtstar local_planner:=local_planner` |
| Substation | RRT* | DWA | E11 | `roslaunch comparative_experiment substation.launch global_planner:=rrtstar local_planner:=dwa` |
| Substation | RRT* | MPC | E12 | `roslaunch comparative_experiment substation.launch global_planner:=rrtstar local_planner:=mpc` |
| Cylinder | RRT* | Local Planner | E13 | `roslaunch comparative_experiment cylinder.launch global_planner:=rrtstar local_planner:=local_planner` |
| Cylinder | RRT* | DWA | E14 | `roslaunch comparative_experiment cylinder.launch global_planner:=rrtstar local_planner:=dwa` |
| Cylinder | RRT* | MPC | E15 | `roslaunch comparative_experiment cylinder.launch global_planner:=rrtstar local_planner:=mpc` |
| Maze | RRT* | Local Planner | E16 | `roslaunch comparative_experiment maze.launch global_planner:=rrtstar local_planner:=local_planner` |
| Maze | RRT* | DWA | E17 | `roslaunch comparative_experiment maze.launch global_planner:=rrtstar local_planner:=dwa` |
| Maze | RRT* | MPC | E18 | `roslaunch comparative_experiment maze.launch global_planner:=rrtstar local_planner:=mpc` |

## 注意事项

1. **控制变量**：
   - 每次实验使用相同的初始位姿和目标点
   - 保持 `maxSpeed` 等参数一致
   - 确保传感器配置相同

2. **重复实验**：
   - 建议每个配置重复 3-5 次
   - 计算平均值和标准差

3. **环境一致性**：
   - 确保仿真环境未被修改
   - 关闭不必要的后台进程

4. **数据同步**：
   - 使用 rosbag 时注意时间戳对齐
   - 记录实验开始和结束的准确时间

## 故障排查

### 问题：规划器未启动
**解决**：检查包是否编译成功，确认参数拼写正确

### 问题：机器人不移动
**解决**：
1. 确认已发送目标点
2. 检查 `/cmd_vel` 话题是否有数据
3. 查看规划器日志输出

### 问题：找不到包
**解决**：
```bash
source ~/tare_ws/devel/setup.bash
rospack find comparative_experiment
```

## 作者

自主导航系统对比实验包

## 许可

BSD License

