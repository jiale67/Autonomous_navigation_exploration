# navigation_start

这是一个专门用于启动导航和探索功能的ROS功能包。

## 功能说明

本功能包不包含任何源代码，仅包含启动文件（launch files），用于启动各种导航和探索模式。

## Launch文件说明

### 1. exploration_tare.launch
**功能**: 启动TARE探索算法

**描述**: 
- 使用TARE (Topologically-guided Autonomous Robotic Exploration) 算法进行自主探索
- 包含传感器覆盖规划、局部和全局路径规划
- 适用于未知环境的自主探索任务

**使用方法**:
```bash
roslaunch navigation_start exploration_tare.launch
```

### 2. global_navigation.launch
**功能**: 启动全局导航系统

**描述**:
- 启动全局路径规划器
- 包含地形分析和障碍物检测
- 适用于已知地图的全局导航

**使用方法**:
```bash
roslaunch navigation_start global_navigation.launch
```

### 3. local_navigation.launch
**功能**: 启动局部导航系统

**描述**:
- 启动局部路径规划器（local planner）
- 实时避障和轨迹跟踪
- 适用于动态环境中的局部导航

**使用方法**:
```bash
roslaunch navigation_start local_navigation.launch
```

## 依赖关系

本功能包依赖以下ROS包：
- `vehicle_simulator` - 机器人仿真
- `local_planner` - 局部路径规划
- `loam_interface` - LOAM SLAM接口
- `terrain_analysis` - 地形分析
- `terrain_analysis_ext` - 扩展地形分析
- `sensor_scan_generation` - 传感器扫描生成
- `tare_planner` - TARE探索规划器
- `global_path_planner` - 全局路径规划器

## 包结构

```
navigation_start/
├── CMakeLists.txt          # CMake配置文件
├── package.xml             # 包描述文件
├── README.md              # 本文件
└── launch/                # 启动文件目录
    ├── exploration_tare.launch
    ├── global_navigation.launch
    └── local_navigation.launch
```

## 编译

```bash
cd ~/tare_ws
catkin_make
```

## 注意事项

1. 使用前请确保所有依赖包已正确安装和编译
2. 启动launch文件前请确保ROS master已启动（roscore）
3. 不同的launch文件可能需要不同的配置参数，请根据实际情况调整

## 版本信息

- 版本: 1.0.0
- 维护者: RJL
- 许可证: MIT


