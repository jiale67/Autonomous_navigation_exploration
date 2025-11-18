# 🚀 快速开始指南

完整的数据录制和分析流程，3 步搞定！

---

## 📦 准备工作

### 1. 安装依赖

```bash
# 安装 Python 依赖
pip3 install pandas numpy matplotlib scipy

# 如果使用 ROS Python2，安装对应版本
sudo apt-get install python-pandas python-numpy python-matplotlib python-scipy
```

### 2. 进入分析目录

```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis
```

---

## 🎬 完整工作流程

### 方法 A: 使用辅助脚本（推荐）

#### 步骤 1: 录制数据

**终端 1 - 启动实验:**
```bash
cd ~/tare_ws
source devel/setup.bash
roslaunch comparative_experiment cylinder.launch local_planner:=local_planner
```

**终端 2 - 录制数据:**
```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis
./record_experiments.sh cylinder local_planner
```

按提示操作，等待机器人完成任务后按 Ctrl+C 停止录制。

重复上述步骤，分别录制 DWA 和 MPC：
```bash
# DWA
./record_experiments.sh cylinder dwa

# MPC
./record_experiments.sh cylinder mpc
```

#### 步骤 2: 分析数据

```bash
./analyze_all.sh cylinder
```

或手动运行：
```bash
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local_planner.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --output results/cylinder
```

#### 步骤 3: 查看结果

```bash
cd results/cylinder
ls -lh
```

生成的文件：
- `trajectory_comparison.png` - 轨迹对比图
- `velocity_comparison.png` - 速度对比图
- `control_smoothness.png` - 控制平滑度对比
- `performance_summary.png` - 综合性能对比
- `statistics_report.txt` - 统计报告

---

### 方法 B: 手动录制（更灵活）

#### 步骤 1: 录制第一个实验（Local Planner）

**终端 1:**
```bash
cd ~/tare_ws
source devel/setup.bash
roslaunch comparative_experiment cylinder.launch local_planner:=local_planner
```

**终端 2 (等待 10 秒后):**
```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis
mkdir -p bagfiles

rosbag record -O bagfiles/cylinder_local_planner.bag \
    /state_estimation \
    /cmd_vel \
    /odom \
    /local_path \
    /global_planned_path
```

等待机器人到达目标点后，按 Ctrl+C 停止录制。

#### 步骤 2: 录制第二个实验（DWA）

关闭之前的实验，重新启动：

**终端 1:**
```bash
roslaunch comparative_experiment cylinder.launch local_planner:=dwa
```

**终端 2 (等待 10 秒后):**
```bash
rosbag record -O bagfiles/cylinder_dwa.bag \
    /state_estimation \
    /cmd_vel \
    /odom \
    /local_path \
    /global_planned_path
```

#### 步骤 3: 录制第三个实验（MPC）

**终端 1:**
```bash
roslaunch comparative_experiment cylinder.launch local_planner:=mpc
```

**终端 2 (等待 10 秒后):**
```bash
rosbag record -O bagfiles/cylinder_mpc.bag \
    /state_estimation \
    /cmd_vel \
    /odom \
    /local_path \
    /global_planned_path
```

#### 步骤 4: 运行分析

```bash
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --names local_planner dwa mpc \
    --output results/cylinder
```

---

## 🎯 一键命令（测试用）

如果你已经有了 bag 文件，直接分析：

```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis

# 分析单个场景
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local_planner.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --output results/cylinder

# 批量分析所有场景
./analyze_all.sh
```

---

## 📊 验证结果

### 检查 bag 文件

```bash
# 查看文件信息
rosbag info bagfiles/cylinder_local_planner.bag

# 应该看到类似输出:
# path:        bagfiles/cylinder_local_planner.bag
# duration:    45.2s
# topics:      /state_estimation    452 msgs
#              /cmd_vel             450 msgs
```

### 查看生成的图表

```bash
# 使用图片查看器
eog results/cylinder/trajectory_comparison.png

# 或用浏览器打开
firefox results/cylinder/*.png
```

### 查看统计报告

```bash
cat results/cylinder/statistics_report.txt
```

---

## 🔧 常见问题

### Q1: rosbag 命令找不到

```bash
# 确保 source 了 ROS 环境
source /opt/ros/noetic/setup.bash
source ~/tare_ws/devel/setup.bash
```

### Q2: Python 模块找不到

```bash
# 尝试用 pip3 安装
pip3 install rosbag pandas numpy matplotlib scipy

# 或使用 conda
conda install pandas numpy matplotlib scipy
pip install rosbag
```

### Q3: bag 文件没有数据

检查话题是否正确：
```bash
# 先查看当前运行的话题
rostopic list

# 确认话题名称后录制
rosbag record -O test.bag /your_actual_topic_name
```

### Q4: 图表没有生成

检查输出目录权限：
```bash
chmod -R 755 results/
```

---

## 💡 最佳实践

### 1. 数据命名规范

使用统一的命名格式：
```
<场景>_<规划器>.bag
```

示例：
- `cylinder_local_planner.bag`
- `cylinder_dwa.bag`
- `cylinder_mpc.bag`

### 2. 录制时机

- ✅ **开始录制**: 系统启动后 5-10 秒（等待目标点发送）
- ✅ **停止录制**: 机器人到达目标点且稳定后
- ❌ **避免**: 录制系统启动过程或关闭过程

### 3. 数据质量

确保：
- 机器人完成了完整的任务
- bag 文件大小合理（通常 > 1MB）
- 包含必要的话题

### 4. 批量处理

对所有三个场景进行实验：

```bash
# Substation
./record_experiments.sh substation local_planner
./record_experiments.sh substation dwa
./record_experiments.sh substation mpc

# Cylinder
./record_experiments.sh cylinder local_planner
./record_experiments.sh cylinder dwa
./record_experiments.sh cylinder mpc

# Maze
./record_experiments.sh maze local_planner
./record_experiments.sh maze dwa
./record_experiments.sh maze mpc

# 分析所有
./analyze_all.sh
```

---

## 📁 最终文件结构

```
analysis/
├── bagfiles/
│   ├── cylinder_local_planner.bag
│   ├── cylinder_dwa.bag
│   ├── cylinder_mpc.bag
│   ├── substation_local_planner.bag
│   ├── substation_dwa.bag
│   ├── substation_mpc.bag
│   ├── maze_local_planner.bag
│   ├── maze_dwa.bag
│   └── maze_mpc.bag
└── results/
    ├── cylinder/
    │   ├── trajectory_comparison.png
    │   ├── velocity_comparison.png
    │   ├── control_smoothness.png
    │   ├── performance_summary.png
    │   └── statistics_report.txt
    ├── substation/
    └── maze/
```

---

## ✅ 检查清单

实验前：
- [ ] ROS 环境已 source
- [ ] comparative_experiment 包已编译
- [ ] Python 依赖已安装
- [ ] 创建了 bagfiles 和 results 目录

录制时：
- [ ] 等待系统完全启动（~10秒）
- [ ] 目标点已自动发送
- [ ] 机器人开始移动
- [ ] 等待任务完成后停止录制

分析后：
- [ ] 检查生成的 5 个文件
- [ ] 查看统计报告
- [ ] 验证图表质量

---

现在开始你的实验吧！🎉


