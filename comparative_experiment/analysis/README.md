# 数据分析工具使用指南

这是一个独立的数据分析模块，用于对比三种局部规划器（local_planner、DWA、MPC）的性能。

## 📋 功能特性

### 生成的图表

1. **轨迹对比图** (`trajectory_comparison.png`)
   - 三条轨迹叠加在同一图上
   - 不同颜色和线型区分
   - 带方向箭头
   - 显示起点、终点和路径长度

2. **速度对比图** (`velocity_comparison.png`)
   - 线速度时间曲线
   - 角速度时间曲线
   - 对比三种规划器的速度变化

3. **控制平滑度对比** (`control_smoothness.png`)
   - 线加速度变化曲线
   - 角加速度变化曲线
   - 平滑度统计柱状图（越小越平滑）

4. **综合性能对比** (`performance_summary.png`)
   - 路径长度对比
   - 任务完成时间对比
   - 平均速度对比
   - 综合评分

5. **统计报告** (`statistics_report.txt`)
   - 详细的数值统计
   - 各项性能指标

---

## 🚀 使用方法

### 前提条件

1. 已录制 rosbag 数据
2. 安装必要的 Python 库

```bash
pip3 install rosbag pandas numpy matplotlib scipy
```

### 步骤 1: 录制实验数据

在运行实验时，在**独立终端**录制数据：

```bash
# 进入数据目录
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis
mkdir -p bagfiles

# 录制 Local Planner 数据
roslaunch comparative_experiment cylinder.launch local_planner:=local_planner &
sleep 10  # 等待系统启动
rosbag record -O bagfiles/cylinder_local_planner.bag /state_estimation /cmd_vel /odom

# 等待任务完成后 Ctrl+C 停止录制

# 录制 DWA 数据
roslaunch comparative_experiment cylinder.launch local_planner:=dwa &
sleep 10
rosbag record -O bagfiles/cylinder_dwa.bag /state_estimation /cmd_vel /odom

# 录制 MPC 数据
roslaunch comparative_experiment cylinder.launch local_planner:=mpc &
sleep 10
rosbag record -O bagfiles/cylinder_mpc.bag /state_estimation /cmd_vel /odom
```

### 步骤 2: 分析数据

运行分析脚本：

```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis

python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --names local_planner dwa mpc \
    --output results/cylinder
```

### 参数说明

| 参数 | 说明 | 示例 |
|------|------|------|
| `--bags` | rosbag 文件列表（按 local/dwa/mpc 顺序） | `bag1.bag bag2.bag bag3.bag` |
| `--names` | 规划器名称（可选，默认 local_planner dwa mpc） | `local_planner dwa mpc` |
| `--output` | 输出目录 | `results/experiment1` |

---

## 📊 批量分析示例

### 分析所有三个场景

```bash
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis

# Substation 场景
python3 trajectory_analyzer.py \
    --bags bagfiles/substation_local_planner.bag bagfiles/substation_dwa.bag bagfiles/substation_mpc.bag \
    --output results/substation

# Cylinder 场景
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local_planner.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --output results/cylinder

# Maze 场景
python3 trajectory_analyzer.py \
    --bags bagfiles/maze_local_planner.bag bagfiles/maze_dwa.bag bagfiles/maze_mpc.bag \
    --output results/maze
```

---

## 📁 目录结构

```
analysis/
├── README.md                      # 本文件
├── trajectory_analyzer.py         # 主分析脚本
├── bagfiles/                      # 存放 rosbag 数据
│   ├── cylinder_local_planner.bag
│   ├── cylinder_dwa.bag
│   └── cylinder_mpc.bag
└── results/                       # 分析结果
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

## 🎨 图表配置

### 规划器颜色和样式

- **Local Planner**: 绿色实线 (━)
- **DWA**: 蓝色虚线 (- -)
- **MPC**: 红色点划线 (-·-)

### 自定义配置

可以在 `trajectory_analyzer.py` 中修改 `PLANNER_CONFIGS` 字典来自定义颜色和样式。

---

## 💡 使用技巧

### 1. 确保数据质量

- **时间充足**: 确保录制了完整的任务过程（从起点到终点）
- **话题正确**: 检查 bag 文件包含必要的话题
  ```bash
  rosbag info bagfiles/cylinder_local_planner.bag
  ```

### 2. 检查数据内容

```bash
# 查看 bag 文件信息
rosbag info bagfiles/cylinder_local_planner.bag

# 播放数据查看
rosbag play bagfiles/cylinder_local_planner.bag
```

### 3. 只分析两个规划器

如果只想对比两个规划器：

```bash
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local_planner.bag bagfiles/cylinder_dwa.bag \
    --names local_planner dwa \
    --output results/cylinder_2planners
```

### 4. 处理不同话题名称

脚本会自动尝试以下话题：
- 位姿: `/state_estimation`, `/odom`, `/odometry/filtered`
- 控制: `/cmd_vel`

如果你的话题名称不同，需要修改脚本中的话题列表。

---

## ⚠️ 常见问题

### 问题1: 找不到 rosbag 模块

```bash
pip3 install --extra-index-url https://rospypi.github.io/simple/ rosbag
```

或使用系统的 Python2（如果使用 ROS Noetic）：

```bash
python2 trajectory_analyzer.py ...
```

### 问题2: 没有数据点

检查 bag 文件是否包含正确的话题：

```bash
rosbag info your_file.bag
```

### 问题3: 图表中文显示乱码

脚本已配置中文字体，如果仍有问题：

```bash
# 安装中文字体
sudo apt-get install fonts-wqy-microhei fonts-wqy-zenhei
```

---

## 📈 性能指标说明

### 轨迹对比维度

1. **路径长度**: 实际行驶距离，越接近最短路径越好
2. **完成时间**: 任务总耗时，越短越好
3. **平均速度**: 行驶效率指标
4. **控制平滑度**: 加速度变化幅度，越小表示控制越平滑

### 评价 Local Planner 优势

通过以下指标体现 Local Planner 的优越性：

- ✅ **路径平滑度**: 查看轨迹曲线是否更加平滑
- ✅ **控制稳定性**: 加速度变化是否更小
- ✅ **跟踪精度**: 如有全局路径，对比偏差
- ✅ **综合效率**: 路径长度、时间、速度的综合评分

---

## 🔧 高级用法

### 修改评分权重

在 `plot_performance_summary()` 函数中修改评分权重：

```python
# 原始权重
total_score = (path_score * 0.3 + time_score * 0.4 + speed_score * 0.3) * 100

# 示例：更重视路径效率
total_score = (path_score * 0.5 + time_score * 0.3 + speed_score * 0.2) * 100
```

### 添加自定义分析

可以在脚本中添加新的分析函数，例如：

```python
def plot_custom_metric(data_list, output_dir):
    # 你的自定义分析代码
    pass

# 在 main() 中调用
plot_custom_metric(data_list, args.output)
```

---

## 📞 获取帮助

```bash
python3 trajectory_analyzer.py --help
```

---

## ✅ 完整工作流示例

```bash
# 1. 创建数据目录
cd ~/tare_ws/src/autonomous_navigation/src/comparative_experiment/analysis
mkdir -p bagfiles results

# 2. 录制三个实验的数据（在实验运行时）
# （省略录制步骤，见上文）

# 3. 运行分析
python3 trajectory_analyzer.py \
    --bags bagfiles/cylinder_local_planner.bag bagfiles/cylinder_dwa.bag bagfiles/cylinder_mpc.bag \
    --output results/cylinder

# 4. 查看结果
cd results/cylinder
ls -lh  # 查看生成的文件
cat statistics_report.txt  # 查看统计报告
```

生成的图片可以直接用于论文、报告或演示！


