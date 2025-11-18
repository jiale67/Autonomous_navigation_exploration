#!/bin/bash
# A* 路径规划器测试脚本

echo "========================================="
echo "  A* 全局路径规划器 - 快速测试"
echo "========================================="

# 检查是否source了workspace
if [ -z "$ROS_PACKAGE_PATH" ]; then
    echo "❌ 错误: ROS环境未配置"
    echo "请运行: source ~/tare_ws/devel/setup.bash"
    exit 1
fi

# 检查A*节点是否存在
if ! rospack find global_path_planner > /dev/null 2>&1; then
    echo "❌ 错误: 找不到global_path_planner包"
    exit 1
fi

ASTAR_NODE=$(rospack find global_path_planner)/../../devel/lib/global_path_planner/astar_interactive_node

if [ ! -f "$ASTAR_NODE" ]; then
    echo "❌ 错误: A*节点未编译"
    echo "请运行: catkin_make --pkg global_path_planner"
    exit 1
fi

echo "✅ 检查通过: A*节点已编译"
echo ""
echo "测试步骤:"
echo "1. 确保有Octomap地图发布到 /octomap_binary"
echo "2. 确保有里程计数据发布到 /state_estimation"
echo "3. 在RViz中使用 '2D Nav Goal' 设置目标点"
echo ""
echo "启动A*规划器..."
echo ""

# 启动A*规划器
roslaunch global_path_planner astar_interactive.launch


