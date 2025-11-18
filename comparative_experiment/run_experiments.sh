#!/bin/bash
#
# 自动运行对比实验脚本
# 用于依次运行三个场景下三种局部规划器的性能测试
#
# 使用方法：
#   ./run_experiments.sh [场景名称] [局部规划器]
#
# 参数：
#   场景名称: substation, cylinder, maze, all (默认: all)
#   局部规划器: local_planner, dwa, mpc, all (默认: all)
#
# 示例：
#   ./run_experiments.sh substation all        # 在 substation 场景运行所有规划器
#   ./run_experiments.sh all dwa               # 在所有场景运行 DWA
#   ./run_experiments.sh cylinder mpc          # 在 cylinder 场景运行 MPC
#   ./run_experiments.sh                       # 运行所有组合（9个实验）
#

# 默认参数
SCENARIO=${1:-all}
PLANNER=${2:-all}
GLOBAL_PLANNER=${3:-astar}

# 场景列表
SCENARIOS=("substation" "cylinder" "maze")
# 规划器列表
PLANNERS=("local_planner" "dwa" "mpc")

# 颜色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 帮助信息
function show_help() {
    echo -e "${BLUE}============================================${NC}"
    echo -e "${BLUE}   局部规划器对比实验自动运行脚本${NC}"
    echo -e "${BLUE}============================================${NC}"
    echo ""
    echo "使用方法："
    echo "  $0 [场景] [规划器] [全局规划器]"
    echo ""
    echo "参数："
    echo "  场景:        substation, cylinder, maze, all (默认: all)"
    echo "  规划器:      local_planner, dwa, mpc, all (默认: all)"
    echo "  全局规划器:  astar, rrtstar (默认: astar)"
    echo ""
    echo "示例："
    echo "  $0 substation all         # Substation场景，所有规划器"
    echo "  $0 all dwa                # 所有场景，DWA规划器"
    echo "  $0 cylinder mpc           # Cylinder场景，MPC规划器"
    echo "  $0                        # 所有场景和规划器"
    echo ""
}

# 检查参数
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    show_help
    exit 0
fi

# 运行单个实验
function run_experiment() {
    local scenario=$1
    local planner=$2
    local exp_num=$3
    
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}实验 E${exp_num}: ${scenario} + ${planner}${NC}"
    echo -e "${GREEN}全局规划器: ${GLOBAL_PLANNER}${NC}"
    echo -e "${GREEN}============================================${NC}"
    echo ""
    
    # 构建命令
    local cmd="roslaunch comparative_experiment ${scenario}.launch"
    cmd="${cmd} global_planner:=${GLOBAL_PLANNER}"
    cmd="${cmd} local_planner:=${planner}"
    
    echo -e "${BLUE}执行命令:${NC} ${cmd}"
    echo ""
    echo -e "${YELLOW}提示: 按 Ctrl+C 结束当前实验并继续下一个${NC}"
    echo ""
    
    # 执行实验
    $cmd
    
    local exit_code=$?
    if [ $exit_code -eq 0 ]; then
        echo -e "${GREEN}✓ 实验 E${exp_num} 完成${NC}"
    else
        echo -e "${RED}✗ 实验 E${exp_num} 异常退出 (退出码: ${exit_code})${NC}"
    fi
    echo ""
    
    # 等待用户确认
    echo -e "${YELLOW}按回车键继续下一个实验...${NC}"
    read
}

# 主程序
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}   启动对比实验${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "场景选择:      ${GREEN}${SCENARIO}${NC}"
echo -e "规划器选择:    ${GREEN}${PLANNER}${NC}"
echo -e "全局规划器:    ${GREEN}${GLOBAL_PLANNER}${NC}"
echo ""

# 确定要运行的场景
if [[ "$SCENARIO" == "all" ]]; then
    scenarios_to_run=("${SCENARIOS[@]}")
else
    scenarios_to_run=("$SCENARIO")
fi

# 确定要运行的规划器
if [[ "$PLANNER" == "all" ]]; then
    planners_to_run=("${PLANNERS[@]}")
else
    planners_to_run=("$PLANNER")
fi

# 计算总实验数
total_exp=$((${#scenarios_to_run[@]} * ${#planners_to_run[@]}))
echo -e "总实验数: ${YELLOW}${total_exp}${NC}"
echo ""
echo -e "${YELLOW}3秒后开始实验...${NC}"
sleep 3

# 运行实验
exp_counter=1
for scenario in "${scenarios_to_run[@]}"; do
    for planner in "${planners_to_run[@]}"; do
        run_experiment "$scenario" "$planner" "$exp_counter"
        ((exp_counter++))
    done
done

echo -e "${GREEN}============================================${NC}"
echo -e "${GREEN}   所有实验完成!${NC}"
echo -e "${GREEN}============================================${NC}"
echo ""
echo "实验总结:"
echo "  - 总实验数: ${total_exp}"
echo "  - 场景数: ${#scenarios_to_run[@]}"
echo "  - 规划器数: ${#planners_to_run[@]}"
echo ""
echo "后续步骤:"
echo "  1. 分析记录的 rosbag 数据"
echo "  2. 对比各规划器的性能指标"
echo "  3. 生成实验报告和图表"
echo ""





















