#!/bin/bash
#
# 辅助录制实验数据脚本
# 使用方法: ./record_experiments.sh <场景名称> <规划器名称>
#
# 示例:
#   ./record_experiments.sh cylinder local_planner
#   ./record_experiments.sh substation dwa
#   ./record_experiments.sh maze mpc
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 检查参数
if [ $# -lt 2 ]; then
    echo -e "${RED}用法: $0 <场景> <规划器>${NC}"
    echo ""
    echo "场景: substation, cylinder, maze"
    echo "规划器: local_planner, dwa, mpc"
    echo ""
    echo "示例:"
    echo "  $0 cylinder local_planner"
    echo "  $0 substation dwa"
    echo "  $0 maze mpc"
    echo ""
    exit 1
fi

SCENARIO=$1
PLANNER=$2

# 验证参数
if [[ ! "$SCENARIO" =~ ^(substation|cylinder|maze)$ ]]; then
    echo -e "${RED}错误: 场景必须是 substation, cylinder 或 maze${NC}"
    exit 1
fi

if [[ ! "$PLANNER" =~ ^(local_planner|dwa|mpc)$ ]]; then
    echo -e "${RED}错误: 规划器必须是 local_planner, dwa 或 mpc${NC}"
    exit 1
fi

# 创建 bagfiles 目录
mkdir -p bagfiles

# 输出文件名
OUTPUT_BAG="bagfiles/${SCENARIO}_${PLANNER}.bag"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  实验数据录制助手${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "场景:     ${GREEN}$SCENARIO${NC}"
echo -e "规划器:   ${GREEN}$PLANNER${NC}"
echo -e "输出文件: ${GREEN}$OUTPUT_BAG${NC}"
echo ""

# 检查文件是否已存在
if [ -f "$OUTPUT_BAG" ]; then
    echo -e "${YELLOW}警告: 文件 $OUTPUT_BAG 已存在${NC}"
    read -p "是否覆盖? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "已取消"
        exit 0
    fi
    rm "$OUTPUT_BAG"
fi

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  准备录制${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo "步骤:"
echo "  1. 在另一个终端启动实验:"
echo -e "     ${BLUE}roslaunch comparative_experiment ${SCENARIO}.launch local_planner:=${PLANNER}${NC}"
echo ""
echo "  2. 等待系统完全启动 (约 10 秒)"
echo ""
echo "  3. 回到本终端按任意键开始录制"
echo ""
read -p "按任意键开始录制..." -n 1 -r
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  开始录制...${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}提示: 按 Ctrl+C 停止录制${NC}"
echo ""

# 录制数据
rosbag record \
    -O "$OUTPUT_BAG" \
    /state_estimation \
    /cmd_vel \
    /odom \
    /local_path \
    /global_planned_path \
    /move_base_simple/goal

# 录制完成
echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  录制完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 显示 bag 信息
if [ -f "$OUTPUT_BAG" ]; then
    echo "文件信息:"
    rosbag info "$OUTPUT_BAG"
    echo ""
    echo -e "${GREEN}✓ 数据已保存到: $OUTPUT_BAG${NC}"
    
    # 检查是否收集了所有三个规划器的数据
    echo ""
    echo "当前场景 ($SCENARIO) 的数据收集进度:"
    count=0
    for p in local_planner dwa mpc; do
        bag_file="bagfiles/${SCENARIO}_${p}.bag"
        if [ -f "$bag_file" ]; then
            echo -e "  ✓ ${GREEN}$p${NC}"
            ((count++))
        else
            echo -e "  ✗ ${RED}$p (未录制)${NC}"
        fi
    done
    
    echo ""
    if [ $count -eq 3 ]; then
        echo -e "${GREEN}✓ 所有三个规划器的数据已收集完成！${NC}"
        echo ""
        echo "可以运行分析:"
        echo -e "  ${BLUE}cd $SCRIPT_DIR${NC}"
        echo -e "  ${BLUE}python3 trajectory_analyzer.py \\${NC}"
        echo -e "    ${BLUE}--bags bagfiles/${SCENARIO}_local.bag bagfiles/${SCENARIO}_dwa.bag bagfiles/${SCENARIO}_mpc.bag \\${NC}"
        echo -e "    ${BLUE}--output results/${SCENARIO}${NC}"
    else
        echo -e "${YELLOW}还需要录制 $((3-count)) 个规划器的数据${NC}"
    fi
else
    echo -e "${RED}错误: 录制失败${NC}"
fi

echo ""





















