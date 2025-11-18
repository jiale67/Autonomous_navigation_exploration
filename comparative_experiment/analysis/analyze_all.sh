#!/bin/bash
#
# 批量分析所有场景的实验数据
# 使用方法: ./analyze_all.sh [场景名称]
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 颜色定义
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 场景列表
SCENARIOS=("substation" "cylinder" "maze")

# 检查参数
if [ $# -gt 0 ]; then
    SCENARIOS=("$1")
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  批量轨迹与性能分析${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 检查 Python 脚本
if [ ! -f "trajectory_analyzer.py" ]; then
    echo -e "${RED}错误: 找不到 trajectory_analyzer.py${NC}"
    exit 1
fi

# 创建输出目录
mkdir -p bagfiles results

# 分析每个场景
for scenario in "${SCENARIOS[@]}"; do
    echo -e "${GREEN}----------------------------------------${NC}"
    echo -e "${GREEN}分析场景: $scenario${NC}"
    echo -e "${GREEN}----------------------------------------${NC}"
    echo ""
    
    # 检查 bag 文件是否存在
    local_bag="bagfiles/${scenario}_local_planner.bag"
    dwa_bag="bagfiles/${scenario}_dwa.bag"
    mpc_bag="bagfiles/${scenario}_mpc.bag"
    
    if [ ! -f "$local_bag" ]; then
        echo -e "${YELLOW}警告: 找不到 $local_bag${NC}"
    fi
    if [ ! -f "$dwa_bag" ]; then
        echo -e "${YELLOW}警告: 找不到 $dwa_bag${NC}"
    fi
    if [ ! -f "$mpc_bag" ]; then
        echo -e "${YELLOW}警告: 找不到 $mpc_bag${NC}"
    fi
    
    # 检查至少有两个 bag 文件
    bag_count=0
    bags=()
    names=()
    
    if [ -f "$local_bag" ]; then
        bags+=("$local_bag")
        names+=("local_planner")
        ((bag_count++))
    fi
    if [ -f "$dwa_bag" ]; then
        bags+=("$dwa_bag")
        names+=("dwa")
        ((bag_count++))
    fi
    if [ -f "$mpc_bag" ]; then
        bags+=("$mpc_bag")
        names+=("mpc")
        ((bag_count++))
    fi
    
    if [ $bag_count -lt 2 ]; then
        echo -e "${RED}错误: 场景 $scenario 至少需要 2 个 bag 文件${NC}"
        echo ""
        continue
    fi
    
    # 运行分析
    echo -e "${BLUE}正在分析 $scenario (共 $bag_count 个数据集)...${NC}"
    
    python3 trajectory_analyzer.py \
        --bags "${bags[@]}" \
        --names "${names[@]}" \
        --output "results/$scenario"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ $scenario 分析完成${NC}"
    else
        echo -e "${RED}✗ $scenario 分析失败${NC}"
    fi
    
    echo ""
done

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  所有分析完成！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "结果保存在: $SCRIPT_DIR/results/"
echo ""
echo "查看结果:"
for scenario in "${SCENARIOS[@]}"; do
    if [ -d "results/$scenario" ]; then
        echo "  - results/$scenario/"
    fi
done
echo ""


