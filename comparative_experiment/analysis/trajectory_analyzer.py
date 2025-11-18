#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
轨迹与性能分析工具
用于对比 local_planner, dwa, mpc 三种局部规划器的性能

使用方法：
    python3 trajectory_analyzer.py --bags local.bag dwa.bag mpc.bag --output results/
"""

import rosbag
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import rcParams
import argparse
import os
from collections import defaultdict
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d

# 设置中文字体
import matplotlib
matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
matplotlib.rcParams['font.size'] = 10

# 清除字体缓存以确保新设置生效
cache_dir = os.path.join(matplotlib.get_cachedir(), 'fontlist*.json')
for cache_file in matplotlib.get_cachedir().split():
    try:
        if 'fontlist' in cache_file and cache_file.endswith('.json'):
            os.remove(cache_file)
    except:
        pass

# 规划器配置
PLANNER_CONFIGS = {
    'local_planner': {'color': '#2E7D32', 'linestyle': '-', 'marker': 'o', 'label': 'Local Planner'},
    'dwa': {'color': '#1976D2', 'linestyle': '--', 'marker': 's', 'label': 'DWA'},
    'mpc': {'color': '#D32F2F', 'linestyle': '-.', 'marker': '^', 'label': 'MPC'}
}


class TrajectoryData:
    """存储单个规划器的轨迹数据"""
    
    def __init__(self, name):
        self.name = name
        self.timestamps = []
        self.positions = []  # [(x, y, z)]
        self.velocities = []  # [(vx, vy, vz, omega_z)]
        self.cmd_vel = []    # [(linear_x, linear_y, angular_z)]
        self.global_path = None
        
    def add_pose(self, timestamp, x, y, z=0):
        """添加位姿数据"""
        self.timestamps.append(timestamp)
        self.positions.append((x, y, z))
        
    def add_velocity(self, timestamp, vx, vy, vz, omega_z):
        """添加速度数据"""
        self.velocities.append((timestamp, vx, vy, vz, omega_z))
        
    def add_cmd_vel(self, timestamp, linear_x, linear_y, angular_z):
        """添加控制命令"""
        self.cmd_vel.append((timestamp, linear_x, linear_y, angular_z))
        
    def to_dataframe(self):
        """转换为 DataFrame"""
        df = pd.DataFrame({
            'timestamp': self.timestamps,
            'x': [p[0] for p in self.positions],
            'y': [p[1] for p in self.positions],
            'z': [p[2] for p in self.positions]
        })
        df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
        df.set_index('timestamp', inplace=True)
        return df
    
    def get_path_length(self):
        """计算路径总长度"""
        length = 0
        for i in range(1, len(self.positions)):
            dx = self.positions[i][0] - self.positions[i-1][0]
            dy = self.positions[i][1] - self.positions[i-1][1]
            length += np.sqrt(dx**2 + dy**2)
        return length


def extract_data_from_bag(bag_file, planner_name):
    """从 rosbag 提取数据"""
    
    print(f"正在读取 {bag_file}...")
    data = TrajectoryData(planner_name)
    
    try:
        bag = rosbag.Bag(bag_file)
        
        # 读取位姿数据 (state_estimation 或 odom)
        pose_topics = ['/state_estimation', '/odom', '/odometry/filtered']
        for topic in pose_topics:
            try:
                for topic_name, msg, t in bag.read_messages(topics=[topic]):
                    timestamp = t.to_sec()
                    try:
                        # PoseStamped
                        x = msg.pose.pose.position.x
                        y = msg.pose.pose.position.y
                        z = msg.pose.pose.position.z
                    except AttributeError:
                        # Odometry
                        try:
                            x = msg.pose.position.x
                            y = msg.pose.position.y
                            z = msg.pose.position.z
                        except:
                            continue
                    data.add_pose(timestamp, x, y, z)
                if len(data.timestamps) > 0:
                    print(f"  - 从 {topic} 读取了 {len(data.timestamps)} 个位姿点")
                    break
            except:
                continue
        
        # 读取控制命令
        for topic, msg, t in bag.read_messages(topics=['/cmd_vel']):
            timestamp = t.to_sec()
            try:
                linear_x = msg.linear.x if hasattr(msg, 'linear') else msg.twist.linear.x
                linear_y = msg.linear.y if hasattr(msg, 'linear') else msg.twist.linear.y
                angular_z = msg.angular.z if hasattr(msg, 'angular') else msg.twist.angular.z
            except:
                continue
            data.add_cmd_vel(timestamp, linear_x, linear_y, angular_z)
        
        print(f"  - 读取了 {len(data.cmd_vel)} 个控制命令")
        
        bag.close()
        
    except Exception as e:
        print(f"错误: 无法读取 {bag_file}: {e}")
        return None
    
    if len(data.timestamps) == 0:
        print(f"警告: {bag_file} 中没有找到位姿数据")
        return None
    
    return data


def align_time_series(data_list, resample_rate='100ms'):
    """对齐多个轨迹的时间序列"""
    
    dfs = []
    for data in data_list:
        df = data.to_dataframe()
        df = df.resample(resample_rate).mean().interpolate(method='linear')
        dfs.append(df)
    
    # 找到公共时间范围
    start_time = max([df.index.min() for df in dfs])
    end_time = min([df.index.max() for df in dfs])
    
    aligned_dfs = []
    for df in dfs:
        aligned_df = df.loc[start_time:end_time]
        aligned_dfs.append(aligned_df)
    
    return aligned_dfs


def calculate_velocity_and_acceleration(df, smooth_window=5):
    """计算速度和加速度"""
    
    dt = 0.1  # 假设重采样率为 100ms
    
    # 计算速度
    df['vx'] = np.gradient(df['x'], dt)
    df['vy'] = np.gradient(df['y'], dt)
    df['speed'] = np.sqrt(df['vx']**2 + df['vy']**2)
    
    # 平滑处理
    if smooth_window > 0:
        df['speed'] = gaussian_filter1d(df['speed'], sigma=smooth_window)
    
    # 计算加速度
    df['ax'] = np.gradient(df['vx'], dt)
    df['ay'] = np.gradient(df['vy'], dt)
    df['acceleration'] = np.sqrt(df['ax']**2 + df['ay']**2)
    
    if smooth_window > 0:
        df['acceleration'] = gaussian_filter1d(df['acceleration'], sigma=smooth_window)
    
    return df


def plot_trajectories(data_list, output_dir):
    """绘制轨迹对比图（无底图）"""
    
    plt.figure(figsize=(12, 10))
    
    for data in data_list:
        config = PLANNER_CONFIGS.get(data.name, {'color': 'gray', 'linestyle': '-', 'label': data.name})
        positions = np.array(data.positions)
        
        plt.plot(positions[:, 0], positions[:, 1], 
                color=config['color'],
                linestyle=config['linestyle'],
                linewidth=2.5,
                label=f"{config['label']} (长度: {data.get_path_length():.2f}m)",
                alpha=0.8)
        
        # 标记起点和终点
        plt.plot(positions[0, 0], positions[0, 1], 'go', markersize=12, 
                label='起点' if data == data_list[0] else '', zorder=10)
        plt.plot(positions[-1, 0], positions[-1, 1], 'r*', markersize=15, 
                label='终点' if data == data_list[0] else '', zorder=10)
        
        # 每隔一定距离添加方向箭头
        step = max(1, len(positions) // 10)
        for i in range(0, len(positions)-1, step):
            dx = positions[i+1, 0] - positions[i, 0]
            dy = positions[i+1, 1] - positions[i, 1]
            plt.arrow(positions[i, 0], positions[i, 1], dx*0.3, dy*0.3,
                     head_width=0.2, head_length=0.15, fc=config['color'], 
                     ec=config['color'], alpha=0.5, width=0.05)
    
    plt.xlabel('X (m)', fontsize=14, fontweight='bold')
    plt.ylabel('Y (m)', fontsize=14, fontweight='bold')
    plt.title('三种局部规划器轨迹对比', fontsize=16, fontweight='bold')
    plt.legend(fontsize=11, loc='best', framealpha=0.9)
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.axis('equal')
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, 'trajectory_comparison.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ 轨迹对比图已保存: {output_file}")
    plt.close()


def plot_speed_comparison(data_list, output_dir):
    """绘制速度对比图"""
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    for data in data_list:
        config = PLANNER_CONFIGS.get(data.name, {'color': 'gray', 'linestyle': '-', 'label': data.name})
        
        if len(data.cmd_vel) == 0:
            continue
        
        cmd_data = np.array(data.cmd_vel)
        times = cmd_data[:, 0] - cmd_data[0, 0]  # 相对时间
        linear_vel = cmd_data[:, 1]
        angular_vel = cmd_data[:, 3]
        
        # 线速度
        ax1.plot(times, linear_vel, 
                color=config['color'],
                linestyle=config['linestyle'],
                linewidth=2,
                label=config['label'],
                alpha=0.8)
        
        # 角速度
        ax2.plot(times, angular_vel,
                color=config['color'],
                linestyle=config['linestyle'],
                linewidth=2,
                label=config['label'],
                alpha=0.8)
    
    # 线速度图
    ax1.set_xlabel('时间 (s)', fontsize=12, fontweight='bold')
    ax1.set_ylabel('线速度 (m/s)', fontsize=12, fontweight='bold')
    ax1.set_title('线速度对比', fontsize=14, fontweight='bold')
    ax1.legend(fontsize=10)
    ax1.grid(True, alpha=0.3, linestyle='--')
    
    # 角速度图
    ax2.set_xlabel('时间 (s)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('角速度 (rad/s)', fontsize=12, fontweight='bold')
    ax2.set_title('角速度对比', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3, linestyle='--')
    
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, 'velocity_comparison.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ 速度对比图已保存: {output_file}")
    plt.close()


def plot_control_smoothness(data_list, output_dir):
    """绘制控制平滑度对比"""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    
    stats = []
    
    for data in data_list:
        config = PLANNER_CONFIGS.get(data.name, {'color': 'gray', 'linestyle': '-', 'label': data.name})
        
        if len(data.cmd_vel) == 0:
            continue
        
        cmd_data = np.array(data.cmd_vel)
        times = cmd_data[:, 0] - cmd_data[0, 0]
        linear_vel = cmd_data[:, 1]
        angular_vel = cmd_data[:, 3]
        
        # 计算加速度（速度变化率）
        dt = np.diff(times)
        dt[dt == 0] = 0.001  # 避免除零
        linear_acc = np.diff(linear_vel) / dt
        angular_acc = np.diff(angular_vel) / dt
        
        # 绘制加速度
        axes[0, 0].plot(times[1:], linear_acc,
                       color=config['color'],
                       linestyle=config['linestyle'],
                       linewidth=1.5,
                       label=config['label'],
                       alpha=0.8)
        
        axes[0, 1].plot(times[1:], angular_acc,
                       color=config['color'],
                       linestyle=config['linestyle'],
                       linewidth=1.5,
                       label=config['label'],
                       alpha=0.8)
        
        # 计算统计指标
        linear_jerk = np.abs(linear_acc).mean()
        angular_jerk = np.abs(angular_acc).mean()
        
        stats.append({
            'planner': config['label'],
            'linear_jerk': linear_jerk,
            'angular_jerk': angular_jerk,
            'color': config['color']
        })
    
    # 加速度图
    axes[0, 0].set_xlabel('时间 (s)', fontsize=11, fontweight='bold')
    axes[0, 0].set_ylabel('线加速度 (m/s²)', fontsize=11, fontweight='bold')
    axes[0, 0].set_title('线加速度变化', fontsize=13, fontweight='bold')
    axes[0, 0].legend(fontsize=9)
    axes[0, 0].grid(True, alpha=0.3, linestyle='--')
    
    axes[0, 1].set_xlabel('时间 (s)', fontsize=11, fontweight='bold')
    axes[0, 1].set_ylabel('角加速度 (rad/s²)', fontsize=11, fontweight='bold')
    axes[0, 1].set_title('角加速度变化', fontsize=13, fontweight='bold')
    axes[0, 1].legend(fontsize=9)
    axes[0, 1].grid(True, alpha=0.3, linestyle='--')
    
    # 平滑度统计柱状图
    planners = [s['planner'] for s in stats]
    linear_jerks = [s['linear_jerk'] for s in stats]
    angular_jerks = [s['angular_jerk'] for s in stats]
    colors = [s['color'] for s in stats]
    
    x = np.arange(len(planners))
    width = 0.35
    
    axes[1, 0].bar(x, linear_jerks, width, color=colors, alpha=0.7)
    axes[1, 0].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[1, 0].set_ylabel('平均线加速度幅值 (m/s²)', fontsize=11, fontweight='bold')
    axes[1, 0].set_title('线性控制平滑度（越小越平滑）', fontsize=13, fontweight='bold')
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(planners)
    axes[1, 0].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    axes[1, 1].bar(x, angular_jerks, width, color=colors, alpha=0.7)
    axes[1, 1].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[1, 1].set_ylabel('平均角加速度幅值 (rad/s²)', fontsize=11, fontweight='bold')
    axes[1, 1].set_title('角度控制平滑度（越小越平滑）', fontsize=13, fontweight='bold')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(planners)
    axes[1, 1].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, 'control_smoothness.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ 控制平滑度对比图已保存: {output_file}")
    plt.close()


def plot_performance_summary(data_list, output_dir):
    """绘制综合性能对比"""
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    
    metrics = []
    
    for data in data_list:
        config = PLANNER_CONFIGS.get(data.name, {'color': 'gray', 'label': data.name})
        
        # 计算各项指标
        path_length = data.get_path_length()
        
        if len(data.cmd_vel) > 0:
            cmd_data = np.array(data.cmd_vel)
            times = cmd_data[:, 0]
            completion_time = times[-1] - times[0]
            linear_vel = cmd_data[:, 1]
            avg_speed = np.mean(np.abs(linear_vel))
            max_speed = np.max(np.abs(linear_vel))
        else:
            completion_time = 0
            avg_speed = 0
            max_speed = 0
        
        metrics.append({
            'planner': config['label'],
            'path_length': path_length,
            'completion_time': completion_time,
            'avg_speed': avg_speed,
            'max_speed': max_speed,
            'color': config['color']
        })
    
    planners = [m['planner'] for m in metrics]
    colors = [m['color'] for m in metrics]
    x = np.arange(len(planners))
    
    # 路径长度
    axes[0, 0].bar(x, [m['path_length'] for m in metrics], color=colors, alpha=0.7)
    axes[0, 0].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[0, 0].set_ylabel('路径长度 (m)', fontsize=11, fontweight='bold')
    axes[0, 0].set_title('实际行驶路径长度', fontsize=13, fontweight='bold')
    axes[0, 0].set_xticks(x)
    axes[0, 0].set_xticklabels(planners)
    axes[0, 0].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    # 完成时间
    axes[0, 1].bar(x, [m['completion_time'] for m in metrics], color=colors, alpha=0.7)
    axes[0, 1].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[0, 1].set_ylabel('任务完成时间 (s)', fontsize=11, fontweight='bold')
    axes[0, 1].set_title('任务完成时间（越短越好）', fontsize=13, fontweight='bold')
    axes[0, 1].set_xticks(x)
    axes[0, 1].set_xticklabels(planners)
    axes[0, 1].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    # 平均速度
    axes[1, 0].bar(x, [m['avg_speed'] for m in metrics], color=colors, alpha=0.7)
    axes[1, 0].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[1, 0].set_ylabel('平均速度 (m/s)', fontsize=11, fontweight='bold')
    axes[1, 0].set_title('平均行驶速度', fontsize=13, fontweight='bold')
    axes[1, 0].set_xticks(x)
    axes[1, 0].set_xticklabels(planners)
    axes[1, 0].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    # 综合评分（示例：归一化后的加权评分）
    # 评分 = 路径效率 + 时间效率 + 速度稳定性
    max_path = max([m['path_length'] for m in metrics])
    max_time = max([m['completion_time'] for m in metrics]) 
    max_speed = max([m['avg_speed'] for m in metrics])
    
    scores = []
    for m in metrics:
        # 路径越短越好，时间越短越好，速度适中
        path_score = 1.0 - (m['path_length'] / max_path if max_path > 0 else 0)
        time_score = 1.0 - (m['completion_time'] / max_time if max_time > 0 else 0)
        speed_score = m['avg_speed'] / max_speed if max_speed > 0 else 0
        
        total_score = (path_score * 0.3 + time_score * 0.4 + speed_score * 0.3) * 100
        scores.append(total_score)
    
    axes[1, 1].bar(x, scores, color=colors, alpha=0.7)
    axes[1, 1].set_xlabel('规划器', fontsize=11, fontweight='bold')
    axes[1, 1].set_ylabel('综合评分', fontsize=11, fontweight='bold')
    axes[1, 1].set_title('综合性能评分（越高越好）', fontsize=13, fontweight='bold')
    axes[1, 1].set_xticks(x)
    axes[1, 1].set_xticklabels(planners)
    axes[1, 1].set_ylim([0, 100])
    axes[1, 1].grid(True, alpha=0.3, linestyle='--', axis='y')
    
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, 'performance_summary.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"✓ 综合性能对比图已保存: {output_file}")
    plt.close()


def generate_statistics_report(data_list, output_dir):
    """生成统计报告"""
    
    report = []
    report.append("=" * 80)
    report.append("局部规划器性能对比统计报告")
    report.append("=" * 80)
    report.append("")
    
    for data in data_list:
        config = PLANNER_CONFIGS.get(data.name, {'label': data.name})
        report.append(f"【{config['label']}】")
        report.append("-" * 40)
        
        # 路径统计
        path_length = data.get_path_length()
        num_points = len(data.positions)
        report.append(f"  路径长度: {path_length:.3f} m")
        report.append(f"  轨迹点数: {num_points}")
        
        # 速度统计
        if len(data.cmd_vel) > 0:
            cmd_data = np.array(data.cmd_vel)
            times = cmd_data[:, 0]
            linear_vel = cmd_data[:, 1]
            angular_vel = cmd_data[:, 3]
            
            completion_time = times[-1] - times[0]
            avg_linear = np.mean(np.abs(linear_vel))
            max_linear = np.max(np.abs(linear_vel))
            std_linear = np.std(linear_vel)
            avg_angular = np.mean(np.abs(angular_vel))
            max_angular = np.max(np.abs(angular_vel))
            
            report.append(f"  完成时间: {completion_time:.3f} s")
            report.append(f"  平均线速度: {avg_linear:.3f} m/s")
            report.append(f"  最大线速度: {max_linear:.3f} m/s")
            report.append(f"  线速度标准差: {std_linear:.3f} m/s")
            report.append(f"  平均角速度: {avg_angular:.3f} rad/s")
            report.append(f"  最大角速度: {max_angular:.3f} rad/s")
        
        report.append("")
    
    report.append("=" * 80)
    
    # 保存报告
    report_text = "\n".join(report)
    output_file = os.path.join(output_dir, 'statistics_report.txt')
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(report_text)
    
    print(f"✓ 统计报告已保存: {output_file}")
    print("\n" + report_text)


def main():
    parser = argparse.ArgumentParser(description='轨迹与性能分析工具')
    parser.add_argument('--bags', nargs='+', required=True,
                       help='rosbag 文件列表（按 local_planner dwa mpc 顺序）')
    parser.add_argument('--names', nargs='+', default=['local_planner', 'dwa', 'mpc'],
                       help='规划器名称列表')
    parser.add_argument('--output', default='./results',
                       help='输出目录')
    
    args = parser.parse_args()
    
    # 创建输出目录
    os.makedirs(args.output, exist_ok=True)
    
    print("\n" + "=" * 80)
    print("局部规划器轨迹与性能分析")
    print("=" * 80 + "\n")
    
    # 读取数据
    data_list = []
    for bag_file, name in zip(args.bags, args.names):
        data = extract_data_from_bag(bag_file, name)
        if data is not None:
            data_list.append(data)
    
    if len(data_list) == 0:
        print("错误: 没有成功读取任何数据")
        return
    
    print(f"\n成功读取 {len(data_list)} 个数据集\n")
    
    # 生成图表
    print("正在生成图表...")
    plot_trajectories(data_list, args.output)
    plot_speed_comparison(data_list, args.output)
    plot_control_smoothness(data_list, args.output)
    plot_performance_summary(data_list, args.output)
    
    # 生成统计报告
    print("\n正在生成统计报告...")
    generate_statistics_report(data_list, args.output)
    
    print("\n" + "=" * 80)
    print("✓ 分析完成！所有结果已保存到:", args.output)
    print("=" * 80 + "\n")


if __name__ == '__main__':
    main()






