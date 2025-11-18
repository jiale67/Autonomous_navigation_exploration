#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
自动目标点发布节点
在系统启动后延迟指定时间，自动发送预设的目标点

使用方法：
  rosrun comparative_experiment auto_goal_publisher.py _goalX:=10.0 _goalY:=5.0 _delay:=5.0
"""

import rospy
import sys
from geometry_msgs.msg import PoseStamped

def publish_goal():
    """发布目标点到 /move_base_simple/goal 话题"""
    
    rospy.init_node('auto_goal_publisher', anonymous=False)
    
    # 获取参数
    goal_x = rospy.get_param('~goalX', 0.0)
    goal_y = rospy.get_param('~goalY', 0.0)
    goal_z = rospy.get_param('~goalZ', 0.0)
    delay = rospy.get_param('~delay', 5.0)
    frame_id = rospy.get_param('~frame_id', 'map')
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("自动目标点发布器已启动")
    rospy.loginfo("=" * 50)
    rospy.loginfo("目标点坐标: x=%.2f, y=%.2f, z=%.2f", goal_x, goal_y, goal_z)
    rospy.loginfo("参考坐标系: %s", frame_id)
    rospy.loginfo("延迟时间: %.1f 秒", delay)
    rospy.loginfo("=" * 50)
    
    # 等待延迟时间
    rospy.loginfo("等待 %.1f 秒后发送目标点...", delay)
    rospy.sleep(delay)
    
    # 创建发布者
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # 等待发布者连接
    rospy.sleep(0.5)
    
    # 构造目标点消息
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = frame_id
    goal_msg.pose.position.x = goal_x
    goal_msg.pose.position.y = goal_y
    goal_msg.pose.position.z = goal_z
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0
    
    # 发布目标点（多次发布以确保被接收）
    for i in range(5):
        pub.publish(goal_msg)
        rospy.loginfo("发送目标点 (第 %d 次): (%.2f, %.2f, %.2f)", 
                     i+1, goal_x, goal_y, goal_z)
        rospy.sleep(0.2)
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("✓ 目标点发送完成！")
    rospy.loginfo("=" * 50)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        rospy.loginfo("目标点发布器已关闭")
    except Exception as e:
        rospy.logerr("发生错误: %s", str(e))
        sys.exit(1)





















