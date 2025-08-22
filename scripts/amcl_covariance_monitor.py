#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import csv
import os
from datetime import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque

class AMCLCovarianceMonitor:
    def __init__(self):
        rospy.init_node('amcl_covariance_monitor', anonymous=True)
        
        # 数据存储
        self.max_points = 1000  # 最大显示点数
        self.timestamps = deque(maxlen=self.max_points)
        self.x_covariance = deque(maxlen=self.max_points)
        self.y_covariance = deque(maxlen=self.max_points)
        self.xy_covariance = deque(maxlen=self.max_points)
        self.yaw_covariance = deque(maxlen=self.max_points)
        
        # CSV文件设置
        self.csv_filename = f"amcl_covariance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.init_csv()
        
        # 图形设置
        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('AMCL Covariance Real-time Monitor')
        
        # 订阅话题
        self.subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # 启动动画
        self.animation = FuncAnimation(self.fig, self.update_plots, interval=100, blit=False)
        
        rospy.loginfo("AMCL协方差监控器已启动")
    
    def init_csv(self):
        """初始化CSV文件"""
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'timestamp', 'x_covariance', 'y_covariance', 'xy_covariance', 
                'yaw_covariance', 'full_covariance_matrix'
            ])
    
    def pose_callback(self, msg):
        """处理AMCL位姿消息"""
        # 提取协方差矩阵 (6x6)
        cov = np.array(msg.pose.covariance).reshape(6, 6)
        
        # 提取协方差数据
        x_cov = cov[0, 0]  # X方向协方差
        y_cov = cov[1, 1]  # Y方向协方差
        xy_cov = cov[0, 1]  # XY协方差
        yaw_cov = cov[5, 5]  # 偏航角协方差
        
        # 时间戳
        timestamp = rospy.get_time()
        
        # 存储数据
        self.timestamps.append(timestamp)
        self.x_covariance.append(x_cov)
        self.y_covariance.append(y_cov)
        self.xy_covariance.append(xy_cov)
        self.yaw_covariance.append(yaw_cov)
        
        # 写入CSV
        self.write_to_csv(timestamp, x_cov, y_cov, xy_cov, yaw_cov, cov.flatten())
    
    def write_to_csv(self, timestamp, x_cov, y_cov, xy_cov, yaw_cov, full_cov):
        """写入CSV文件"""
        with open(self.csv_filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                timestamp, x_cov, y_cov, xy_cov, yaw_cov, 
                ' '.join(map(str, full_cov))
            ])
    
    def update_plots(self, frame):
        """更新图表"""
        if len(self.timestamps) < 2:
            return
        
        # 清除所有子图
        for ax in self.axes.flat:
            ax.clear()
        
        # 确保所有数据长度一致
        min_length = min(len(self.timestamps), len(self.x_covariance), 
                        len(self.y_covariance), len(self.xy_covariance), 
                        len(self.yaw_covariance))
        
        if min_length < 2:
            return
            
        times = list(self.timestamps)[-min_length:]
        start_time = times[0] if times else 0
        relative_times = [(t - start_time) for t in times]
        
        x_cov_data = list(self.x_covariance)[-min_length:]
        y_cov_data = list(self.y_covariance)[-min_length:]
        xy_cov_data = list(self.xy_covariance)[-min_length:]
        yaw_cov_data = list(self.yaw_covariance)[-min_length:]
        
        # X方向协方差
        self.axes[0, 0].plot(relative_times, x_cov_data, 'b-', linewidth=2)
        self.axes[0, 0].set_title('X Covariance')
        self.axes[0, 0].set_ylabel('Covariance (m²)')
        self.axes[0, 0].grid(True)
        
        # Y方向协方差
        self.axes[0, 1].plot(relative_times, y_cov_data, 'r-', linewidth=2)
        self.axes[0, 1].set_title('Y Covariance')
        self.axes[0, 1].set_ylabel('Covariance (m²)')
        self.axes[0, 1].grid(True)
        
        # XY协方差
        self.axes[1, 0].plot(relative_times, xy_cov_data, 'g-', linewidth=2)
        self.axes[1, 0].set_title('XY Covariance')
        self.axes[1, 0].set_ylabel('Covariance (m²)')
        self.axes[1, 0].set_xlabel('Time (s)')
        self.axes[1, 0].grid(True)
        
        # 偏航角协方差
        self.axes[1, 1].plot(relative_times, yaw_cov_data, 'm-', linewidth=2)
        self.axes[1, 1].set_title('Yaw Covariance')
        self.axes[1, 1].set_ylabel('Covariance (rad²)')
        self.axes[1, 1].set_xlabel('Time (s)')
        self.axes[1, 1].grid(True)
        
        plt.tight_layout()
    
    def run(self):
        """运行监控器"""
        try:
            plt.show()
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("监控器已停止")
        finally:
            rospy.loginfo(f"数据已保存到: {self.csv_filename}")

if __name__ == '__main__':
    try:
        monitor = AMCLCovarianceMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
