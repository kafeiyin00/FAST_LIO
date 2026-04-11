#!/usr/bin/env python3

"""
双格式点云输出测试脚本
验证系统是否正确输出PointCloud2和Livox CustomMsg两种格式的合并点云
"""

import rospy
import time
import numpy as np
from sensor_msgs.msg import PointCloud2
from livox_ros_driver2.msg import CustomMsg
import sensor_msgs.point_cloud2 as pc2

class DualFormatMonitor:
    def __init__(self):
        rospy.init_node('dual_format_monitor', anonymous=True)
        
        # 订阅两种格式的合并点云
        self.pc2_sub = rospy.Subscriber('/merged_pointcloud', PointCloud2, self.pc2_callback)
        self.livox_sub = rospy.Subscriber('/merged_livox', CustomMsg, self.livox_callback)
        
        # 统计信息
        self.pc2_count = 0
        self.livox_count = 0
        self.start_time = time.time()
        self.last_time = time.time()
        
        self.pc2_points = []
        self.livox_points = []
        self.pc2_intervals = []
        self.livox_intervals = []
        
        print("Dual Format Point Cloud Monitor Started")
        print("Monitoring topics:")
        print("  PointCloud2: /merged_pointcloud")
        print("  Livox CustomMsg: /merged_livox")
        print("=" * 60)
    
    def pc2_callback(self, msg):
        current_time = time.time()
        
        # 计算帧间间隔
        if self.pc2_count > 0:
            interval = current_time - self.last_time
            self.pc2_intervals.append(interval)
            
            # 保持最近50帧的间隔记录
            if len(self.pc2_intervals) > 50:
                self.pc2_intervals.pop(0)
        
        self.pc2_count += 1
        self.last_time = current_time
        
        # 解析点云数据
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        point_count = len(points)
        self.pc2_points.append(point_count)
        
        # 保持最近记录
        if len(self.pc2_points) > 50:
            self.pc2_points.pop(0)
        
        # 每10帧输出一次统计
        if self.pc2_count % 10 == 0:
            avg_interval = np.mean(self.pc2_intervals) if self.pc2_intervals else 0
            current_fps = 1.0 / avg_interval if avg_interval > 0 else 0
            
            print(f"📊 PointCloud2 #{self.pc2_count:4d}: "
                  f"Points={point_count:6d}, "
                  f"FPS={current_fps:5.1f}, "
                  f"Frame={msg.header.frame_id}")
    
    def livox_callback(self, msg):
        current_time = time.time()
        
        # 计算帧间间隔
        if self.livox_count > 0:
            interval = current_time - self.last_time
            self.livox_intervals.append(interval)
            
            # 保持最近50帧的间隔记录
            if len(self.livox_intervals) > 50:
                self.livox_intervals.pop(0)
        
        self.livox_count += 1
        
        # 获取点云信息
        point_count = len(msg.points)
        self.livox_points.append(point_count)
        
        # 保持最近记录
        if len(self.livox_points) > 50:
            self.livox_points.pop(0)
        
        # 每10帧输出一次统计
        if self.livox_count % 10 == 0:
            avg_interval = np.mean(self.livox_intervals) if self.livox_intervals else 0
            current_fps = 1.0 / avg_interval if avg_interval > 0 else 0
            
            print(f"🔶 Livox CustomMsg #{self.livox_count:4d}: "
                  f"Points={point_count:6d}, "
                  f"FPS={current_fps:5.1f}, "
                  f"LidarID={msg.lidar_id}, "
                  f"Frame={msg.header.frame_id}")
    
    def print_comparison(self):
        """打印两种格式的对比统计"""
        elapsed_time = time.time() - self.start_time
        
        print(f"\n{'='*60}")
        print(f"DUAL FORMAT COMPARISON (Running for {elapsed_time:.1f}s)")
        print(f"{'='*60}")
        
        if self.pc2_count == 0 and self.livox_count == 0:
            print("❌ No merged point cloud messages received!")
            print("   Make sure the merge_lidar node is running")
            return
        
        # PointCloud2 统计
        if self.pc2_count > 0:
            pc2_fps = self.pc2_count / elapsed_time
            pc2_avg_points = np.mean(self.pc2_points) if self.pc2_points else 0
            pc2_avg_interval = np.mean(self.pc2_intervals) if self.pc2_intervals else 0
            
            print(f"📊 PointCloud2 Format:")
            print(f"    Messages: {self.pc2_count}")
            print(f"    Average FPS: {pc2_fps:.2f}")
            print(f"    Average Points: {pc2_avg_points:.0f}")
            print(f"    Average Interval: {pc2_avg_interval*1000:.1f}ms")
        else:
            print("📊 PointCloud2 Format: No messages received")
        
        # Livox CustomMsg 统计
        if self.livox_count > 0:
            livox_fps = self.livox_count / elapsed_time
            livox_avg_points = np.mean(self.livox_points) if self.livox_points else 0
            livox_avg_interval = np.mean(self.livox_intervals) if self.livox_intervals else 0
            
            print(f"🔶 Livox CustomMsg Format:")
            print(f"    Messages: {self.livox_count}")
            print(f"    Average FPS: {livox_fps:.2f}")
            print(f"    Average Points: {livox_avg_points:.0f}")
            print(f"    Average Interval: {livox_avg_interval*1000:.1f}ms")
        else:
            print("🔶 Livox CustomMsg Format: No messages received")
        
        # 同步性检查
        if self.pc2_count > 0 and self.livox_count > 0:
            count_diff = abs(self.pc2_count - self.livox_count)
            points_diff = abs(np.mean(self.pc2_points) - np.mean(self.livox_points)) if self.pc2_points and self.livox_points else 0
            
            print(f"\n🔄 Synchronization Check:")
            print(f"    Message count difference: {count_diff}")
            print(f"    Average points difference: {points_diff:.0f}")
            
            if count_diff <= 2 and points_diff <= 10:
                print("    ✅ Formats are well synchronized!")
            else:
                print("    ⚠️  Formats may have synchronization issues")

def main():
    monitor = DualFormatMonitor()
    
    rate = rospy.Rate(0.2)  # 5秒间隔
    
    try:
        print("Monitoring dual format output... Press Ctrl+C to stop")
        while not rospy.is_shutdown():
            monitor.print_comparison()
            rate.sleep()
            
    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print("FINAL DUAL FORMAT SUMMARY")
        print(f"{'='*60}")
        
        monitor.print_comparison()
        
        if monitor.pc2_count > 0 and monitor.livox_count > 0:
            print(f"\n✅ Both PointCloud2 and Livox CustomMsg formats are working!")
            print(f"   You can subscribe to either:")
            print(f"   • /merged_pointcloud (PointCloud2)")
            print(f"   • /merged_livox (Livox CustomMsg)")
        elif monitor.pc2_count > 0:
            print(f"\n⚠️  Only PointCloud2 format is working")
        elif monitor.livox_count > 0:
            print(f"\n⚠️  Only Livox CustomMsg format is working")
        else:
            print(f"\n❌ No point cloud data received")
            print(f"   Please check if merge_lidar node is running")

if __name__ == '__main__':
    main()
