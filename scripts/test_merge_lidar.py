#!/usr/bin/env python3

"""
多雷达融合系统测试脚本
用于验证融合后点云的质量和实时性能
"""

import rospy
import time
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

class LidarMergeMonitor:
    def __init__(self):
        rospy.init_node('lidar_merge_monitor', anonymous=True)
        
        # 订阅融合后的点云
        self.merged_sub = rospy.Subscriber('/merged_pointcloud', PointCloud2, self.merged_callback)
        
        # 统计信息
        self.frame_count = 0
        self.start_time = time.time()
        self.last_time = time.time()
        self.total_points = 0
        self.frame_intervals = []
        
        print("Lidar Merge Monitor Started")
        print("Monitoring topic: /merged_pointcloud")
        print("=" * 50)
    
    def merged_callback(self, msg):
        current_time = time.time()
        
        # 计算帧间间隔
        if self.frame_count > 0:
            interval = current_time - self.last_time
            self.frame_intervals.append(interval)
            
            # 保持最近100帧的间隔记录
            if len(self.frame_intervals) > 100:
                self.frame_intervals.pop(0)
        
        self.frame_count += 1
        self.last_time = current_time
        
        # 解析点云数据
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        point_count = len(points)
        self.total_points += point_count
        
        # 计算统计信息
        elapsed_time = current_time - self.start_time
        avg_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        avg_points_per_frame = self.total_points / self.frame_count if self.frame_count > 0 else 0
        
        # 计算实时频率
        if len(self.frame_intervals) > 0:
            avg_interval = np.mean(self.frame_intervals)
            current_fps = 1.0 / avg_interval if avg_interval > 0 else 0
        else:
            current_fps = 0
        
        # 每10帧输出一次统计
        if self.frame_count % 10 == 0:
            print(f"Frame {self.frame_count:4d}: "
                  f"Points={point_count:6d}, "
                  f"FPS={current_fps:5.1f}, "
                  f"AvgFPS={avg_fps:5.1f}, "
                  f"AvgPoints={avg_points_per_frame:.0f}")
        
        # 质量检查
        if point_count == 0:
            rospy.logwarn("Received empty merged point cloud!")
        
        # 检查点云数据合理性
        if points:
            x_coords = [p[0] for p in points[:1000]]  # 采样前1000个点
            y_coords = [p[1] for p in points[:1000]]
            z_coords = [p[2] for p in points[:1000]]
            
            if len(x_coords) > 0:
                x_range = max(x_coords) - min(x_coords)
                y_range = max(y_coords) - min(y_coords)
                z_range = max(z_coords) - min(z_coords)
                
                if self.frame_count % 50 == 0:  # 每50帧输出一次范围统计
                    print(f"  -> Point range: X={x_range:.1f}m, Y={y_range:.1f}m, Z={z_range:.1f}m")
                    
                    # 检查异常值
                    if x_range > 200 or y_range > 200 or z_range > 100:
                        rospy.logwarn("Point cloud range seems abnormal!")
    
    def print_summary(self):
        """打印最终统计报告"""
        if self.frame_count == 0:
            print("No frames received!")
            return
        
        elapsed_time = time.time() - self.start_time
        avg_fps = self.frame_count / elapsed_time
        avg_points = self.total_points / self.frame_count
        
        print("\n" + "=" * 50)
        print("LIDAR MERGE MONITOR SUMMARY")
        print("=" * 50)
        print(f"Total frames received: {self.frame_count}")
        print(f"Total time elapsed: {elapsed_time:.1f} seconds")
        print(f"Average FPS: {avg_fps:.2f}")
        print(f"Average points per frame: {avg_points:.0f}")
        print(f"Total points processed: {self.total_points}")
        
        if len(self.frame_intervals) > 0:
            intervals = np.array(self.frame_intervals)
            print(f"Frame interval stats:")
            print(f"  Mean: {np.mean(intervals)*1000:.1f}ms")
            print(f"  Std:  {np.std(intervals)*1000:.1f}ms")
            print(f"  Min:  {np.min(intervals)*1000:.1f}ms")
            print(f"  Max:  {np.max(intervals)*1000:.1f}ms")
        
        # 性能评估
        if avg_fps >= 15:
            print("✅ Fusion performance: EXCELLENT (>= 15 FPS)")
        elif avg_fps >= 10:
            print("✅ Fusion performance: GOOD (>= 10 FPS)")
        elif avg_fps >= 5:
            print("⚠️  Fusion performance: MODERATE (>= 5 FPS)")
        else:
            print("❌ Fusion performance: POOR (< 5 FPS)")

def main():
    monitor = LidarMergeMonitor()
    
    try:
        print("Press Ctrl+C to stop monitoring and show summary...")
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        monitor.print_summary()

if __name__ == '__main__':
    main()
