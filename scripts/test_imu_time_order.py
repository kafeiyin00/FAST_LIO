#!/usr/bin/env python3

"""
IMU时间序列验证脚本
验证系统是否按时间顺序发布转换后的IMU数据
"""

import rospy
import time
import numpy as np
from sensor_msgs.msg import Imu
from collections import deque

class ImuTimeOrderMonitor:
    def __init__(self):
        rospy.init_node('imu_time_order_monitor', anonymous=True)
        
        # 订阅合并后的IMU数据
        self.imu_sub = rospy.Subscriber('/merged_imu', Imu, self.imu_callback)
        
        # 统计信息
        self.imu_count = 0
        self.start_time = time.time()
        self.last_timestamp = None
        self.out_of_order_count = 0
        self.timestamps = deque(maxlen=1000)  # 保持最近1000个时间戳
        self.time_diffs = deque(maxlen=999)   # 时间差
        
        print("IMU Time Order Monitor Started")
        print("Monitoring topic: /merged_imu")
        print("Checking for time-ordered IMU data from multiple sensors")
        print("=" * 60)
    
    def imu_callback(self, msg):
        current_timestamp = msg.header.stamp.to_sec()
        self.imu_count += 1
        
        # 检查时间顺序
        if self.last_timestamp is not None:
            time_diff = current_timestamp - self.last_timestamp
            self.time_diffs.append(time_diff)
            
            if current_timestamp < self.last_timestamp:
                self.out_of_order_count += 1
                print(f"⚠️  Out-of-order IMU data detected! Previous: {self.last_timestamp:.6f}, Current: {current_timestamp:.6f}")
        
        self.timestamps.append(current_timestamp)
        self.last_timestamp = current_timestamp
        
        # 每100个IMU消息输出一次统计
        if self.imu_count % 100 == 0:
            self.print_statistics()
    
    def print_statistics(self):
        elapsed_time = time.time() - self.start_time
        imu_freq = self.imu_count / elapsed_time if elapsed_time > 0 else 0
        
        print(f"\n📊 IMU Statistics (Count: {self.imu_count})")
        print(f"   Frequency: {imu_freq:.1f} Hz")
        print(f"   Out-of-order: {self.out_of_order_count}")
        print(f"   Order accuracy: {((self.imu_count - self.out_of_order_count) / self.imu_count * 100):.1f}%")
        
        if len(self.time_diffs) > 0:
            avg_diff = np.mean(self.time_diffs)
            std_diff = np.std(self.time_diffs)
            min_diff = np.min(self.time_diffs)
            max_diff = np.max(self.time_diffs)
            
            print(f"   Time differences (s):")
            print(f"     Average: {avg_diff*1000:.2f}ms")
            print(f"     Std dev: {std_diff*1000:.2f}ms")
            print(f"     Min: {min_diff*1000:.2f}ms")
            print(f"     Max: {max_diff*1000:.2f}ms")
        
        # 检查最近时间戳的单调性
        if len(self.timestamps) >= 10:
            recent_timestamps = list(self.timestamps)[-10:]
            is_monotonic = all(recent_timestamps[i] <= recent_timestamps[i+1] 
                             for i in range(len(recent_timestamps)-1))
            status = "✅ Monotonic" if is_monotonic else "❌ Non-monotonic"
            print(f"   Recent 10 samples: {status}")
        
        print(f"   Frame ID: {getattr(self, 'last_frame_id', 'Unknown')}")
        print(f"   Coordinate system: body (transformed)")
    
    def analyze_imu_data(self, msg):
        """分析IMU数据内容"""
        self.last_frame_id = msg.header.frame_id
        
        # 每1000个消息输出一次详细分析
        if self.imu_count % 1000 == 0:
            print(f"\n🔍 IMU Data Analysis (Sample #{self.imu_count})")
            print(f"   Timestamp: {msg.header.stamp.to_sec():.6f}")
            print(f"   Frame ID: {msg.header.frame_id}")
            
            # 线性加速度
            linear_acc = msg.linear_acceleration
            acc_magnitude = np.sqrt(linear_acc.x**2 + linear_acc.y**2 + linear_acc.z**2)
            print(f"   Linear acceleration: [{linear_acc.x:.3f}, {linear_acc.y:.3f}, {linear_acc.z:.3f}] (|a|={acc_magnitude:.3f} m/s²)")
            
            # 角速度
            angular_vel = msg.angular_velocity
            gyro_magnitude = np.sqrt(angular_vel.x**2 + angular_vel.y**2 + angular_vel.z**2)
            print(f"   Angular velocity: [{angular_vel.x:.3f}, {angular_vel.y:.3f}, {angular_vel.z:.3f}] (|ω|={gyro_magnitude:.3f} rad/s)")
            
            # 四元数
            orientation = msg.orientation
            q_magnitude = np.sqrt(orientation.w**2 + orientation.x**2 + orientation.y**2 + orientation.z**2)
            print(f"   Orientation: [{orientation.w:.3f}, {orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}] (|q|={q_magnitude:.3f})")
    
    def imu_callback(self, msg):
        current_timestamp = msg.header.stamp.to_sec()
        self.imu_count += 1
        
        # 分析IMU数据
        self.analyze_imu_data(msg)
        
        # 检查时间顺序
        if self.last_timestamp is not None:
            time_diff = current_timestamp - self.last_timestamp
            self.time_diffs.append(time_diff)
            
            if current_timestamp < self.last_timestamp:
                self.out_of_order_count += 1
                print(f"⚠️  Out-of-order IMU data detected! Previous: {self.last_timestamp:.6f}, Current: {current_timestamp:.6f}")
        
        self.timestamps.append(current_timestamp)
        self.last_timestamp = current_timestamp
        
        # 每100个IMU消息输出一次统计
        if self.imu_count % 100 == 0:
            self.print_statistics()

def main():
    monitor = ImuTimeOrderMonitor()
    
    try:
        print("Monitoring IMU time order... Press Ctrl+C to stop")
        rospy.spin()
        
    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print("FINAL IMU TIME ORDER SUMMARY")
        print(f"{'='*60}")
        
        elapsed_time = time.time() - monitor.start_time
        total_freq = monitor.imu_count / elapsed_time if elapsed_time > 0 else 0
        
        print(f"Total IMU messages: {monitor.imu_count}")
        print(f"Total time: {elapsed_time:.1f}s")
        print(f"Average frequency: {total_freq:.1f} Hz")
        print(f"Out-of-order messages: {monitor.out_of_order_count}")
        
        if monitor.imu_count > 0:
            order_accuracy = (monitor.imu_count - monitor.out_of_order_count) / monitor.imu_count * 100
            print(f"Time order accuracy: {order_accuracy:.2f}%")
            
            if order_accuracy >= 99.0:
                print("✅ Excellent time ordering!")
            elif order_accuracy >= 95.0:
                print("⚠️  Good time ordering with minor issues")
            else:
                print("❌ Poor time ordering - check synchronization")
        
        if len(monitor.time_diffs) > 0:
            avg_period = np.mean(monitor.time_diffs)
            expected_freq = 1.0 / avg_period if avg_period > 0 else 0
            print(f"Average sampling period: {avg_period*1000:.2f}ms")
            print(f"Effective IMU frequency: {expected_freq:.1f} Hz")

if __name__ == '__main__':
    main()
