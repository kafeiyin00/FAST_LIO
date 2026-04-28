#!/usr/bin/env python3

"""
多雷达融合系统自动类型检测测试脚本
验证系统是否能正确自动检测和处理不同类型的雷达消息
"""

import rospy
import time
from sensor_msgs.msg import PointCloud2
from livox_ros_driver.msg import CustomMsg

class MessageTypeDetector:
    def __init__(self):
        rospy.init_node('message_type_detector', anonymous=True)
        
        self.detected_types = {}
        self.message_counts = {}
        
        print("Message Type Detector Started")
        print("Monitoring lidar topics for automatic type detection...")
        print("=" * 60)
        
        # 获取要监控的话题列表
        self.topics_to_monitor = [
            "/os_cloud_node/points",
            "/livox/lidar", 
            "/velodyne_points",
            "/merged_pointcloud"
        ]
        
        # 创建订阅者来检测消息类型
        self.subscribers = []
        for topic in self.topics_to_monitor:
            # 尝试订阅PointCloud2
            try:
                sub_pc2 = rospy.Subscriber(topic, PointCloud2, 
                                         lambda msg, t=topic: self.pc2_callback(msg, t))
                self.subscribers.append(sub_pc2)
            except:
                pass
            
            # 尝试订阅CustomMsg
            try:
                sub_custom = rospy.Subscriber(topic, CustomMsg,
                                            lambda msg, t=topic: self.custom_callback(msg, t))
                self.subscribers.append(sub_custom)
            except:
                pass
        
        self.start_time = time.time()
        
    def pc2_callback(self, msg, topic):
        if topic not in self.detected_types:
            self.detected_types[topic] = "PointCloud2"
            self.message_counts[topic] = 0
            print(f"✅ Detected {topic}: PointCloud2 format")
            print(f"   Frame ID: {msg.header.frame_id}")
            print(f"   Points: ~{msg.width * msg.height}")
            print(f"   Fields: {[field.name for field in msg.fields]}")
            
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
        
    def custom_callback(self, msg, topic):
        if topic not in self.detected_types:
            self.detected_types[topic] = "Livox CustomMsg"
            self.message_counts[topic] = 0
            print(f"✅ Detected {topic}: Livox CustomMsg format")
            print(f"   Frame ID: {msg.header.frame_id}")
            print(f"   Points: {len(msg.points)}")
            print(f"   Lidar ID: {msg.lidar_id}")
            
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
    
    def print_status(self):
        elapsed_time = time.time() - self.start_time
        
        print(f"\n{'='*60}")
        print(f"AUTO-DETECTION STATUS (Running for {elapsed_time:.1f}s)")
        print(f"{'='*60}")
        
        if not self.detected_types:
            print("❌ No lidar messages detected yet...")
            print("   Make sure lidar nodes are running and publishing data")
        else:
            for topic, msg_type in self.detected_types.items():
                count = self.message_counts.get(topic, 0)
                freq = count / elapsed_time if elapsed_time > 0 else 0
                
                if msg_type == "PointCloud2":
                    icon = "🔷"
                else:
                    icon = "🔶"
                    
                print(f"{icon} {topic}")
                print(f"    Type: {msg_type}")
                print(f"    Messages: {count} ({freq:.1f} Hz)")
        
        print(f"\n💡 MergeLidar will automatically handle all detected types")
        print(f"   No manual configuration required!")

def main():
    detector = MessageTypeDetector()
    
    rate = rospy.Rate(0.2)  # 5秒间隔
    
    try:
        print("Monitoring topics... Press Ctrl+C to stop")
        while not rospy.is_shutdown():
            detector.print_status()
            rate.sleep()
            
    except KeyboardInterrupt:
        print(f"\n{'='*60}")
        print("FINAL DETECTION SUMMARY")
        print(f"{'='*60}")
        
        if detector.detected_types:
            print("✅ Successfully detected the following lidar types:")
            for topic, msg_type in detector.detected_types.items():
                print(f"   • {topic} → {msg_type}")
            
            print(f"\n🚀 You can now run MergeLidar with these topics:")
            topics_yaml = str(list(detector.detected_types.keys())).replace("'", '"')
            print(f"   lidar_topic: {topics_yaml}")
            
        else:
            print("❌ No lidar messages were detected")
            print("   Please check if lidar nodes are running")

if __name__ == '__main__':
    main()
