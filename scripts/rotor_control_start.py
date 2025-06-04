#!/usr/bin/env python3
from serial import Serial
import threading
import time
import rospy
import signal
import sys
from std_msgs.msg import Int32  # Changed to Int32 for encoder counts

# 全局变量定义
ser = None
stop_thread = False

def serial_read_thread(pub):
    global stop_thread
    while not stop_thread and not rospy.is_shutdown():
        try:
            data = ser.readline().decode('utf-8').strip()
            if data.startswith("#ENC:"):
                enc_value = int(data[5:])
                pub.publish(Int32(enc_value))
                rospy.loginfo(f"Published encoder value: {enc_value*360.0/65535}")  # Assuming the value needs to be multiplied by 360
        except Exception as e:
            rospy.logwarn(f"Error reading serial data: {e}")

def signal_handler(sig, frame):
    global stop_thread
    print("\n程序正在停止...")
    stop_thread = True
    try:
        ser.write("#speed0\n".encode('utf-8'))
        print("已发送停止命令")
        time.sleep(0.5)
        ser.close()
        print("已关闭串口")
    except Exception as e:
        print(f"停止时发生错误: {e}")
    sys.exit(0)

def main():
    global ser, stop_thread
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS node
    rospy.init_node('rotor_control', anonymous=True)
    pub = rospy.Publisher('/rotor_encoder', Int32, queue_size=10)  # Changed topic name and message type

    try:
        # 尝试打开串口
        ser = Serial("/dev/ttyACM_rotor", 9600, timeout=1)
        # 注意：通常Linux下串口设备路径为 /dev/ttyACM0，请根据实际情况修改
    except Exception as e:
        print("打开串口失败:", e)
        return

    # 等待串口初始化
    time.sleep(2)

    command = "#start\n"
    # 发送命令，编码为 UTF-8
    ser.write(command.encode('utf-8'))
    print("发送命令:", command)
    
    # 创建串口读取线程
    read_thread = threading.Thread(target=serial_read_thread, args=(pub,))
    read_thread.daemon = True
    read_thread.start()

    last_command_time = time.time()
    # 持续读取数据并发布
    while not rospy.is_shutdown():
        # 只处理发送速度命令
        current_time = time.time()
        if current_time - last_command_time >= 0.3:
            command = "#speed3600\n"
            ser.write(command.encode('utf-8'))
            last_command_time = current_time
        rospy.sleep(0.01)

    stop_thread = True
    read_thread.join(timeout=1.0)
    
    command = "#speed0\n"
    # 发送命令，编码为 UTF-8
    ser.write(command.encode('utf-8'))
    print("发送命令:", command)
    
    # 关闭串口
    ser.close()

if __name__ == '__main__':
    main()
