#!/usr/bin/env python3
from serial import Serial
import threading
import time
import rospy
import signal
import sys
from std_msgs.msg import Int32  # 保留原有
from fast_lio.msg import StampedInt32

# 全局变量定义
ser = None
stop_thread = False
current_enc = 0
current_angle = 0.0
angle_lock = threading.Lock()

def serial_read_thread(pub):
    global stop_thread, current_enc, current_angle
    while not stop_thread and not rospy.is_shutdown():
        try:
            data = ser.readline().decode('utf-8').strip()
            if data.startswith("#ENC:"):
                enc_value = int(data[5:])
                with angle_lock:
                    current_enc = enc_value
                    # 根据原有代码的换算关系，将编码器值转换为角度（度）
                    current_angle = enc_value * 360.0 / 65535
                msg = StampedInt32()
                msg.header.stamp = rospy.Time.now()
                msg.data = enc_value
                pub.publish(msg)
                rospy.loginfo(f"Published encoder value: {current_angle}, time: {msg.header.stamp.to_sec()}")
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
    global ser, stop_thread, current_angle
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialize ROS node
    rospy.init_node('rotor_control', anonymous=True)
    pub = rospy.Publisher('/rotor_encoder', StampedInt32, queue_size=10)  # 改为自定义消息类型

    try:
        # 尝试打开串口
        ser = Serial("/dev/ttyACM_rotor", 115200, timeout=1)
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

    # PD 控制器参数（可通过 ROS 参数调整）
    Kp = rospy.get_param('~kp', 150.0)
    Ki = rospy.get_param('~ki', 0.3)
    Kd = rospy.get_param('~kd', 8)
    max_speed = rospy.get_param('~max_speed', 100000)
    control_rate = rospy.get_param('~rate', 100.0)  # Hz
    target_angle = rospy.get_param('~target_angle', 295.0)  # 目标角度，默认为 0°

    # 额外控制参数：容差、稳态时间和超时，以及积分限幅
    tolerance = rospy.get_param('~tolerance', 1)  # deg
    settle_time = rospy.get_param('~settle_time', 1)  # seconds 要求误差在容差内持续时间
    timeout = rospy.get_param('~timeout', 30.0)  # seconds 最大运行时间
    integral_limit = rospy.get_param('~integral_limit', max_speed)  # 防止积分饱和

    prev_error = 0.0
    prev_time = time.time()
    integral = 0.0

    rate = rospy.Rate(control_rate)

    # PID 控制主循环 —— 在到达目标并稳定后退出脚本
    start_time = time.time()
    settled_since = None

    while not rospy.is_shutdown():
        with angle_lock:
            angle = current_angle
        now = time.time()
        dt = now - prev_time if now - prev_time > 1e-6 else 1e-6
        # 计算最短角度误差（考虑环绕）
        error = target_angle - angle
        if error > 180.0:
            error -= 360.0
        elif error < -180.0:
            error += 360.0

        # 积分项累积并限幅（反积分风）
        integral += error * dt
        # 限幅积分项，避免风up
        if integral > integral_limit:
            integral = integral_limit
        elif integral < -integral_limit:
            integral = -integral_limit

        # 微分项
        derivative = (error - prev_error) / dt

        # PID 计算
        control = Kp * error + Ki * integral + Kd * derivative

        # 将控制量映射到速度命令并限幅
        cmd = int(max(min(control, max_speed), -max_speed))

        try:
            # 发送速度命令到串口
            command = f"#speed{cmd}\n"
            ser.write(command.encode('utf-8'))
            print("发送命令:", command)
        except Exception as e:
            rospy.logwarn(f"发送速度命令失败: {e}")

        # 检查是否到达并稳定在目标角度
        if abs(error) <= tolerance:
            if settled_since is None:
                settled_since = now
            elif (now - settled_since) >= settle_time:
                rospy.loginfo(f"目标角度 {target_angle}° 达到并稳定 (误差 {error}°)，退出节点")
                break
        else:
            settled_since = None

        # 检查超时
        if (now - start_time) >= timeout:
            rospy.logwarn(f"达到超时 {timeout}s，停止控制")
            break

        prev_error = error
        prev_time = now

        rate.sleep()

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
