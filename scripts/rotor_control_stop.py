#!/usr/bin/env python3
import serial
import time

def main():
    try:
        # 尝试打开串口
        ser = serial.Serial("/dev/ttyACM_rotor", 9600, timeout=1)
        # 注意：通常Linux下串口设备路径为 /dev/ttyACM0，请根据实际情况修改
    except Exception as e:
        print("打开串口失败:", e)
        return

    # 等待串口初始化
    time.sleep(1)

    command = "#speed0\n"
    # 发送命令，编码为 UTF-8
    ser.write(command.encode('utf-8'))
    print("发送命令:", command)

    # 关闭串口
    ser.close()

if __name__ == '__main__':
    main()
