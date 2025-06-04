#!/bin/bash

xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                roscore " &

# 启动 rosbag 记录数据，并使用时间戳命名文件
xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                cd /home/root01/rosbags; rosbag record /livox/lidar /livox/imu /Odometry " &

