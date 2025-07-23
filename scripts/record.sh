#!/bin/bash

xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                roscore " &
                
sleep 2

# 启动 rosbag 记录数据，并使用时间戳命名文件
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                cd \$HOME/Desktop/bags; rosbag record /livox/lidar /livox/imu /Odometry /rotor_encoder /mavros/imu/data_raw" &

