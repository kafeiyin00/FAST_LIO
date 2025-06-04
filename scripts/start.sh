#!/bin/bash

# 启动 Livox ROS Driver 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                roslaunch livox_ros_driver2 msg_MID360.launch" &

# 等待2秒，确保前一个节点启动
sleep 2

# 启动第一个 Fast LIO 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                roslaunch fast_lio run_helmet_mid.launch" &

xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                rosrun fast_lio transformPX4" &

# 等待3秒
sleep 3
xterm -e "cd /home/root01/ws_motor/src/FAST_LIO/scripts; \
                python3 rotor_control_start.py " &
