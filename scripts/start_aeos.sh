#!/bin/bash

# 启动 Livox ROS Driver 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_driver/devel/setup.bash; \
                roslaunch livox_ros_driver2 msg_MID360.launch" &

# 等待2秒，确保前一个节点启动
sleep 2

# 启动第一个 Fast LIO 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                roslaunch fast_lio mapping_mid360.launch" &


xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                roslaunch motor_tf_utility motor_tf_utility_aeos.launch" &

# 等待3秒
sleep 5
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                cd \$HOME/workspace/ws_slam/src/FAST_LIO/scripts; \
                 python3 rotor_control_start.py " &
sleep 1
xterm -e "source /opt/ros/noetic/setup.bash; \
                 roslaunch mavros px4.launch fcu_url:=/dev/ttyACM_px4:921600 " &
