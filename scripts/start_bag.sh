#!/bin/bash



# 等待2秒，确保前一个节点启动


# 启动第一个 Fast LIO 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_rotor/devel/setup.bash; \
                roslaunch fast_lio mapping_mid360.launch" &
sleep 2
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_rotor/devel/setup.bash; \
                rosrun fast_lio transformPX4" &

sleep 2
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_rotor/devel/setup.bash; \
                rosbag play /media/iot/Jianping/rotor_uav/delta_0929/2025-09-26-20-23-43.bag" &

