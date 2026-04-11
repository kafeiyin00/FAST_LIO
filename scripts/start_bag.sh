#!/bin/bash



# 等待2秒，确保前一个节点启动


# 启动第一个 Fast LIO 节点
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_pano/devel/setup.bash; \
                roslaunch fast_lio mapping_mid360.launch" &
sleep 2
# xterm -e "source /opt/ros/noetic/setup.bash; \
#                 source \$HOME/workspace/ws_rotor/devel/setup.bash; \
#                 rosrun fast_lio transformPX4" &

xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_pano/devel/setup.bash; \
                roslaunch motor_tf_utility motor_tf_utility_handheld.launch" &

xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_pano/devel/setup.bash; \
                roslaunch uav_bridge uav_net_tx_with_config.launch" &

sleep 2
xterm -e "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_pano/devel/setup.bash; \
                rosbag play /home/iot/Downloads/1.bag --topics /rotor_encoder /livox/imu /livox/lidar \
                /fisheye/bleft/image_raw/compressed /fisheye/bright/image_raw/compressed /fisheye/left/image_raw/compressed /fisheye/right/image_raw/compressed" &


xterm -e "source /opt/ros/noetic/setup.bash; \
                rosbag record /cloud_registered /fisheye/bleft/image_raw/compressed \
                /fisheye/bright/image_raw/compressed /fisheye/left/image_raw/compressed \
                /fisheye/right/image_raw/compressed\
                /mavros/vision_pose/pose" &

