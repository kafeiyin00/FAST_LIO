#!/bin/bash

run_nohup() {
    nohup bash -c "$1" > $2 2>&1 &
}

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roscore" roscore.log
                
sleep 2

# 启动 rosbag 记录数据，并使用时间戳命名文件
run_nohup "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                cd \$HOME/Desktop/bags; \
                rosbag record /livox/lidar /livox/imu /Odometry /rotor_encoder /mavros/imu/data_raw /imu_data_raw \
                /fisheye/bleft/image_raw/compressed /fisheye/bright/image_raw/compressed /fisheye/left/image_raw/compressed /fisheye/right/image_raw/compressed" rosbag.log

echo "Recording started. Check roscore.log and rosbag.log for details."

