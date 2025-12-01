#!/bin/bash

run_nohup() {
    nohup bash -c "$1" > $2 2>&1 &
}

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_driver/devel/setup.bash; roslaunch livox_ros_driver2 msg_MID360.launch" livox.log

run_nohup "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                cd \$HOME/workspace/ws_slam/src/FAST_LIO/scripts; \
                python3 rotor_control_start_zero.py " rotor_control_start_zero.log

sleep 2

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roslaunch fast_lio mapping_mid360.launch" fastlio.log

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roslaunch motor_tf_utility motor_tf_utility_vertical.launch" motor_tf.log

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roslaunch seeker 1seeker_nodelet.launch" seeker.log

sleep 5

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; cd \$HOME/workspace/ws_slam/src/FAST_LIO/scripts; python3 rotor_control_start.py" rotor.log

run_nohup "source /opt/ros/noetic/setup.bash; roslaunch mavros px4.launch fcu_url:=/dev/ttyACM_px4:921600" mavros.log

run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roslaunch uav_bridge uav_server.launch" server.log

#run_nohup "source /opt/ros/noetic/setup.bash; source \$HOME/workspace/ws_slam/devel/setup.bash; roslaunch uav_bridge gstreamer_video_stream.launch" gst.log

echo "All nodes started with nohup."
