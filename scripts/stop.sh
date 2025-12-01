#!/bin/bash

bash -c "source /opt/ros/noetic/setup.bash; \
                source \$HOME/workspace/ws_slam/devel/setup.bash; \
                rosnode kill -a " &

bash -c "cd \$HOME/workspace/ws_slam/src/FAST_LIO/scripts; \
                 python3 rotor_control_stop.py " &

sleep 2s
                
bash -c "pkill xterm " &
