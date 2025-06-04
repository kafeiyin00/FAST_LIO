#!/bin/bash

xterm -e "source /opt/ros/noetic/setup.bash; \
                source /home/root01/ws_motor/devel/setup.bash; \
                rosnode kill -a " &

xterm -e "cd /home/root01/; \
                python3 rotor_control_stop.py " &

sleep 2s
                
xterm -e "pkill xterm " &
