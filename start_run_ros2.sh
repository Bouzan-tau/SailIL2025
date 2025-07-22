#!/bin/bash

# Start Object Detection
gnome-terminal -- bash -c "
cd /home/sail/foxy_ws;
source /opt/ros/foxy/setup.bash;
source ~/foxy_ws/install/setup.bash;
export PYTHONPATH=\$PYTHONPATH:/home/sail/foxy_ws/install/lib/python3.8/site-packages;
/bin/python /home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/start.py;
exec bash"

