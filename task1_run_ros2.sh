#!/bin/bash

# Start Navigation (ROS2) + Steering Monitor
gnome-terminal -- bash -c "
cd /home/sail/foxy_ws;
source /opt/ros/foxy/setup.bash;
source install/setup.bash;
stdbuf -oL python3 /home/sail/Desktop/danel_gadya_sail_2025_git/task1_navigation_2025_ros2_eff.py &
stdbuf -oL ros2 topic echo /steering_directions; 
stdbuf -oL /bin/python /home/sail/Desktop/danel_gadya_sail_2025_git/engine_controller.py; \
exec bash"

