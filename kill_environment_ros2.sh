#!/bin/bash

# Trigger TTS shutdown message
curl -s -o /dev/null -X POST http://localhost:5000/say \
     -H "Content-Type: application/json" \
     -d '{"text":". Server is close."}' \
     --max-time 1 &

# Check if ROS2 daemon is running and stop it if necessary
if pgrep -f "ros2 daemon" > /dev/null; then
    echo "Killing ROS2 daemon..."
    ros2 daemon stop
fi

# Kill object detection process
echo "Killing object detection..."
pkill -f publisher_cv_2025_ros2.py


# Kill engine controller process
echo "Stopping engines..."
python3 /home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/kill_engine.py

echo "Killing engine controller..."
pkill -f engine_controller.py


# Give some time before killing server
sleep 5

# Kill the Flask server (server.py)
echo "Killing server_ros2.py..."
pkill -f server_ros2.py

echo "All relevant processes have been killed."
