#!/bin/bash
echo "Stopping Task 1..."

# Closing task 1 processes (ROS2 version)
pkill -f task1_navigation_2025_ros2_eff.py

# Sending POST request to the server to read the wanted message (asynchronously)
curl -s -o /dev/null -X POST http://localhost:5000/say \
     -H "Content-Type: application/json" \
     -d '{"text":". Task 1 Finished."}' \
     --max-time 1 &

echo "Task 1 stopped and server notified asynchronously."

