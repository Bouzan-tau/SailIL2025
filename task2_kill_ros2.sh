#!/bin/bash
echo "Stopping Task 2..."

# Closing task 2 processes (ROS2 version)
pkill -f task2_navigation_2025_ros2_eff.py

# Sending POST request to the server to read the wanted message (asynchronously)
curl -s -o /dev/null -X POST http://localhost:5000/say \
     -H "Content-Type: application/json" \
     -d '{"text":". Task 2 Finished."}' \
     --max-time 1 &

echo "Task 2 stopped and server notified asynchronously."

