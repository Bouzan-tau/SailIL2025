#!/bin/bash
echo "Stopping Task 3..."

# Closing task 3 processes (ROS2 version)
pkill -f task3_navigation_2025_ros2_eff.py

# Sending POST request to the server to read the wanted message (asynchronously)
curl -s -o /dev/null -X POST http://localhost:5000/say \
     -H "Content-Type: application/json" \
     -d '{"text":". Task 3 Finished."}' \
     --max-time 1 &

echo "Task 3 stopped and server notified asynchronously."

