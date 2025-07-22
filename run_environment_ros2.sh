#!/bin/bash

echo "🚀 Starting ROS2 Environment (Foxy only)..."

# === Start the TTS Server in virtual environment ===
gnome-terminal -- bash -c "
source /home/sail/Desktop/danel_gadya_sail_2025_git/tts_runner/venv/bin/activate && \
cd /home/sail/Desktop/danel_gadya_sail_2025_git && \
python3 server_ros2.py; \
exec bash"

# === Source ROS2 Foxy + your workspace ===
source /opt/ros/foxy/setup.bash
source /home/sail/foxy_ws/install/setup.bash

# === Start Object Detection Node (ROS2 Python script) ===
gnome-terminal -- bash -c "
cd /home/sail/Desktop/danel_gadya_sail_2025_git && \
python3 publisher_cv_2025_ros2.py; \
exec bash"

# === Start Engine Controller Node (ROS2 Python script) ===
gnome-terminal -- bash -c "
cd /home/sail/Desktop/danel_gadya_sail_2025_git && \
/bin/python /home/sail/Desktop/danel_gadya_sail_2025_git/engine_controller.py; \
exec bash"


