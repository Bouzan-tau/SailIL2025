# 🛠️ EngineController ROS 2 Node

This ROS 2 Python node (`engine_controller.py`) interfaces with an Arduino via serial to control three thrusters (left, right, and forward) using steering direction messages from a ROS 2 topic. the script is using a object detection algorithm with a camera , the camera is with the 2025 missions of the boat 

---

## 📦 Dependencies
Ensure the following packages are installed:

- ROS 2 (Foxy)
- Python 3
- `rclpy`, `std_msgs`
- `pyserial`
### Install serial support:
```bash
pip3 install pyserial




🚀 Features
Subscribes to ROS 2 topic: /steering_directions
Parses a steering angle and translates it into thruster commands
Sends commands over serial to an Arduino in the format:
l<left_engine_power>,r<right_engine_power>,f<front_engine_power>
for example: (20,30,50)
it will send sene force to the engines as follow:
left engine:20
right engine:30
front engine:50

# the power goes from 0 to 100.



🔌 Arduino Setup
Arduino must be connected via /dev/ttyUSB1 
ensure that this is the port the arduino is connected to- you can verify by running the bash commad: lsusb 
Baud rate: 9600




▶️ How to Run
Make sure your ROS 2 environment is sourced:
source /opt/ros/foxy/setup.bash
Run the program:
./home/sail/Desktop/danel_gadya_sail_2025_git/bashes4/start_run_ros2.sh
once you will open this script you will get options to run:
first write s to setup the enviorment, this will trigger the arduino to run the engines
then you will choose a mition that will respond to the camera output
when you need to kill press first the 'mission number' + 'k' and then 'sk' for kill the engines and the enviorment .


Notes:
1. sometime after running the script once the engines wont work, first check if the physical connection is established correctly, and if it is ok run the kill command and retry again, it should work.
2. be aware of getting to bashes repository , since all the scripts are related to bashes4 repository
3. If needed to adjust or change angles and the arduino doesnt respond first check the protocol using to convert the pwd bytes sent to the arduino.
4.A git repository exists in the url:"https://github.com/Bouzan-tau/SailIL2025" use it for validation



