# Object Detection and Stirring Decision Project

This repository contains the following components:
- A pre-trained object detection model
- An object detection algorithm
- A stirring decision algorithm

## Requirements
- Ubuntu (Linux)
- ROS (Robot Operating System) installed
- Python 3.8+
- PyTorch (for loading the pre-trained model)
- ultralytics (for using the model)

## Instructions

1. **Clone the Repository:**
    ```bash
    git clone https://github.com/your_username/Object_Detection_Project.git
    cd Object_Detection_Project
    ```

2. **Save the detection algorithm and model:**
    - Ensure that the detection algorithm (`detection_algorithm.py`) and the pre-trained model (`object_detection_model.pth`) are saved in the respective directories as shown above.

3. **Run ROS Core:**
    - In your terminal, start the ROS master node:
    ```bash
    roscore
    ```

4. **Run the Detection Algorithm:**
    - Open a new terminal and navigate to the `src` directory:
    ```bash
    cd Object_Detection_Project/src
    ```
    - Run the detection algorithm script:
    ```bash
    python3 detection_algorithm.py
    ```

5. **Run the Stirring Decision Algorithm:**
    - In another terminal, also within the `src` directory:
    ```bash
    cd Object_Detection_Project/src
    ```
    - Run the stirring decision algorithm script:
    ```bash
    python3 stirring_decision.py
    ```

## Notes
- Ensure that ROS is properly configured and that your environment is correctly sourced.
- If you encounter any issues, verify that all dependencies are installed.
