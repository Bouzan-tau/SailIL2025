#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, degrees, sqrt
from std_msgs.msg import String
from collections import defaultdict
import subprocess
import os
import requests

from cv_from_zed_ros2.msg import ObjectDistanceInfo

# ===============================
# Global Navigation Configuration
# ===============================

DISTANCE_THRESHOLD = 8            # Max distance (in meters) to consider an object relevant for navigation
TIMEOUT = 1500.0                  # Time (in seconds) to wait before shutting down if no valid gate is found
FREQ = 1.0                        # Frequency (Hz) of the navigation loop
MINUS_INFINITY = float('-inf')    # Used to detect invalid ZED depth readings

# === Navigation Fallback Commands ===
NAV_LEFT = 'NAV_LEFT'             # Command to steer left
NAV_RIGHT = 'NAV_RIGHT'           # Command to steer right
NAV_FORWARD = 'NAV_FORWARD'       # Command to continue straight

# === Mapping navigation commands to angles (in degrees) ===
NAV_TO_ANGLE = {
    NAV_LEFT: 315,                # Left turn angle
    NAV_RIGHT: 45,                # Right turn angle
    NAV_FORWARD: 0                # Straight ahead
}

# Global dictionary to store detected object positions by label
object_positions = defaultdict(list)

def launch_gui():
    """Launches the PyQt-based GUI for real-time object visualization."""
    gui_path = os.path.join(os.path.dirname(__file__), 'map_gui_viewer.py')
    subprocess.Popen(['python3', gui_path])


def say(text):
    """Sends a text message to the local TTS server for audio feedback."""
    try:
        requests.post("http://localhost:5000/say", json={"text": text})
    except Exception as e:
        print(f"Cannot connect to TTS server: {e}")

def is_valid_point(p):
    """
    Checks whether a given value represents a valid 3D point.

    A point is considered valid if:
    - It is a tuple or list of length 3
    - All elements are numeric (int or float)
    - None of the elements are equal to MINUS_INFINITY

    Args:
        p (Any): Point to validate.

    Returns:
        bool: True if p is a valid (x, y, z) point, False otherwise.
    """
    return (
        isinstance(p, (tuple, list)) and
        len(p) == 3 and
        all(isinstance(coord, (int, float)) for coord in p) and
        all(coord != MINUS_INFINITY for coord in p)
    )

def get_distance(p1, p2):
    """Calculating the Euclidean Distance between two points"""
    if p1 is None and p2 is not None:
        return sqrt((p2[0]) ** 2 + p2[1] ** 2 + p2[2] ** 2)
    elif p2 is None and p1 is not None:
        return sqrt((p1[0]) ** 2 + p1[1] ** 2 + p1[2] ** 2)
    elif p1 is not None and p2 is not None:
        return sqrt(((p1[0] - p2[0]) ** 2 + (p1[2] - p2[2]) ** 2))
    return float('inf')

def get_angle(p1, p2=None):
    """
    Computes the steering angle based on the position of one or two points, using atan2 function.

    Args:
        p1 (tuple): First point (x, y, z).
        p2 (tuple, optional): Second point (x, y, z). If not provided, angle is computed based on p1 alone.

    Returns:
        int: Angle in degrees, normalized to [0, 360).
    """
    if not p1 and not p2:
        return 0

    if p1 and p2:
        x = (p1[0] + p2[0]) / 2
        z = (p1[2] + p2[2]) / 2
    elif p1 or p2:
        x, z = (p1 or p2)[0], (p1 or p2)[2]
    else:
        return 0

    angle = round(degrees(atan2(x, z)))
    return angle % 360

def find_closest(obj_list):
    """
    Finds the closest object from a list of 3D points.

    Args:
        obj_list (list): List of (x, y, z) tuples.

    Returns:
        tuple or None: The closest object within DISTANCE_THRESHOLD, or None if none found.
    """
    closest_distance = float('inf')  # Start with infinity as the furthest possible distance
    nearest_obj = None  # No nearest object found yet
    for obj in obj_list:  # Loop through each object in the list
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:  # Check if the object's depth (z-coordinate) is within the threshold and closer than the current closest
            closest_distance = obj[2]  # Update the closest distance
            nearest_obj = obj  # Update the closest object
    return nearest_obj

# === ROS2 Callback ===

def position_callback(msg):
    """
    Receives new detected object positions and stores them in global dictionary by label.
    """
    object_positions[msg.label].append((
        msg.distance_x,
        msg.distance_y,
        msg.distance_z
    ))


# === Supporting Classes ===

class TimerManager:
    def __init__(self, node):
        self.node = node
        self.timer = None
        self.timer_running = False

    def start_timer(self, duration):
        """Start a timer if not already running."""
        if not self.timer_running:
            self.timer = self.node.create_timer(duration, self.timer_callback)
            self.timer_running = True

    def stop_timer(self):
        """Stop the timer if running."""
        if self.timer_running and self.timer is not None:
            self.timer.cancel()
            self.timer_running = False

    def timer_callback(self):
        """Called when timer expires — shuts down the node."""
        rclpy.shutdown()
        self.node.get_logger().info("Finish")

# === Core Node Class ===

class Navigator(Node):
    def __init__(self):
        super().__init__('navigation_task3_node')
        self.publisher = self.create_publisher(String, 'steering_directions', 10)
        self.timer_manager = TimerManager(self)
        self.create_subscription(ObjectDistanceInfo, 'object_distance_info', position_callback, 10)
        self.create_timer(1.0 / FREQ, self.navigate)
        self.last_seen = None  # Optional: track last seen object type if desired
        self.conf_set = False

    @staticmethod
    def get_bounds():
        """
        Finds the closest dock shape detected: triangle, square or circle.

        Returns:
            tuple: Two identical points (left_bound, right_bound) representing the detected dock shape position.
            If no shape detected, returns (None, None).
        """
        triangle = find_closest(object_positions['Dock Triangle'])
        square = find_closest(object_positions['Dock Square'])
        circle = find_closest(object_positions['Dock Circle'])

        if triangle:
            return triangle, triangle
        elif square:
            return square, square
        elif circle:
            return circle, circle
        else:
            return None, None

    def publish_angle(self, angle):
        """Publish the steering angle as a ROS2 String message."""
        msg = String()
        msg.data = f"{angle}"
        self.publisher.publish(msg)

    def navigate(self):
        """
        Main navigation loop:
        - Waits for 'end' marker indicating frame completion.
        - Obtains bounds (detected dock shape).
        - Calculates and publishes steering angle or fallback command.
        - Handles timer for no detection timeout.
        - Clears stored object positions after each cycle.
        """
        if not self.conf_set:
            with open("/tmp/current_task.txt", "w") as f:
              f.write("task3")  # שנה ל-task1 או task3 לפי הקובץ
            self.conf_set = True
            
        if not object_positions['end']:
            return

        left_bound, right_bound = self.get_bounds()

        if is_valid_point(left_bound) and is_valid_point(right_bound):
            angle = get_angle(left_bound, right_bound)
            self.publish_angle(angle)
            self.timer_manager.stop_timer()

        elif is_valid_point(left_bound):
            angle = get_angle(left_bound)
            # Simple fallback logic (e.g., steer right if angle is sharp)
            if angle < 90 or angle > 345:
                self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])
            else:
                self.publish_angle(NAV_TO_ANGLE[NAV_FORWARD])
            self.timer_manager.stop_timer()

        elif is_valid_point(right_bound):
            angle = get_angle(right_bound)
            # Simple fallback logic (e.g., steer left if angle is sharp)
            if angle > 270 or angle < 15:
                self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            else:
                self.publish_angle(NAV_TO_ANGLE[NAV_FORWARD])
            self.timer_manager.stop_timer()

        else:
            self.publish_angle(NAV_TO_ANGLE[NAV_FORWARD])
            self.timer_manager.start_timer(TIMEOUT)

        object_positions.clear()


# === Main Execution ===

def main(args=None):
    launch_gui()
    say(". Starting Task Three.")
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()