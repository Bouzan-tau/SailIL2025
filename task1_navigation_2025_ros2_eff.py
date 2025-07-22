#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, degrees, sqrt
from std_msgs.msg import String
from collections import defaultdict
import subprocess
import os
import requests
from cv_from_zed_ros2.msg import ObjectDistanceInfo  # Custom message type

# ===========================================
# Global Navigation Configuration - Constants
# ===========================================

DISTANCE_THRESHOLD = 8            # Maximum distance (in meters) to consider an object for navigation
TIMEOUT = 1500.0                  # Time (in seconds) to wait before shutting down if no valid gate is found
FREQ = 1.0                        # Frequency (Hz) of the navigation loop
MINUS_INFINITY = float('-inf')   # Used to detect invalid ZED depth readings

# === Navigation Fallback Commands ===
NAV_LEFT = 'NAV_LEFT'             # Command to steer left
NAV_RIGHT = 'NAV_RIGHT'           # Command to steer right
NAV_FORWARD = 'NAV_FORWARD'       # Command to continue straight

# === Mapping navigation commands to angles (in degrees)
NAV_TO_ANGLE = {
    NAV_LEFT: 315,                # Left turn angle
    NAV_RIGHT: 45,                # Right turn angle
    NAV_FORWARD: 0                # Straight ahead
}

# Global dictionary to store detected object positions by label
object_positions = defaultdict(list)

# === Utility Functions ===

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
    if p1 is None and p2 is None:
        return 0
    if p2:  # If two points are provided, calculate the midpoint between them
        middle_point = (
            (p1[0] + p2[0]) / 2,  # Average of x-coordinates (horizontal)
            (p1[1] + p2[1]) / 2,  # Average of y-coordinates (vertical, unused here)
            (p1[2] + p2[2]) / 2   # Average of z-coordinates (depth/forward)
        )
    else:
        middle_point = p1  # If only one point is provided, use it directly

    # Calculate the angle between the positive Z-axis (forward direction) and the point in the XZ plane
    # atan2(x, z) ensures the angle is correct for all quadrants
    angle = round(degrees(atan2(middle_point[0], middle_point[2])))
    if angle < 0:  # Adjust negative angles to the range [0, 360]
        angle += 360
    return angle


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
    Callback function triggered when new object distance information is received.
    Stores the (x, y, z) position of each detected object under its label.
    """
    object_positions[msg.label].append((
        msg.distance_x,  # Horizontal
        msg.distance_y,  # Vertical
        msg.distance_z,  # Depth
    ))

# === Supporting Classes ===
class TimerManager:
    def __init__(self, node):
        self.timer = None
        self.timer_running = False
        self.node = node

    def start_timer(self, duration):  # Start the timer if it's not already running
        if not self.timer_running:
            self.timer = self.node.create_timer(duration, self.timer_callback)
            self.timer_running = True

    def stop_timer(self):  # Stop the timer if it's running
        if self.timer_running and self.timer:
            self.timer.cancel()
            self.timer_running = False

    def timer_callback(self):  # Shutdown the ROS node if no objects are detected within the timeout
        rclpy.shutdown()
        self.node.get_logger().info("Finish")

# === Core Node Class ===

class Navigator(Node):
    def __init__(self):
        super().__init__('navigation_task1_node')
        self.publisher = self.create_publisher(String, 'steering_directions', 10)
        self.timer_manager = TimerManager(self)
        self.create_subscription(ObjectDistanceInfo, 'object_distance_info', position_callback, 10)
        self.create_timer(1.0 / FREQ, self.navigate)
        self.last_seen_right_or_left = None  # Keeps track of last known horizontal side of right buoy
        self.conf_set = False

    
    
    
    # def update_last_seen_right_or_left(self, red, green):
    #     # """
    #     # Update the last seen side of the yellow ball, based on its X coordinate.
    #     # Called only if yellow ball has valid X.
    #     # """
    #     if red and green is None:
    #         self.last_seen_right_or_left = 'left'
    #     if green and red is None:
    #         self.last_seen_right_or_left = 'right'
    #     if red and green:
    #         self.last_seen_right_or_left = None

    @staticmethod
    def get_bounds():
        """
        Identifies the nearest valid red and green buoys to determine the navigation gate.

        Assumes that:
        - The red buoy is always on the left.
        - The green buoy is always on the right.

        This function locates the closest red and green buoys (if within the defined distance threshold),
        verifies that their depth values are valid (i.e., not equal to MINUS_INFINITY),
        and returns them as left and right bounds respectively for navigation.

        Returns:
            tuple: A pair of 3D points (left_bound, right_bound), corresponding to the closest
                   valid red and green buoys. If a buoy is missing or invalid, its value will be None.
        """
        nearest_red_buoy = find_closest(object_positions['Red Buoy'])  # Closest red buoy
        nearest_green_buoy = find_closest(object_positions['Green Buoy'])  # Closest green buoy

        return nearest_red_buoy, nearest_green_buoy

    def publish_angle(self, angle):
        """Publishes the computed steering angle as a string message to the ROS2 topic."""
        msg = String()
        msg.data = f"{angle}"
        self.publisher.publish(msg)

    def navigate(self):
        """
        Main navigation loop. Determines the gate boundaries and publishes
        the appropriate steering angle based on detected object positions.
        Also handles fallback behaviors and task timeout if no navigation is possible.
        """
        if not self.conf_set:
            with open("/tmp/current_task.txt", "w") as f:
                f.write("task1")  # שנה ל-task1 או task3 לפי הקובץ
            self.conf_set = True
        
        if not object_positions['end']:
            return

        left_bound, right_bound = self.get_bounds()
        # self.update_last_seen_right_or_left(left_bound, right_bound)
        
        if right_bound and (right_bound[2] is MINUS_INFINITY or not left_bound):
            # Red only → green missing → steer right
            self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            self.timer_manager.stop_timer()
            
        elif left_bound and (left_bound[2] is MINUS_INFINITY or not right_bound):
            # Green only → red missing → steer left
            self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])
            self.timer_manager.stop_timer()
        
        elif left_bound and right_bound:
            angle = get_angle(left_bound, right_bound)
            if angle >= 315 or angle <= 45:
                self.publish_angle(angle)
            elif 180 < angle < 315:
                self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            elif 45 < angle <= 180:
                self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])
            self.timer_manager.stop_timer()

        else:
            if self.last_seen_right_or_left == 'left':
                self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])
            elif self.last_seen_right_or_left == 'right':
                self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            else:
                # No valid buoys – go forward and start timeout
                self.publish_angle(NAV_TO_ANGLE[NAV_FORWARD])
            # self.timer_manager.start_timer(TIMEOUT)

        object_positions.clear()

# === Main Execution ===
def main(args=None):
    """
     Initializes the ROS2 node, launches the GUI and TTS,
     and starts the navigation loop for Task 1.
     """
    rclpy.init(args=args)
    launch_gui()
    say(". Starting Task One.")
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()