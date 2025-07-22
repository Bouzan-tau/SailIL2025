#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, degrees, sqrt
from collections import defaultdict
from std_msgs.msg import String
from cv_from_zed_ros2.msg import ObjectDistanceInfo
import subprocess
import os
import requests

# ===========================================
# Global Navigation Configuration - Constants
# ===========================================

DISTANCE_THRESHOLD = 8           # Max distance (in meters) to consider an object relevant for navigation
TIMEOUT = 1500.0                 # Time (in seconds) to wait before stopping if no valid navigation is possible
FREQ = 1                         # Frequency (Hz) of the main navigation loop

# === Geometric Direction Constants (used for object positions, based on X coordinate) ===
LEFT_DIRECTION = 'left'          # Indicates an object is on the left side of the frame (X < 0)
RIGHT_DIRECTION = 'right'        # Indicates an object is on the right side of the frame (X > 0)

# === Navigation Fallback Commands (used when no valid gate is found) ===
NAV_LEFT = 'NAV_LEFT'            # Command to steer left
NAV_RIGHT = 'NAV_RIGHT'          # Command to steer right
NAV_FORWARD = 'NAV_FORWARD'      # Command to continue straight

# === Mapping from navigation commands to corresponding angles ===
NAV_TO_ANGLE = {
    NAV_LEFT: 315,
    NAV_RIGHT: 45,
    NAV_FORWARD: 0
}

# === Object Filtering Thresholds ===
YELLOW_COLLISION_THRESHOLD = 0.9   # Distance under which yellow is considered blocking (collision risk)
VIRTUAL_GATE_OFFSET = 1.8          # Lateral offset for creating virtual gates near yellow objects
FAR_ENOUGH_DISTANCE = 1.5          # Min depth required to consider a red/green object "far"
MINUS_INFINITY = float('-inf')     # Used to detect invalid depth measurements from ZED

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


def is_obj_far(p1):
    """
    Returns True if p1 is considered too far
    """
    return is_valid_point(p1) and p1[2] > FAR_ENOUGH_DISTANCE


def create_virtual_gate(yellow, direction):
    """
    Creates a virtual left/right point offset from the yellow ball.
    direction: 'left' or 'right'
    """
    direction = -1 if direction == LEFT_DIRECTION else 1
    return yellow[0] + direction * VIRTUAL_GATE_OFFSET, yellow[1], yellow[2]


def find_closest(obj_list):
    """
    Finds the closest object from a list of 3D points.

    Args:
        obj_list (list): List of (x, y, z) tuples.

    Returns:
        tuple or None: The closest object within DISTANCE_THRESHOLD, or None if none found.
    """
    closest_distance = float('inf')
    nearest_obj = None
    for obj in obj_list:
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:
            closest_distance = obj[2]
            nearest_obj = obj
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
        self.node = node
        self.timer = None
        self.timer_running = False

    def start_timer(self, duration):  # Start the timer if it's not already running
        if not self.timer_running:
            self.timer = self.node.create_timer(duration, self.timer_callback)
            self.timer_running = True

    def stop_timer(self):  # Stop the timer if it's running
        if self.timer_running and self.timer is not None:
            self.timer.cancel()
            self.timer_running = False

    def timer_callback(self):
        rclpy.shutdown()
        self.node.get_logger().info("Finish")


# === Core Node Class ===

class Navigator(Node):
    def __init__(self):
        super().__init__('navigation_task2_node')
        self.publisher = self.create_publisher(String, 'steering_directions', 10)
        self.timer_manager = TimerManager(self)
        self.create_subscription(ObjectDistanceInfo, 'object_distance_info', position_callback, 10)
        self.create_timer(1.0 / FREQ, self.navigate)
        self.last_seen_yellow_side = None  # Keeps track of last known horizontal side of yellow ball
        self.conf_set = False


    def update_last_seen_yellow_side(self, yellow):
        """
        Update the last seen side of the yellow ball, based on its X coordinate.
        Called only if yellow ball has valid X.
        """
        if yellow[0] != MINUS_INFINITY:
            if yellow[0] < 0:
                self.last_seen_yellow_side = LEFT_DIRECTION
            else:
                self.last_seen_yellow_side = RIGHT_DIRECTION

    @staticmethod
    def navigate_missing_bound(potential_yellow, other, other_pos):
        """
        Checks if yellow bound exists. If it does, moves between it and the other bound.
        Otherwise, moves just using the other bound.

        Args:
            potential_yellow (tuple or None): The potential yellow bound (might be None).
            other (tuple): The other bound (red / green).
            other_pos (str): Direction of the existing bound — 'left' for Red, 'right' for Green.

        Returns:
            tuple: A pair of points representing the gate (left_bound, right_bound).
        """
        if potential_yellow:
            return (other, potential_yellow) if other_pos == LEFT_DIRECTION else (potential_yellow, other)
        return (other, None) if other_pos == LEFT_DIRECTION else (None, other)

    @staticmethod
    def navigate_between_yellow_and_single_bound(yellow, other):
        """
        Checks if the yellow ball is too close to the other bound (either red or green, but not both).
        Green is always to the right of the yellow, and red is always to the left of it.
        Therefore, if the yellow is too close to the other bound which is green,
        or far enough from the other bound which is red – navigate left to the yellow using a left virtual gate.
        Otherwise, it means the yellow is too close to the red or far enough from the green –
        so navigate right to the yellow using a right virtual gate.

        Args:
            yellow (tuple): The yellow ball’s position in (x, y, z).
            other (tuple): The single bound’s position (either red or green).

        Returns:
            tuple: A pair of points (left_bound, right_bound) representing the chosen gate.
        """

        if ((get_distance(yellow, other) < YELLOW_COLLISION_THRESHOLD and yellow[0] < other[0]) or
                (get_distance(yellow, other) >= YELLOW_COLLISION_THRESHOLD and yellow[0] > other[0])):
            return create_virtual_gate(yellow, LEFT_DIRECTION), yellow
        return yellow, create_virtual_gate(yellow, RIGHT_DIRECTION)

    @staticmethod
    def navigate_when_two_bounds_and_yellow_exist(yellow, other):
        """
        Handles the case where both red and green exist, but only one of them is close to the yellow ball.
        If the yellow is too close to the bound, the function avoids it by placing a virtual gate on the opposite side.

        Args:
            yellow (tuple): The yellow ball’s position (x, y, z).
            other (tuple): The bound that is close to the yellow (either red or green).

        Returns:
            tuple: A pair of points (left_bound, right_bound) representing the chosen gate.
        """
        is_collision = get_distance(yellow, other) < YELLOW_COLLISION_THRESHOLD
        if yellow[0] < other[0]:  # other is green (since yellow is always between red and green)
            return (create_virtual_gate(yellow, LEFT_DIRECTION), yellow) if is_collision else (yellow, other)
        return (yellow, create_virtual_gate(yellow, RIGHT_DIRECTION)) if is_collision else (other, yellow)

    def get_bounds(self):
        """
        Determines the optimal gate for navigation based on the positions of red, green, and yellow balls.

        Logic Overview:

        1. Initialization:
           - Finds the closest red, green, and yellow balls (within a distance threshold).
           - If the yellow ball is valid, updates the last seen side (left/right) using its X coordinate.

        2. Red or Green is invalid:
           - If green exist but invalid → steer left
           - If red exist but invalid → steer right

        3. Special Case – Yellow too close (depth = -inf or invalid):
           - If yellow is detected but its Z value is invalid:
             * If last seen yellow side is 'left' → steer right
             * If last seen yellow side is 'right' → steer left
             * If unknown → go forward

        4. Yellow is missing or too far:
           - If red and green are valid → navigate between them directly (standard gate)
           - If only red or green is available:
            a) if yellow exist → use `navigate_missing_bound()`
            b) if yellow does not exist → turn right if only red, turn left if only green
           - If none exist → go forward

        5. Yellow is present and relevant:
           - If red and green are missing → create virtual gate around yellow based on its X
           - If only one bound (red or green) exists → use `navigate_between_yellow_and_single_bound()`
           - If both red and green exist:
             a) If both are far → treat yellow as primary and place a gate relative to it (using virtual gate)
             b) If yellow is close to only one of them → use `navigate_when_two_bounds_and_yellow_exist()` to avoid collision and select safe gate side

        This method ensures robust and flexible gate selection by combining geometric reasoning,
        proximity checks, and memory of past yellow direction to handle occlusions, partial detections,
        and edge cases.

        Returns:
            tuple: A pair of points (left_bound, right_bound) representing the chosen navigation gate,
                   or directional indicators ("left", "right", None) in fallback scenarios.
        """

        # 1. Initialization
        nearest_red = find_closest(object_positions['Red Ball'])
        nearest_green = find_closest(object_positions['Green Ball'])
        nearest_yellow = find_closest(object_positions['Yellow Ball'])

        if is_valid_point(nearest_yellow):
            self.update_last_seen_yellow_side(nearest_yellow)

        # 2. Red or Green is invalid
        if nearest_green and not is_valid_point(nearest_green):
            return None, NAV_LEFT

        if nearest_red and not is_valid_point(nearest_red):
            return NAV_RIGHT, None

        # 3. Special Case – Yellow too close (depth = -inf)
        if nearest_yellow and not is_valid_point(nearest_yellow):
            if self.last_seen_yellow_side == LEFT_DIRECTION:
                return NAV_RIGHT, None
            elif self.last_seen_yellow_side == RIGHT_DIRECTION:
                return None, NAV_LEFT
            else:
                return NAV_FORWARD, None

        # 4. Yellow is missing or too far
        if not nearest_yellow or nearest_yellow[2] > FAR_ENOUGH_DISTANCE:
            if nearest_red and nearest_green:
                return nearest_red, nearest_green
            elif nearest_red and not nearest_green:
                return self.navigate_missing_bound(nearest_yellow, nearest_red, LEFT_DIRECTION)
            elif not nearest_red and nearest_green:
                return self.navigate_missing_bound(nearest_yellow, nearest_green, RIGHT_DIRECTION)
            else:
                return NAV_FORWARD, None

        # 5. Yellow is present and relevant
        else:
            # Red and green are missing
            if not nearest_red and not nearest_green:
                if nearest_yellow[0] > 0:
                    return create_virtual_gate(nearest_yellow, LEFT_DIRECTION), nearest_yellow
                return nearest_yellow, create_virtual_gate(nearest_yellow, RIGHT_DIRECTION)

            # Only one bound (red) exist
            elif nearest_red and not nearest_green:
                return self.navigate_between_yellow_and_single_bound(nearest_yellow, nearest_red)

            # Only one bound (green) exist
            elif not nearest_red and nearest_green:
                return self.navigate_between_yellow_and_single_bound(nearest_yellow, nearest_green)

            else:
                # 5a — Red & Green both exist and are far
                if is_obj_far(nearest_red) and is_obj_far(nearest_green):
                    if nearest_yellow[0] > 0:
                        return create_virtual_gate(nearest_yellow, LEFT_DIRECTION), nearest_yellow
                    return nearest_yellow, create_virtual_gate(nearest_yellow, RIGHT_DIRECTION)
                # 5b — One bound is near the yellow, checking yellow interference
                if not is_obj_far(nearest_green):
                    return self.navigate_when_two_bounds_and_yellow_exist(nearest_yellow, nearest_green)

                if not is_obj_far(nearest_red):
                    return self.navigate_when_two_bounds_and_yellow_exist(nearest_yellow, nearest_red)

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
                f.write("task2")  # שנה ל-task1 או task3 לפי הקובץ
            self.conf_set = True
        if not object_positions['end']:
            return

        left_bound, right_bound = self.get_bounds()

        if is_valid_point(left_bound) and is_valid_point(right_bound):
            angle = get_angle(left_bound, right_bound)
            if angle >= 315 or angle <= 45:
                self.publish_angle(angle)
            elif 315 > angle > 180:
                self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            elif 45 < angle <= 180:
                self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])

            self.timer_manager.stop_timer()

        elif is_valid_point(left_bound):
            self.publish_angle(NAV_TO_ANGLE[NAV_RIGHT])
            self.timer_manager.stop_timer()

        elif is_valid_point(right_bound):
            self.publish_angle(NAV_TO_ANGLE[NAV_LEFT])
            self.timer_manager.stop_timer()

        elif isinstance(left_bound, str) and left_bound in NAV_TO_ANGLE:
            self.publish_angle(NAV_TO_ANGLE[left_bound])
            self.timer_manager.stop_timer()

        elif isinstance(right_bound, str) and right_bound in NAV_TO_ANGLE:
            self.publish_angle(NAV_TO_ANGLE[right_bound])
            self.timer_manager.stop_timer()

        else:
            self.publish_angle(NAV_TO_ANGLE[NAV_FORWARD])
            self.timer_manager.start_timer(TIMEOUT)

        object_positions.clear()


# === Main Execution ===

def main(args=None):
    """
    Initializes the ROS2 node, launches the GUI and TTS,
    and starts the navigation loop for Task 2.
    """
    rclpy.init(args=args)
    launch_gui()
    say(". Starting Task Two.")
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
