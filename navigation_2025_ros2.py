#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import atan2, degrees, sqrt
from cv_from_zed.msg import ObjectDistanceInfo
from std_msgs.msg import String
from collections import defaultdict

# Define global variables to store objects' positions and labels
object_positions = defaultdict(list)

# Constants
DISTANCE_THRESHOLD = 8  # Maximum allowed distance (in meters) to consider an object for navigation
TIMEOUT = 15.0          # Time (in seconds) to wait for detecting objects before shutting down navigation
FREQ = 1                # Frequency (in Hz) for the navigation loop, controlling the main node's rate of operation
ANGLE_LEFT = 315        # Steering angle (degrees) to turn left; corresponds to a specific orientation
ANGLE_RIGHT = 45        # Steering angle (degrees) to turn right; corresponds to a specific orientation
ANGLE_FORWARD = 0       # Steering angle (degrees) for moving forward in the current direction


class TimerManager:
    def __init__(self, node):  # Node passed to TimerManager for timer access
        self.node = node
        self.timer = None
        self.timer_running = False

    def start_timer(self, duration):  # Start the timer if it's not already running
        if not self.timer_running:
            self.timer = self.node.create_timer(duration, self.timer_callback)  # Create ROS 2 timer
            self.timer_running = True

    def stop_timer(self):  # Stop the timer
        if self.timer_running:
            self.timer.cancel()  # Cancel the timer
            self.timer_running = False

    def timer_callback(self):
        self.node.get_logger().info('No objects detected, shutting down navigation...')
        rclpy.shutdown()  # Shutdown ROS 2 node


def position_callback(msg):
    # Store the detected object's position (x, y, z) under its label in object_positions
    object_positions[msg.label].append((msg.distance_x, msg.distance_y, msg.distance_z))


def find_closest(obj_list):
    closest_distance = float('inf')  # Start with infinity as the furthest possible distance
    nearest_obj = None
    for obj in obj_list:
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:
            closest_distance = obj[2]
            nearest_obj = obj
    return nearest_obj


def get_angle(p1, p2=None):
    if p2:
        middle_point = (
            (p1[0] + p2[0]) / 2,
            (p1[1] + p2[1]) / 2,
            (p1[2] + p2[2]) / 2
        )
    else:
        middle_point = p1

    angle = round(degrees(atan2(middle_point[0], middle_point[2])))
    if angle < 0:
        angle += 360

    return angle


def get_distance(p1, p2):
    if p1 is None and p2 is not None:
        return p2
    elif p2 is None and p1 is not None:
        return p1
    return sqrt(((p1[0] - p2[0]) ** 2 + (p1[2] - p2[2]) ** 2))


def get_bounds():
    nearest_red = find_closest(object_positions['Red ball'])
    nearest_green = find_closest(object_positions['Green ball'])
    nearest_yellow = find_closest(object_positions['Yellow ball'])

    if not nearest_yellow or nearest_red is None or nearest_green is None:
        return nearest_red, nearest_green
    else:
        if (nearest_red[0] < nearest_yellow[0]) and (nearest_green[0] > nearest_yellow[0]):
            if get_distance(nearest_yellow, nearest_green) > get_distance(nearest_red, nearest_yellow):
                return nearest_yellow, nearest_green
            return nearest_red, nearest_yellow
        else:
            return nearest_red, nearest_green


def publish_angle(steering_pub, angle):
    steering_pub.publish(String(data=str(angle)))


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.subscription = self.create_subscription(
            ObjectDistanceInfo,
            'object_distance_info',
            position_callback,
            10
        )
        self.steering_pub = self.create_publisher(String, 'steering_directions', 10)

        # Initialize an instance of the TimerManager class to handle timeout functionality
        self.timer = TimerManager(self)

        self.last_seen = None
        self.rate = self.create_rate(FREQ)  # Set the loop rate (Hz)


    def navigate(self):
        while rclpy.ok():
            if not object_positions.get('end', False):
                self.rate.sleep()
                continue

            left_bound, right_bound = get_bounds()

            if left_bound and right_bound:
                angle = get_angle(left_bound, right_bound)
                publish_angle(self.steering_pub, angle)
                self.timer.stop_timer()
            elif left_bound:
                angle = get_angle(left_bound)
                if angle < 90 or angle > 345:
                    publish_angle(self.steering_pub, ANGLE_RIGHT)
                else:
                    publish_angle(self.steering_pub, ANGLE_FORWARD)
                self.timer.stop_timer()
            elif right_bound:
                angle = get_angle(right_bound)
                if angle > 270 or angle < 15:
                    publish_angle(self.steering_pub, ANGLE_LEFT)
                else:
                    publish_angle(self.steering_pub, ANGLE_FORWARD)
                self.timer.stop_timer()
            else:
                self.timer.start_timer(TIMEOUT)
                if self.last_seen == 'r':
                    publish_angle(self.steering_pub, ANGLE_RIGHT)
                elif self.last_seen == 'g':
                    publish_angle(self.steering_pub, ANGLE_LEFT)
                else:
                    publish_angle(self.steering_pub, ANGLE_FORWARD)

            if left_bound and right_bound:
                self.last_seen = 'r&g'
            elif left_bound:
                self.last_seen = 'r'
            elif right_bound:
                self.last_seen = 'g'

            object_positions.clear()  # Clear object_positions for the next frame
            self.rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()

    try:
        node.navigate()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
