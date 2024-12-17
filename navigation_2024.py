#!/usr/bin/env python

import rospy
from math import atan2, degrees
from cv_from_zed.msg import ObjectDistanceInfo
from std_msgs.msg import String
from collections import defaultdict

# Define global variables to store objects' positions and labels
object_positions = defaultdict(list)

# Constants
DISTANCE_THRESHOLD = 8 # Maximum distance of object taken account [meters]
TIMEOUT = 5.0 # Timer duration [seconds]
FREQ = 1 # Program loop frequency [Hrz] 
ANGLE_LEFT = 315 # Sharp turn right angle [degree]
ANGLE_RIGHT = 45 # Sharp left right angle [degree]
ANGLE_FORWARD = 0 # Moving straight forward angle [degree]

class TimerManager:
    # Class managing the timer of the program

    def __init__(self):
        self.timer = None # Timer variable of ROS timer type
        self.timer_running = False # Boolean marking the state of the timer

    def start_timer(self, duration):
        if not self.timer_running:
            self.timer = rospy.Timer(rospy.Duration(duration), timer_callback)
            self.timer_running = True

    def stop_timer(self):
        if self.timer_running:
            self.timer.shutdown()
            self.timer_running = False

def position_callback(msg):
    # Callback function of subscribed topic for objects receiving

    object_positions[msg.label].append((
        msg.distance_x,  # Horizontal
        msg.distance_y,  # Vertical
        msg.distance_z,  # Depth
    ))

def timer_callback(event):
    # Timer callback for time-out

    rospy.signal_shutdown("No objects detected")
    rospy.loginfo("Finish")

def find_closest(obj_list):
    # Function receiving a list of objects and returning the closest object

    closest_distance = float('inf')
    nearest_obj = None
    for obj in obj_list:
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:
            closest_distance = obj[2]
            nearest_obj = obj

    return nearest_obj

def get_angle(p1, p2=None):
    # Function receiving a point in space and returing the angle to it,
    # or two points in space and returning the angle to the middle point

    if p2:
        middle_point = ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2)
    else:
        middle_point = p1
    angle = round(degrees(atan2(middle_point[0], middle_point[2])))

    if angle < 0: # Engine input is in the range of [0-359] degrees
        angle += 360

    return angle

def get_bounds():
    # Function finding the left and right and right bound to navigate in the current frame

    # Find the closest ball of each color, None if not existing
    nearest_red = find_closest(object_positions['Red ball'])
    nearest_green = find_closest(object_positions['Green ball'])
    nearest_black = find_closest(object_positions['Black ball'])
    nearest_yellow = find_closest(object_positions['Yellow ball'])

    if not (nearest_yellow or nearest_black):
        # If there are no yellow and black balls - navigate with red and green
        return nearest_red, nearest_green
    elif nearest_black and nearest_yellow:
        if nearest_black[2] < nearest_yellow[2]:
            # If black and yellow balls are in frame, and black is closer - navigate with black and green
            return nearest_black, nearest_green
        else:
            # If black and yellow balls are in frame, and yellow is closer - navigate with red and yellow
            return nearest_red, nearest_yellow
    elif nearest_black:
        # If only black exists - navigate with black and green
        return nearest_black, nearest_green
    else:
        # If only yellow exists - navigate with red and yellow
        return nearest_red, nearest_yellow

def publish_angle(steering_pub, angle):
    steering_pub.publish(f"{angle}")

def navigate():
    rospy.init_node('navigation_node', anonymous=True)
    rospy.Subscriber('object_distance_info', ObjectDistanceInfo, position_callback)
    steering_pub = rospy.Publisher('steering_directions', String, queue_size=10)
    timer = TimerManager()
    rate = rospy.Rate(FREQ)
    last_seen = None

    while not rospy.is_shutdown():
        if not object_positions['end']:
            # 'end' marker was not received yet, sleep and wait for a full frame to be received
            rate.sleep()
            continue

        # Get current frame navigation bounds
        left_bound, right_bound = get_bounds()

        if left_bound and right_bound:
            # Both bound exist - navigate to the middle of the course
            angle = get_angle(left_bound, right_bound)
            publish_angle(steering_pub, angle)
            timer.stop_timer()
        elif left_bound:
            # Only left bound is in frame
            angle = get_angle(left_bound)
            if angle < 90 or angle > 345:
                # If left bound is on the right side of the boat (<90), or not left enough (>345) - turn right
                publish_angle(steering_pub, ANGLE_RIGHT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
        elif right_bound:
            # Only right bound in frame
            angle = get_angle(right_bound)
            if angle > 270 or angle < 15:
                # If right bound is on the left side of the boat (>270), or not right enough (<15) - turn right
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
        else:
            # No bound for navigation are seen in the current frame, navigate by last obstacle seen
            timer.start_timer(TIMEOUT)
            if last_seen == 'r':
                publish_angle(steering_pub, ANGLE_RIGHT)
            elif last_seen == 'g':
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)

        # Update last obstacle seen
        if left_bound and right_bound:
            last_seen = 'r&g'
        elif left_bound:
            last_seen = 'r'
        elif right_bound:
            last_seen = 'g'
        
        # Clear dictionary for next frame
        object_positions.clear()
        rate.sleep()

if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass