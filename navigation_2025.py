#!/usr/bin/env python

import rospy
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
    def __init__(self): # Initialize the timer object and a flag to track if the timer is running
        self.timer = None
        self.timer_running = False

    def start_timer(self, duration): # Start the timer if it's not already running
        if not self.timer_running:
            self.timer = rospy.Timer(rospy.Duration(duration), timer_callback) # Create a ROS timer with the given duration
            self.timer_running = True # Update the flag to indicate the timer is running

    def stop_timer(self):  # Start the timer if it's not already running
        if self.timer_running:
            self.timer.shutdown() # Shut down the ROS timer
            self.timer_running = False # Update the flag to indicate the timer is stopped

def position_callback(msg): # Store the detected object's position (x, y, z) under its label in object_positions
    object_positions[msg.label].append((
        msg.distance_x,  # Horizontal
        msg.distance_y,  # Vertical
        msg.distance_z,  # Depth
    ))
 
def timer_callback(event): # Shutdown the ROS node if no objects are detected within the timeout
    rospy.signal_shutdown("No objects detected")
    rospy.loginfo("Finish")

def find_closest(obj_list): # Initialize variables to track the closest object's distance and details
    closest_distance = float('inf') # Start with infinity as the furthest possible distance
    nearest_obj = None # No nearest object found yet
    for obj in obj_list:  # Loop through each object in the list
        if obj[2] < DISTANCE_THRESHOLD and obj[2] < closest_distance:  # Check if the object's depth (z-coordinate) is within the threshold and closer than the current closest
            closest_distance = obj[2] # Update the closest distance
            nearest_obj = obj # Update the closest object
    return nearest_obj

def get_angle(p1, p2=None):
    
    if p2:                        # If two points are provided, calculate the midpoint between them
        middle_point = (
            (p1[0] + p2[0]) / 2,  # Average of x-coordinates (horizontal)
            (p1[1] + p2[1]) / 2,  # Average of y-coordinates (vertical, unused here)
            (p1[2] + p2[2]) / 2   # Average of z-coordinates (depth/forward)
        )
    else:
        middle_point = p1         # If only one point is provided, use it directly

    # Calculate the angle between the positive Z-axis (forward direction) and the point in the XZ plane
    # atan2(x, z) ensures the angle is correct for all quadrants
    angle = round(degrees(atan2(middle_point[0], middle_point[2])))

    if angle < 0:   # Adjust negative angles to the range [0, 360]
        angle += 360

    return angle

def get_distance(p1, p2):
    if p1 == None and p2 != None:
        return p2
    elif p2 == None and p1 != None:
        return p1
    return sqrt(((p1[0] - p2[0]) ** 2 + (p1[2] - p2[2]) ** 2))


def get_bounds():
    nearest_red = find_closest(object_positions['Red ball'])  # Closest red ball
    nearest_green = find_closest(object_positions['Green ball'])  # Closest green ball
    nearest_yellow = find_closest(object_positions['Yellow ball'])  # Closest yellow ball
    
    if not nearest_yellow or nearest_red == None or nearest_green == None:  # Case 1: If no yellow ball is detected, return the red and green balls as bounds
        return nearest_red, nearest_green

    else: # Case 2: If a yellow ball is detected, determine its position relative to the red and green balls
        # Check if the yellow ball lies between the red and green balls
        if (nearest_red[0] < nearest_yellow[0]) and (nearest_green[0] > nearest_yellow[0]):
            # Compare distances between yellow-green and red-yellow to find the tighter bound
            if get_distance(nearest_yellow, nearest_green) > get_distance(nearest_red, nearest_yellow):
                return nearest_yellow, nearest_green
            return nearest_red, nearest_yellow 
        else: # If the yellow ball does not lie between the red and green balls, return red and green as bounds
            return nearest_red, nearest_green

def publish_angle(steering_pub, angle):
    steering_pub.publish(f"{angle}")    

def navigate():
    rospy.init_node('navigation_node', anonymous=True) # Initialize the ROS node for navigation
    
    # Subscribe to the 'object_distance_info' topic to receive data about detected objects
    # The messages are of type ObjectDistanceInfo, and the position_callback function processes the incoming data
    rospy.Subscriber('object_distance_info', ObjectDistanceInfo, position_callback)
    
    # Create a publisher for steering direction commands
    # - Publishes messages of type String on the 'steering_directions' topic
    # - The messages represent steering angles or directions (e.g., "0", "45", "315")
    # - queue_size=10 ensures that up to 10 messages can be queued if the subscriber is not ready to process them immediately
    steering_pub = rospy.Publisher('steering_directions', String, queue_size=10)
    
    # Initialize an instance of the TimerManager class to handle timeout functionality
    # This will be used to track if objects are not detected within a specific time frame
    timer = TimerManager()

    # Set the loop frequency for the navigation process
    # FREQ defines how many iterations per second the loop will execute, ensuring consistent operation
    rate = rospy.Rate(FREQ)

    # Initialize a variable to track the last seen object type
    # This helps decide the boat's fallback direction if no objects are detected in the current cycle
    last_seen = None


    
    while not rospy.is_shutdown(): # Main loop runs until the ROS node is shut down
        # Check if the 'end' marker is present in object_positions.
        # The 'end' marker signals that all objects in the current frame have been processed by the publisher
        # If 'end' is not detected, wait for the next iteration to ensure complete object data is available
        if not object_positions['end']:
            rate.sleep()  # Maintain loop frequency.
            continue  # Skip to the next iteration.


        left_bound, right_bound = get_bounds()
        
        # Case 1: Both left and right bounds are detected.
        # Calculate the steering angle based on the midpoint between the bounds.
        # Publish the angle to adjust the boat's direction, then stop the timeout timer.
        if left_bound and right_bound:
            angle = get_angle(left_bound, right_bound)
            publish_angle(steering_pub, angle)
            timer.stop_timer()
            
        # Case 2: Only the left bound is detected
        # Calculate the steering angle based on the left bound's position
        # If the angle is outside the range for forward movement, turn right
        # Otherwise, move forward. Stop the timeout timer after making a decision

        elif left_bound:
            angle = get_angle(left_bound)
            if angle < 90 or angle > 345:
                publish_angle(steering_pub, ANGLE_RIGHT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
            
        # Case 3: Only the right bound is detected
        # Calculate the steering angle based on the right bound's position
        # If the angle is outside the range for forward movement, turn left
        # Otherwise, move forward. Stop the timeout timer after making a decision

        elif right_bound:
            angle = get_angle(right_bound)
            if angle > 270 or angle < 15:
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
            timer.stop_timer()
            
        # Case 4: No bounds are detected
        # Start the timeout timer as no valid navigation data is available
        # Use the last seen object to determine the fallback steering direction:
        # - Turn right if the last seen object was red
        # - Turn left if the last seen object was green
        # - Move forward if no objects were recently detected

        else:
            timer.start_timer(TIMEOUT)
            if last_seen == 'r':
                publish_angle(steering_pub, ANGLE_RIGHT)
            elif last_seen == 'g':
                publish_angle(steering_pub, ANGLE_LEFT)
            else:
                publish_angle(steering_pub, ANGLE_FORWARD)
                
                
        # Update the last_seen variable to track the most recent bounds:
        # - 'r&g' if both bounds were detected.
        # - 'r' if only the left bound (red) was detected.
        # - 'g' if only the right bound (green) was detected.
        if left_bound and right_bound:
            last_seen = 'r&g'
        elif left_bound:
            last_seen = 'r'
        elif right_bound:
            last_seen = 'g'

        
        object_positions.clear()  # Clear object_positions to remove outdated data and prepare for the next frame
        rate.sleep() # Pause the loop to maintain the defined frequency (FREQ) for consistent operation


if __name__ == '__main__':
    try:
        navigate()
    except rospy.ROSInterruptException:
        pass
