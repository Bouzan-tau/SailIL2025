#!/usr/bin/env python

import rospy
import math
import sys
from std_msgs.msg import Float32

# Constants
P_BASE = 50.0       # Base power for left and right engines (%)
P_CENTER = 100.0    # Constant power for center engine (%)
K = 0.5             # Proportional gain for steering adjustment

def calculate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate the bearing between two GPS coordinates in degrees.
    """
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    d_lon = lon2_rad - lon1_rad

    y = math.sin(d_lon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(d_lon)
    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)
    bearing_deg = (bearing_deg + 360) % 360
    return bearing_deg

def calculate_engine_powers(error):
    """
    Calculate power levels for each engine based on the angle error.
    """
    delta_P = K * error
    P_left = P_BASE - delta_P
    P_right = P_BASE + delta_P

    P_left = max(0.0, min(100.0, P_left))
    P_right = max(0.0, min(100.0, P_right))
    P_center = P_CENTER

    return P_left, P_right, P_center

def navigate_boat(start_lat, start_lon, end_lat, end_lon):
    """
    Calculate the route and publish proportional power commands for each engine.
    """
    rospy.init_node('boat_navigation', anonymous=True)
    
    # Publishers for each engine
    left_pub = rospy.Publisher('left_engine_power', Float32, queue_size=10)
    right_pub = rospy.Publisher('right_engine_power', Float32, queue_size=10)
    center_pub = rospy.Publisher('center_engine_power', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Calculate desired bearing
    desired_bearing = calculate_bearing(start_lat, start_lon, end_lat, end_lon)
    rospy.loginfo(f"Calculated bearing to destination: {desired_bearing} degrees")

    # Assume initial heading is 0 degrees (north)
    current_heading = 0.0
    error = (desired_bearing - current_heading + 180) % 360 - 180  # Error in [-180, 180]

    # Calculate and publish engine powers
    P_left, P_right, P_center = calculate_engine_powers(error)
    left_pub.publish(P_left)
    right_pub.publish(P_right)
    center_pub.publish(P_center)

    rospy.loginfo(f"Engine powers - Left: {P_left}%, Right: {P_right}%, Center: {P_center}%")

    # Keep node alive briefly to ensure messages are sent
    rospy.sleep(1)
    rospy.loginfo("Engine power commands sent. Navigation complete.")

if __name__ == '__main__':
    if len(sys.argv) != 5:
        print("Usage: python boat_navigation.py <start_lat> <start_lon> <end_lat> <end_lon>")
        sys.exit(1)

    try:
        start_lat = float(sys.argv[1])
        start_lon = float(sys.argv[2])
        end_lat = float(sys.argv[3])
        end_lon = float(sys.argv[4])
        navigate_boat(start_lat, start_lon, end_lat, end_lon)
    except ValueError:
        print("Error: Coordinates must be numbers.")
        sys.exit(1)
    except rospy.ROSInterruptException:
        pass
