#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math
import time

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')  # Node name
        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)  # Communication with Arduino
        
        # Thrusters and servo names
        self.right_thruster = "right_thruster"
        self.left_thruster = "left_thruster"
        self.bow_thruster = "bow_thruster"
        self.right_servo = "right_servo" 
        self.left_servo = "left_servo"
        
        # Initial states
        self.x = 0
        self.y = 0
        self.angle = 0
        self.dist = 0
        
        # Subscriber to "cmd_vel" topic
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Start running the state machine logic
        self.situation_loop()

    def cmd_callback(self, cmd_data):
        self.x = cmd_data.linear.x
        self.y = cmd_data.linear.y
        self.angle = math.degrees(cmd_data.angular.z)
        self.dist = math.sqrt(self.x**2 + self.y**2)
        
        if self.angle == 0:
            if self.y != 0:
                self.angle = math.degrees(math.atan2(self.y, self.x))
            elif self.y > 0:
                self.angle = 90
            elif self.y < 0:
                self.angle = -90

    def move(self, motor, angle):
        motor_id = 0
        if motor == "right_thruster":
            motor_id = 1
        elif motor == "left_thruster":
            motor_id = 2
        elif motor == "bow_thruster":
            motor_id = 3
        
        message = str(angle + 1000 * motor_id)  # Encoding motor message with motor id
        self.get_logger().info(f"Motor: {motor}, Angle: {angle}")
        self.arduino.write(message.encode('utf-8'))
        time.sleep(0.05)  # Small delay to avoid overlapping messages

    def situation_loop(self):
        while rclpy.ok():
            if abs(self.dist) <= 0.1 and abs(self.angle) <= 0.1:
                self.move(self.right_thruster, 90)
                self.move(self.left_thruster, 90)
                self.move(self.bow_thruster, 90)
                time.sleep(0.1)

            elif abs(self.dist) <= 0.1 and abs(self.angle) > 0.1:
                if self.angle > 0:
                    self.move(self.right_thruster, 90)
                    self.move(self.left_thruster, 90)
                    self.move(self.bow_thruster, 70)
                elif self.angle < 0:
                    self.move(self.right_thruster, 90)
                    self.move(self.left_thruster, 90)
                    self.move(self.bow_thruster, 110)
                else:
                    self.move(self.right_thruster, 90)
                    self.move(self.left_thruster, 90)
                    self.move(self.bow_thruster, 90)
                time.sleep(0.1)

            if 0.1 < self.dist <= 6:
                if 0 < abs(self.angle) <= 90:  # Forward
                    if self.angle > 0:
                        self.move(self.bow_thruster, 110)
                        self.move(self.right_thruster, (90 + (self.dist * 15) / 2))
                        self.move(self.left_thruster, (90 + (self.dist * 15)) / 2)
                    elif self.angle < 0:
                        self.move(self.bow_thruster, 70)
                        self.move(self.right_thruster, (90 + (self.dist * 15) / 2))
                        self.move(self.left_thruster, (90 + (self.dist * 15)) / 2)
                elif self.angle == 0:
                    self.move(self.right_thruster, (90 + (self.dist * 15) / 2))
                    self.move(self.left_thruster, (90 + (self.dist * 15)) / 2)
                    self.move(self.bow_thruster, 90)
                elif 90 < abs(self.angle):  # Backward
                    if self.angle > 0:
                        self.move(self.bow_thruster, 110)
                        self.move(self.right_thruster, 90)
                        self.move(self.left_thruster, 90)
                    elif self.angle < 0:
                        self.move(self.bow_thruster, 70)
                        self.move(self.right_thruster, 90)
                        self.move(self.left_thruster, 90)

            if self.dist > 6:
                if 0 < abs(self.angle) <= 90:
                    if self.angle < 0:
                        self.move(self.bow_thruster, 80)
                        self.move(self.right_thruster, 150)
                        self.move(self.left_thruster, 150)
                    elif self.angle > 0:
                        self.move(self.bow_thruster, 110)
                        self.move(self.right_thruster, 150)
                        self.move(self.left_thruster, 150)
                elif self.angle == 0:
                    self.move(self.right_thruster, 150)
                    self.move(self.left_thruster, 150)
                    self.move(self.bow_thruster, 90)
                elif 90 < abs(self.angle):  # Backward
                    if self.angle > 0:
                        self.move(self.bow_thruster, 110)
                        self.move(self.right_thruster, 90)
                        self.move(self.left_thruster, 90)
                    elif self.angle < 0:
                        self.move(self.bow_thruster, 70)
                        self.move(self.right_thruster, 90)
                        self.move(self.left_thruster, 90)

            time.sleep(0.1)

def main():
    rclpy.init()
    state_machine_node = StateMachine()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
