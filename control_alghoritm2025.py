import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import math
import time
import threading

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)
        self.gps = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
        
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
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        
        # Subscriber to "cmd_vel" topic
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Subscriber to "steering_directions" topic
        self.create_subscription(String, 'steering_directions', self.steering_callback, 10)
        
        # Start running the state machine logic
        self.situation_loop()

        # Start a thread to read GPS data
        self.gps_thread = threading.Thread(target=self.read_gps_data)
        self.gps_thread.start()

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

    def steering_callback(self, msg):
        angle = int(msg.data)
        self.get_logger().info(f"Received steering angle: {angle}")
        self.control_thrusters(angle)

    def control_thrusters(self, angle):
        if angle == 0:
            # Move forward
            self.move(self.right_thruster, 150)
            self.move(self.left_thruster, 150)
            self.move(self.bow_thruster, 90)
        elif angle == 45:
            # Turn right
            self.move(self.right_thruster, 90)
            self.move(self.left_thruster, 150)
            self.move(self.bow_thruster, 70)
        elif angle == 315:
            # Turn left
            self.move(self.right_thruster, 150)
            self.move(self.left_thruster, 90)
            self.move(self.bow_thruster, 110)
        else:
            # Custom angle handling
            self.move(self.right_thruster, 90 + angle)
            self.move(self.left_thruster, 90 + angle)
            self.move(self.bow_thruster, 90 + angle)

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

    def read_gps_data(self):
        while rclpy.ok():
            try:
                line = self.gps.readline().decode('ascii', errors='replace')
                if line.startswith('$GNGGA'):
                    data = line.split(',')
                    if len(data) > 9:
                        self.latitude = self.convert_to_degrees(data[2], data[3])
                        self.longitude = self.convert_to_degrees(data[4], data[5])
                        self.altitude = float(data[9])
                        self.get_logger().info(f"Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.altitude}")
            except Exception as e:
                self.get_logger().error(f"Error reading GPS data: {e}")

    def convert_to_degrees(self, raw_value, direction):
        if raw_value == '':
            return 0.0
        degrees = float(raw_value[:2])
        minutes = float(raw_value[2:])
        result = degrees + (minutes / 60.0)
        if direction in ['S', 'W']:
            result = -result
        return result

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
