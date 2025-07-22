#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time 
class EngineController(Node):
    def __init__(self):
        super().__init__('engine_controller')
        self.subscription = self.create_subscription(
            String,
            'steering_directions',
            self.listener_callback,
            10)
        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=10)  # Adjust port as needed
        #time.sleep(2)
        self.ser.flush()
        # Wait for Arduino to be ready
        timeout = time.time() + 5  # 5-second timeout
        while True:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if "Send command" in line:
                    break
            if time.time() > timeout:
                print("⚠️ Timeout waiting for Arduino")
                break
        self.get_logger().info('Arduino ready')
        # Send initial stop command
        #self.send_command(0, 0, 0)

    def listener_callback(self, msg):
        print(f"Recieved:{msg.data}")
        try:
            angle = float(msg.data)
        except ValueError:
            self.get_logger().error(f'Invalid angle received: {msg.data}')
            return

        # Convert angle to steering_angle in [-180, 180]
        if angle > 180:
            steering_angle = angle - 360
        else:
            steering_angle = angle

        # Define base power and steering gain
        base_power = 50  # Base speed for left and right engines
        steering_gain = 100 / 45  # Max power difference at 45°

        # Calculate turn_factor
        turn_factor = steering_gain * steering_angle

        # Calculate engine powers
        left_power = base_power + turn_factor
        right_power = base_power - turn_factor
        forward_power = turn_factor  # Lateral thruster assists turn

        # Clip powers to [-100, 100]
        left_power = max(-100, min(100, left_power))
        right_power = max(-100, min(100, right_power))
        forward_power = max(-100, min(100, forward_power))
        print(f"calculated power:{left_power}{right_power}{forward_power}")

        self.send_command(left_power, right_power, forward_power)

    def send_command(self, left, right, forward):
        # Format command as per Arduino expectation
        command = f"l{left},r{right},f{forward} \n"
        print(f"send:{command}")
        self.ser.write(command.encode('utf-8'))
        self.ser.flush()
        self.get_logger().info(f'Sent: {command.strip()}')

    def __del__(self):
        # Ensure serial port is closed on shutdown
        if self.ser.is_open:
            self.send_command(b'l0,r0,f0')  # Stop engines
            self.ser.close()
            self.get_logger().info('Serial connection closed')

def main(args=None):
    print("engines script started")
    rclpy.init(args=args)
    engine_controller = EngineController()
    engine_controller.send_command(50,50,0)
    try:
        rclpy.spin(engine_controller)
        print(rclpy.spin(engine_controller))
    except KeyboardInterrupt:
        pass
    finally:
        #engine_controller.send_command(0,0,0)
        engine_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
