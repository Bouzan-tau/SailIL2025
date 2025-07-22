#!/usr/bin/env python3

import serial
import time

def stop_engines():
    try:
        # Open serial connection to Arduino
        ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=2)  # Adjust if needed
        time.sleep(2)  # Give Arduino time to reset

        # Flush any garbage
        ser.flush()

        # Send stop command
        command = "l0,r0,f0\n"
        print(f"Sending stop command: {command.strip()}")
        ser.write(command.encode('utf-8'))
        ser.flush()
        time.sleep(0.5)

        ser.close()
        print("✔️ Engines stopped and serial closed.")
    except Exception as e:
        print(f"❌ Failed to send stop command: {e}")

if __name__ == "__main__":
    stop_engines()
