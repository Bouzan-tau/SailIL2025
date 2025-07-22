import serial
import time

# Adjust this to match your actual Arduino port
SERIAL_PORT = '/dev/ttyUSB1'
BAUD_RATE = 9600

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
          # wait for Arduino reset
        time.sleep(1)
        # Send test command
        command = b'l50,r30,f10\n'
        print(f"➡ Sending: {command.strip()}")
        ser.write(command)
        #time.sleep(10)
        #ser.flush()

        print("🔄 Reading responses (Ctrl+C to stop)...")
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"⬅ Received: {line}")

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    main()