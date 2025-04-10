import csv
import time
import RPi.GPIO as GPIO
from rplidar import RPLidar, RPLidarException
import json

# Set the correct serial port for your Lidar
PORT_NAME = '/dev/ttyUSB0'  # Update based on your system
motorPin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)
# File to save Lidar data
CSV_FILE = "lidar_training_data.csv"

# Initialize the Lidar
lidar = RPLidar(port=PORT_NAME, baudrate=115200, timeout=1)

def save_lidar_scan(scan, action):
    """Append a Lidar scan with a labeled action to the CSV file."""
    with open(CSV_FILE, mode="a", newline="") as file:
        writer = csv.writer(file)
        scan_data = [(angle, distance) for quality, angle, distance in scan if quality > 0 and distance > 0]
        writer.writerow([json.dumps(scan_data), action])

def run():
    """Main loop for collecting labeled Lidar data."""
    try:   
        GPIO.output(motorPin, GPIO.HIGH)
        time.sleep(0.5)  # Wait for the motor to reach higher speed
        print("\n--- Press Ctrl + C to stop the script ---")
        print("Waiting for a Lidar scan...")
        recordedScan = None
        try:
            for scan in lidar.iter_scans():
                recordedScan = scan
        except KeyboardInterrupt:
            GPIO.output(motorPin, GPIO.LOW)
            for measurement in recordedScan:
                quality, angle, distance = measurement
                print(f"Quality: {quality}, Angle: {angle:.2f}, Distance: {distance:.2f}")

            print("\nEnter 'W' for Forward, 'A' for Left, 'D' for Right, 'S' for Brake, or 'Q' to quit:")
            action = None
            while action not in ['w', 'a', 'd', 's', 'q']:
                action = input("Your action: ").strip().lower()

            if action == 'q':
                print("Exiting...")

            action_map = {'w': 0, 'a': 1, 'd': 2, 's': 3}
            print(f"Action selected: {action_map[action]}")
            save_lidar_scan(recordedScan, action_map[action])
            print("Saved!\n")

    except KeyboardInterrupt:
        print("\nStopping. Data saved.")
    except RPLidarException as e:
        print(f"RPLidar Error: {e}")
    finally:
        GPIO.output(motorPin, GPIO.LOW)
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    run()
