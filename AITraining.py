import csv
import time
import RPi.GPIO as GPIO
from pynput import keyboard
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
lidar = RPLidar(PORT_NAME)

def save_lidar_scan(scan, action):
    """Append a Lidar scan with a labeled action to the CSV file."""
    with open(CSV_FILE, mode="a", newline="") as file:
        writer = csv.writer(file)
        scan_data = [(angle, distance) for quality, angle, distance in scan if quality > 0 and distance > 0]
        writer.writerow([json.dumps(scan_data), action])

action = None

def on_press(key):
    global action
    try:
        if key.char == 'w':
            action = 0  # Forward
        elif key.char == 'a':
            action = 1  # Left
        elif key.char == 'd':
            action = 2  # Right
        elif key.char == 's':
            action = 3  # Brake (Stop)
    except AttributeError:
        pass

def run():
    """Main loop for collecting labeled Lidar data."""
    global action
    try:   
        GPIO.output(motorPin, GPIO.HIGH)
        time.sleep(0.5) # Wait for the motor to reach higher speed
        recordedScan = 0
        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        while True:
            GPIO.output(motorPin, GPIO.HIGH)
            time.sleep(0.5) # Wait for the motor to reach higher speed
            print("\n--- Press Ctrl + c to capture a scan")
            try:
                for scan in lidar.iter_scans():
                    recordedScan = scan
            except KeyboardInterrupt:
                GPIO.output(motorPin, GPIO.LOW)
                lidar.stop()
                for measurement in recordedScan:
                    quality, angle, distance = measurement
                    print(f"Quality: {quality}, Angle: {angle:.2f}, Distance: {distance:.2f}")
                print("\nPress 'W' for Forward, 'A' for Left, 'D' for Right, 'S' for Brake or CTRL + C to stop:")
                action = None
                while action is None:
                    time.sleep(0.1)
                print(f"Action selected: {action}")
                save_lidar_scan(recordedScan, action)
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
