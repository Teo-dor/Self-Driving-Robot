import csv
import time
import threading
import RPi.GPIO as GPIO
from rplidar import RPLidar, RPLidarException
import json
import sys
import termios
import tty

# GPIO motor pins
R1, R2, L1, L2 = 22, 16, 18, 13

def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R1, GPIO.OUT)
    GPIO.setup(R2, GPIO.OUT)
    GPIO.setup(L1, GPIO.OUT)
    GPIO.setup(L2, GPIO.OUT)

def stop():
    GPIO.cleanup()

def forward(sec):
    init()
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
    time.sleep(sec)
    stop()

def reverse(sec):
    init()
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    time.sleep(sec)
    stop()

def right_turn(sec):
    init()
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
    time.sleep(sec)
    stop()

def left_turn(sec):
    init()
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    time.sleep(sec)
    stop()

# Lidar Setup
PORT_NAME = '/dev/ttyUSB0'
motorPin = 11
CSV_FILE = "lidar_training_data.csv"

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)

lidar = RPLidar(port=PORT_NAME, baudrate=115200, timeout=1)
latest_scan = []
scan_lock = threading.Lock()

def save_lidar_scan(scan, action):
    with open(CSV_FILE, mode="a", newline="") as file:
        writer = csv.writer(file)
        scan_data = [(angle, distance) for quality, angle, distance in scan if quality > 0 and distance > 0]
        writer.writerow([json.dumps(scan_data), action])

def scan_thread():
    global latest_scan
    try:
        for scan in lidar.iter_scans():
            with scan_lock:
                latest_scan = scan
    except RPLidarException as e:
        print(f"Lidar error: {e}")

def get_key():
    """Non-blocking key read from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def run():
    GPIO.output(motorPin, GPIO.HIGH)
    time.sleep(0.5)

    # Start background Lidar scanning
    thread = threading.Thread(target=scan_thread, daemon=True)
    thread.start()

    print("Lidar is running. Press W/A/S/D to move. Q to quit.")

    action_map = {'w': 0, 'a': 1, 'd': 2, 's': 3}

    try:
        while True:
            key = get_key().lower()
            if key == 'q':
                print("Exiting...")
                break
            elif key in action_map:
                print(f"Action: {key.upper()}")
                with scan_lock:
                    scan_copy = latest_scan.copy()
                save_lidar_scan(scan_copy, action_map[key])
                if key == 'w':
                    forward(0.1)
                elif key == 'a':
                    left_turn(0.1)
                elif key == 'd':
                    right_turn(0.1)
                elif key == 's':
                    reverse(0.1)
                print("Saved scan + action.")
    except KeyboardInterrupt:
        print("\nStopped by Ctrl+C.")
    finally:
        GPIO.output(motorPin, GPIO.LOW)
        lidar.stop()
        lidar.disconnect()
        GPIO.cleanup()

if __name__ == "__main__":
    run()
