import numpy as np
import torch
import torch.nn as nn
import time
import threading
import RPi.GPIO as GPIO
from rplidar import RPLidar, RPLidarException

# GPIO motor pins
R1, R2, L1, L2 = 22, 16, 18, 13

# =========================
# Define the Neural Network
# =========================
class RobotNN(nn.Module):
    def __init__(self, input_size, hidden_size=256, output_size=4):  # Match NN.py
        super(RobotNN, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.dropout1 = nn.Dropout(p=0.3)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.dropout2 = nn.Dropout(p=0.3)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.dropout3 = nn.Dropout(p=0.3)
        self.fc4 = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        x = self.fc1(x)
        x = self.relu(x)
        x = self.dropout1(x)
        x = self.fc2(x)
        x = self.relu(x)
        x = self.dropout2(x)
        x = self.fc3(x)
        x = self.relu(x)
        x = self.dropout3(x)
        x = self.fc4(x)
        return x

# Initialize GPIO
def init():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R1, GPIO.OUT)
    GPIO.setup(R2, GPIO.OUT)
    GPIO.setup(L1, GPIO.OUT)
    GPIO.setup(L2, GPIO.OUT)

def stop():
    GPIO.output(R1, False)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, False)

def forward(sec):
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
    time.sleep(sec)
    stop()

def reverse(sec):
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    time.sleep(sec)
    stop()

def right_turn(sec):
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
    time.sleep(sec)
    stop()

def left_turn(sec):
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    time.sleep(sec)
    stop()

# Load the trained model
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Dynamically determine the input size from the saved model
input_size = 720  # Set this to match the input size used during training
model = RobotNN(input_size=input_size).to(device)  # Initialize with correct input size
model.load_state_dict(torch.load("robot_model.pth", map_location=device))  # Load weights
model.eval()

# Lidar setup
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(port=PORT_NAME, baudrate=115200, timeout=1)
latest_scan = []
scan_lock = threading.Lock()
stop_event = threading.Event()

def preprocess_lidar_scan(scan, max_length=360):
    """
    Preprocesses a single Lidar scan for the neural network.
    - Flatten the [angle, distance] pairs.
    - Pad or truncate to ensure consistent length.
    - Normalize distances.
    """
    flattened_scan = [item for pair in scan for item in pair]
    processed_scan = flattened_scan[:max_length * 2] + [0] * (max_length * 2 - len(flattened_scan))
    processed_scan = np.array(processed_scan)
    if np.max(processed_scan[1::2]) > 0:  # Avoid division by zero
        processed_scan[1::2] = (processed_scan[1::2] / np.max(processed_scan[1::2])) * 2 - 1  # Normalize distances
    return torch.tensor(processed_scan, dtype=torch.float32).unsqueeze(0).to(device)  # Add batch dimension

def scan_thread():
    """
    Continuously collects Lidar scans in a separate thread.
    """
    global latest_scan
    try:
        for scan in lidar.iter_scans():
            if stop_event.is_set():
                break
            with scan_lock:
                latest_scan = [(angle, distance) for quality, angle, distance in scan if quality > 0 and distance > 0]
    except RPLidarException as e:
        print(f"Lidar error: {e}")
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        GPIO.output(11, GPIO.LOW)

def execute_action(action):
    """
    Executes the robot's movement based on the predicted action.
    - 0: Forward
    - 1: Left
    - 2: Right
    - 3: Stop
    """
    if action == 0:
        forward(0.1)  # Move forward for 0.1 seconds
    elif action == 1:
        left_turn(0.05)  # Turn left for 0.05 seconds
    elif action == 2:
        right_turn(0.05)  # Turn right for 0.05 seconds
    elif action == 3:
        stop()  # Stop the robot

def control_robot():
    """
    Main control loop for the robot.
    Continuously collects Lidar data, makes predictions, and executes actions.
    """
    try:
        while not stop_event.is_set():
            # Get the latest Lidar scan
            with scan_lock:
                scan_copy = latest_scan.copy()

            if not scan_copy:
                continue  # Skip if no scan data is available

            # Preprocess the Lidar data
            input_data = preprocess_lidar_scan(scan_copy)

            # Predict the action
            with torch.no_grad():
                output = model(input_data)
                predicted_action = torch.argmax(output, dim=1).item()

            # Execute the action
            execute_action(predicted_action)

    except KeyboardInterrupt:
        print("Stopping robot control.")
        stop_event.set()  # Signal the scanning thread to stop


if __name__ == "__main__":
    try:
        init()
        GPIO.setup(11, GPIO.OUT)  # Lidar motor control pin
        GPIO.output(11, GPIO.HIGH)  # Ensure Lidar motor is powered on
        time.sleep(0.5)

        # Start the Lidar scanning thread
        thread = threading.Thread(target=scan_thread, daemon=True)
        thread.start()

        # Start the robot control loop
        control_robot()

    except Exception as e:
        print(f"An error occurred: {e}")
        stop_event.set()  # Ensure the scanning thread stops
    finally:
        # Cleanup resources
        if thread.is_alive():
            thread.join()
        if lidar._serial_port.is_open:
            lidar.stop()
            lidar.disconnect()
        GPIO.output(11, GPIO.LOW)
        GPIO.cleanup()
