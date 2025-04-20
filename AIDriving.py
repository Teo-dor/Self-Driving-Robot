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
    def __init__(self, input_size, hidden_size=1024, output_size=3):  # Match NN.py
        super(RobotNN, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.relu = nn.ReLU()
        self.dropout1 = nn.Dropout(p=0.4)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.dropout2 = nn.Dropout(p=0.4)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.dropout3 = nn.Dropout(p=0.4)
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

input_size = 360 # One neuron per angle
model = RobotNN(input_size=input_size).to(device)  # Initialize with correct input size
model.load_state_dict(torch.load("best_model.pth", map_location=device))  # Load weights
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
    - Converts [angle, distance] pairs into a 360-item array.
    - Normalizes distances to the range [-1, 1].
    """
    distances = [0] * max_length  # Initialize array with zeros
    for angle, distance in scan:
        rounded_angle = round(angle)  # Round angle to the nearest integer
        if 0 <= rounded_angle < max_length:  # Ensure angle is within bounds
            distances[rounded_angle] = distance

    # Normalize distances (scale between -1 and 1)
    distances = np.array(distances)
    if np.max(distances) > 0:  # Avoid division by zero
        distances = (distances / np.max(distances)) * 2 - 1

    return torch.tensor(distances, dtype=torch.float32).unsqueeze(0).to(device)  # Add batch dimension

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
    """
    if action == 0:
        forward(0.2)  # Move forward for 0.1 seconds
    elif action == 1:
        left_turn(0.1)  # Turn left for 0.1 seconds
    elif action == 2:
        right_turn(0.1)  # Turn right for 0.1 seconds
    

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
                probabilities = torch.softmax(output, dim=1).cpu().numpy()[0]  # Convert to probabilities
                predicted_action = torch.argmax(output, dim=1).item()

            # Debugging: Print the probabilities and chosen action
            print(f"Probabilities: {probabilities}, Chosen Action: {predicted_action}")

            execute_action(predicted_action)

    except KeyboardInterrupt:
        print("Stopping robot control.")
        stop_event.set()  # Signal the scanning thread to stop


if __name__ == "__main__":
    try:
        init()
        GPIO.setup(11, GPIO.OUT)  # Lidar motor control pin
        GPIO.output(11, GPIO.HIGH)  
        time.sleep(0.5)

        thread = threading.Thread(target=scan_thread, daemon=True)
        thread.start()

        control_robot()

    except Exception as e:
        print(f"An error occurred: {e}")
        stop_event.set()  
    finally:
        # Cleanup resources
        if thread.is_alive():
            thread.join()
        if lidar._serial_port.is_open:
            lidar.stop()
            lidar.disconnect()
        GPIO.output(11, GPIO.LOW)
        GPIO.cleanup()
