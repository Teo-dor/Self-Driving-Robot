import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
from rplidar import RPLidar, RPLidarException
import sys

PORT_NAME = '/dev/ttyUSB0'
motorPin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)
lidar_data = []

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    try:
        print('Recording measurements... Press Ctrl+C to stop.')
        GPIO.output(motorPin, GPIO.HIGH)
        time.sleep(0.5) # Wait for the motor to reach higher speed
        for scan in lidar.iter_scans():
            try:
                for measurement in scan:
                    quality, angle, distance = measurement
                    print(f"Quality: {quality}, Angle: {angle:.2f}, Distance: {distance:.2f}")
                    if quality != 0:
                        lidar_data.append((angle, distance))
            except RPLidarException as e:
                print(f"RPLidarException: {e}")
    except KeyboardInterrupt:
        print('Stopping.')
    except Exception as e:
        print("Error")
        print(e)
    finally:
        GPIO.output(motorPin, GPIO.LOW)
        lidar.stop()
        lidar.disconnect()
        with open(output_file, 'w') as f:
            for angle, distance in lidar_data:
                f.write(f"{angle},{distance}\n")

def plot_lidar_data(lidar_data):
    x_points = []
    y_points = []
    angles = []
    for angle, distance in lidar_data:
        angle_rad = np.radians(angle)
        x = distance * np.cos(angle_rad)
        y = distance * np.sin(angle_rad)
        x_points.append(x*-1)
        y_points.append(y)
        angles.append(angle_rad)

    plt.figure(figsize=(10, 10))
    plt.scatter(x_points, y_points, s=10, c='red')
    plt.scatter(0, 0, s=30, c='blue')
    plt.axhline(0, color='black', linewidth=0.5)
    plt.axvline(0, color='black', linewidth=0.5)
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Lidar Data Visualization")
    plt.axis('equal')
    plt.grid(True)
    plt.show()

def read_lidar_data_from_file(file_path):
    lidar_data = []
    with open(file_path, 'r') as f:
        for line in f:
            angle, distance = map(float, line.strip().split(','))
            lidar_data.append((angle, distance))
    return lidar_data

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python 2DMapping.py <output_file>")
        sys.exit(1)

    output_file = sys.argv[1]

    try:
        print('Ctrl + C to start Lidar')
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('Starting Lidar...')
    #Comment out run if you want to map from data saved in the file.
    run()

    try:
        print('Ctrl + C to start Mapping')
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('Mapping')

    lidar_data = read_lidar_data_from_file(output_file)
    plot_lidar_data(lidar_data)