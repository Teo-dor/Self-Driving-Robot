import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
from rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'
motorPin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)
R1 = 22
R2 = 16
L1 = 18
L2 = 13
obstacleCooldown = 1/6 #Dont get stuck next to an obstacle, 1s = about 3 quarters of a lap
obstacleTime = 0
SAFE_DISTANCE = 200  # Example threshold distance in mm

def init():    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R1, GPIO.OUT)
    GPIO.setup(R2, GPIO.OUT)
    GPIO.setup(L1, GPIO.OUT)
    GPIO.setup(L2, GPIO.OUT)
    GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)
def forward():
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
def reverse():
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
def right_turn():
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
def left_turn():
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
def stopMotor():
    GPIO.output(R1, False)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, False)
lidar_data = []

def run():
    '''Main function'''
    lidar = RPLidar(PORT_NAME)
    try:
        print('Recording measurments... Press Crl+C to stop.')
        GPIO.output(motorPin, GPIO.HIGH)
        time.sleep(0.5) # Wait for the motor to reach higher speed
        obstacleTime = 0
        for scan in lidar.iter_scans():
            for measurement in scan:
                    quality, angle, distance = measurement
                    #print(f"Quality: {quality}, Angle: {angle:.2f}, Distance: {distance:.2f}")
                    if scan[0] != 0:
                        lidar_data.append((angle, distance))
                        if distance < SAFE_DISTANCE:
                            if (angle < 45 or angle > 315) and (time.time() - obstacleTime) > obstacleCooldown: #Something to the left, go right
                                obstacleTime = time.time()
                                right_turn()
                                print("Obstacle to the left")
                            elif angle > 45 and angle < 135 and (time.time() - obstacleTime) > obstacleCooldown: #Something ahead, turn away
                                obstacleTime = time.time()
                                right_turn()
                                print("Obstacle ahead")
                            elif angle > 135 and angle < 225 and (time.time() - obstacleTime) > obstacleCooldown: #Something to the right, go left
                                obstacleTime = time.time()
                                left_turn()
                                print("Obstacle to the right")
                        elif (time.time() - obstacleTime) > obstacleCooldown: #otherwise, go forward
                            forward()
    except KeyboardInterrupt:
        print('Stoping.')
    except Exception as e:
        print("Error")
        print(e)
    GPIO.output(motorPin, GPIO.LOW)
    lidar.stop()
    lidar.disconnect()
    stopMotor()
    GPIO.cleanup()
    

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
        
    #print(f"First few points (X,Y): {list(zip(x_points, y_points))[:100]}")
    print(f"First 100 angles in radians: {list(angles)[:100]}")
    plt.figure(figsize=(1,1))
    plt.scatter(x_points, y_points, s=10, c='red')
    plt.scatter(0, 0, s=30, c='blue')
    plt.axhline(0, color = 'black', linewidth=0.5)
    plt.axvline(0, color = 'black', linewidth=0.5)
    plt.xlabel("X (mm)")
    plt.ylabel("Y (mm)")
    plt.title("Lidar Data Visualization")
    plt.axis('equal')
    plt.grid(True)
    plt.show()


def find_clear_path(lidar_data, safe_distance=SAFE_DISTANCE):
    # Filter directions with safe distances
    clear_paths = [(angle, dist) for angle, dist in lidar_data if dist >= safe_distance]
    if not clear_paths:
        return None  # No clear path found
    return max(clear_paths, key=lambda x: x[1])[0]  # Angle with maximum distance

init()
run()
