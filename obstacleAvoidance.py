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

obstacleCooldown = 1/10 #Dont get stuck next to an obstacle, 1s = about 3 quarters of a lap
obstacleTime = 0
SAFE_DISTANCE = 200  # Example threshold distance in mm

R1, R2, L1, L2 = 22, 16, 15, 13
ENB = 32

right_pwm = 80
def init():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(R1, GPIO.OUT)
    GPIO.setup(R2, GPIO.OUT)
    GPIO.setup(L1, GPIO.OUT)
    GPIO.setup(L2, GPIO.OUT)
    GPIO.setup(ENB, GPIO.OUT)
    pwm = GPIO.PWM(ENB, 1000)
    pwm.start(0)

def stop():
    GPIO.output(R1, False)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, False)
    pwm.ChangeDutyCycle(0) 

def forward():
    GPIO.output(R1, False)
    GPIO.output(L2, False)
    GPIO.output(R2, True)
    GPIO.output(L1, True)
    pwm.ChangeDutyCycle(right_pwm)  

def reverse():
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    pwm.ChangeDutyCycle(100)

def right_turn():
    GPIO.output(R1, True)
    GPIO.output(R2, False)
    GPIO.output(L1, True)
    GPIO.output(L2, False)
    pwm.ChangeDutyCycle(right_pwm) 

def left_turn():
    GPIO.output(R1, False)
    GPIO.output(R2, True)
    GPIO.output(L1, False)
    GPIO.output(L2, True)
    pwm.ChangeDutyCycle(right_pwm) 

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
                            if angle > 60 and angle < 120 and (time.time() - obstacleTime) > obstacleCooldown: #Something ahead, turn away
                                obstacleTime = time.time()
                                if angle < 90:
                                    right_turn()
                                    print("Obstacle to the left")
                                else:
                                    left_turn()
                                    print("Obstacle to the right")
                        elif (time.time() - obstacleTime) > obstacleCooldown: #otherwise, go forward
                            forward()
    except KeyboardInterrupt:
        print('Stopping.')
    except Exception as e:
        print("Error")
        print(e)
    GPIO.output(motorPin, GPIO.LOW)
    lidar.stop()
    lidar.disconnect() 
    stop()
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


init()
run()
