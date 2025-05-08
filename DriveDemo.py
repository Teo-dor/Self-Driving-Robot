import RPi.GPIO as GPIO
import time
import termios
import tty
import sys
import threading

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

def get_key():
    """Non-blocking key read from terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

action_map = {'w': forward, 'a': left_turn, 'd': right_turn, 's': reverse}

init()
try:
    print("Press W/A/S/D to move. Hold the key to keep moving. Release the key to stop. Press Q to quit.")
    current_action = None
    stop_thread = False

    def stop_robot():
        """Stop the robot when no key is pressed."""
        global stop_thread, current_action
        while not stop_thread:
            if current_action is None:
                stop()
            time.sleep(0.1)

    # Start a thread to stop the robot when no key is pressed
    stop_thread = False
    thread = threading.Thread(target=stop_robot, daemon=True)
    thread.start()

    while True:
        key = get_key().lower()
        if key == 'q':
            print("Exiting...")
            break
        elif key in action_map:
            current_action = action_map[key]
            current_action()
        else:
            current_action = None  # Reset current_action when no valid key is pressed
except KeyboardInterrupt:
    print("\nStopped by Ctrl+C.")
finally:
    stop_thread = True
    thread.join()
    GPIO.cleanup()
