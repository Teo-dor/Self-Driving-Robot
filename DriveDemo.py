import RPi.GPIO as gpio #lgpio library
import time
#Change to pins you are using
R1 = 22
R2 = 16
L1 = 18
L2 = 13
def init():    
    gpio.setmode(gpio.BOARD)
    gpio.setup(R1, gpio.OUT)
    gpio.setup(R2, gpio.OUT)
    gpio.setup(L1, gpio.OUT)
    gpio.setup(L2, gpio.OUT)

def forward(sec):
    init()
    gpio.output(R1, False)
    gpio.output(R2, True)
    gpio.output(L1, True)
    gpio.output(L2, False)
    time.sleep(sec)
    gpio.cleanup() 
def reverse(sec):
    init()
    gpio.output(R1, True)
    gpio.output(R2, False)
    gpio.output(L1, False)
    gpio.output(L2, True)
    time.sleep(sec)
    gpio.cleanup()
def right_turn(sec):
    init()
    gpio.output(R1, True)
    gpio.output(R2, False)
    gpio.output(L1, True)
    gpio.output(L2, False)
    time.sleep(sec)
    gpio.cleanup()
def left_turn(sec):
    init()
    gpio.output(R1, False)
    gpio.output(R2, True)
    gpio.output(L1, False)
    gpio.output(L2, True)
    time.sleep(sec)
    gpio.cleanup()

seconds = 2
time.sleep(seconds)
print("forward")
forward(seconds)
time.sleep(seconds-1)

print("right")
right_turn(seconds)
time.sleep(seconds-1)

print("forward")
forward(seconds)
time.sleep(seconds-1)

print("left")
left_turn(seconds)
time.sleep(seconds-1)

print("reverse")
reverse(seconds)
