from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA2 as LaserModel
from rplidar import RPLidar as Lidar, RPLidarException
from roboviz import MapVisualizer
import RPi.GPIO as GPIO
import time

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 10
LIDAR_DEVICE            = '/dev/ttyUSB0'

motorPin = 11
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(motorPin, GPIO.OUT, initial=GPIO.LOW)

# Ideally we could use all 250 or so samples that the RPLidar delivers in one 
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 200    

lidar = Lidar(LIDAR_DEVICE)
try:

    if __name__ == '__main__':

        # Connect to Lidar unit
        
        # Create an RMHC SLAM object with a laser model and optional robot model
        slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

        # Set up a SLAM display
        viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')

        # Initialize an empty trajectory
        trajectory = []

        # Initialize empty map
        mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

        # Create an iterator to collect scan data from the RPLidar
        GPIO.output(motorPin, GPIO.HIGH)
        time.sleep(0.5) # Wait for the motor to reach higher speed
        iterator = lidar.iter_scans()

        # We will use these to store previous scan in case current scan is inadequate
        previous_distances = None
        previous_angles    = None

        # First scan is crap, so ignore it
        next(iterator)
        
        while True:
            # Extract (quality, angle, distance) triples from current scan
            items = [item for item in next(iterator)]
            # Extract distances and angles from triples
            distances = [item[2] for item in items]
            angles    = [item[1] for item in items]

            # Update SLAM with current Lidar scan and scan angles if adequate
            if len(distances) > MIN_SAMPLES:
                slam.update(distances, scan_angles_degrees=angles)
                previous_distances = distances.copy()
                previous_angles    = angles.copy()

            # If not adequate, use previous
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=previous_angles)

            # Get current robot position
            x, y, theta = slam.getpos()

            # Get current map bytes as grayscale
            slam.getmap(mapbytes)

            # Display map and robot pose, exiting gracefully if user closes it
            if not viz.display(x/1000., y/1000., theta, mapbytes):
                exit(0)
except Exception as e:
    print("Error")
    print(e)
finally:
    # Shut down the lidar connection
    GPIO.output(motorPin, GPIO.LOW)
    lidar.stop()
    lidar.disconnect()