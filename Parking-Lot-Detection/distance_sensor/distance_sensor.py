
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2

# Set GPIO Pins
GPIO_TRIGGER = 11
GPIO_ECHO = 12

# Set GPIO direction (IN / OUT)
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Function to measure distance
def measure_distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.000001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    print("measure ettim")
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

# Initialize the camera
picam2 = Picamera2()
picam2.start_preview()

try:
    print("deniyorum")
    while True:
        dist = measure_distance()
        print(dist)
        print("Measured Distance = %.1f cm" % dist)
        time.sleep(1)

        # Capture an image if the distance is less than a certain  threshold, e.g., 10 cm
        if dist < 10:
            picam2.capture_file('/home/pi/image.jpg')
            print("Image captured")
            break

# Reset by pressing CTRL + C
except KeyboardInterrupt:
    print("Measurement stopped by User")
    GPIO.cleanup()
