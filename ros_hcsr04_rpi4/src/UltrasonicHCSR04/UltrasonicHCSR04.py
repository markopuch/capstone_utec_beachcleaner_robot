import RPi.GPIO as GPIO
import time

class UltrasonicHCSR04(object):
    """
    UltrasonicHCSR04 class for interfacing with an HC-SR04 ultrasonic distance sensor.
    Attributes:
        trigger (int): GPIO pin number connected to the trigger pin of the HC-SR04.
        echo (int): GPIO pin number connected to the echo pin of the HC-SR04.
        timeout (float): Maximum time to wait for a response from the sensor.
    Methods:
        __init__(trigger, echo):
            Initializes the UltrasonicHCSR04 object with the specified trigger and echo pins.
        distance():
            Measures the distance to an object using the HC-SR04 sensor.
            Returns:
                float: Distance to the object in centimeters, or -1 if a timeout occurs.
        speed():
            Measures the speed of an object based on two consecutive distance measurements.
            Returns:
                float: Speed of the object in meters per second.
    """

    def __init__(self, trigger, echo):
        self.trigger = trigger
        self.echo = echo

        self.timeout = 0.05

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.trigger, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)

    def distance(self):
        GPIO.output(self.trigger, False)
        time.sleep(0.1)
        
        GPIO.output(self.trigger, True) # set trigger to HIGH

        # set trigger after 0.01 ms to LOW
        time.sleep(0.00001)
        GPIO.output(self.trigger, False)

        startTime = time.time()
        arrivalTime = time.time()

        timeout_start = time.time()

        # store startTime
        while GPIO.input(self.echo) == 0:
            startTime = time.time()

            if startTime - timeout_start > self.timeout:
                return -1

        # store arrivalTime
        while GPIO.input(self.echo) == 1:
            arrivalTime = time.time()

            if startTime - timeout_start > self.timeout:
                return -1

        if startTime != 0 and arrivalTime != 0:
            # calculate the difference between start and stop
            duration = arrivalTime - startTime

            # multiply with speed of sound (34300 cm/s)
            # and divide by 2 because there and back
            distance = (duration * 34300) / 2

            if distance >= 0:
                return distance
            else:
                return -1
        else:
            return -1
