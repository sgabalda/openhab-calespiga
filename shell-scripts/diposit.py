#Libraries
import RPi.GPIO as GPIO
import time
import argparse
import sys

TIME_BETWEEN_MEASUREMENTS = 0.5
DISCARDED_START_MEASUREMENTS = 1
DISCARDED_END_MEASUREMENTS = 0
MEASUREMENTS_TOTAL = 8
DEVIATION_DISCARD_MEASURE = 0.2
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

debug = False
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    if debug:
        print ("Waiting for GPIO_ECHO to be 0")
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
		
    # save time of arrival
    if debug:
        print ("Waiting for GPIO_ECHO to be 1")
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
     
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    if debug:
        print("TimeElapsed is " + TimeElapsed)
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance
    
def getRawMeasurements():
    measurements = []
    for x in range(MEASUREMENTS_TOTAL):
        dist = distance()
        measurements.append(dist)
        time.sleep(TIME_BETWEEN_MEASUREMENTS)
        if debug:
            print("Measured Distance = %.1f cm" % dist)
    return measurements
 	
def getFilteredMeasurements(measurements):
    measurements = measurements[DISCARDED_START_MEASUREMENTS:]
    if DISCARDED_END_MEASUREMENTS > 0:
        measurements = measurements[:-DISCARDED_END_MEASUREMENTS]

    mean = sum(measurements) / float(len(measurements))
    if debug:
        print("The mean is: " + mean)
    result = [a for a in measurements if abs(a - mean) < DEVIATION_DISCARD_MEASURE]
    if len(result) == 0:
        result = [mean]
    if debug:
        print("The filtered measurements is : " + mean)
    return result
 	
def getMeasurement(measurements):
    return sum(measurements) / float(len(measurements))
	
def cmToPercentage(measurement):
    result = 100 - (measurement - 9.1)*100/200
    if result < 0:
        result = 0
    return result 
 
if __name__ == '__main__':
    try:
        if len(sys.argv) > 1:
            if sys.argv[1] == "debug":
                debug = True
        measurement = getMeasurement(getFilteredMeasurements(getRawMeasurements()))
        percentage = cmToPercentage(measurement)
        print ("%.2f" % percentage)
    finally:
        GPIO.cleanup()