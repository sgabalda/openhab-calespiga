#Libraries
import RPi.GPIO as GPIO
import sys
import time
 
total = len(sys.argv)

def getGPIONum():
    try:
        return int(sys.argv[1])
    except ValueError:
        print(f"Not valid argument for <GPIO_NUM>, must be a number: {sys.argv[1]}")
        sys.exit(1)

def getStatus():
    status = sys.argv[2]
    if status == "ON":
        return 1
    elif status == "OFF":
        return 0
    else:
        print(f"Not valid argument for <ON|OFF>: {sys.argv[2]}")
        sys.exit(1)

if total != 3 :
    print(f"Not valid arguments. Arguments are <GPIO_NUM> <ON|OFF>")
    sys.exit(1)
else: 
    gpioNum = getGPIONum()
    status = getStatus()
    GPIO.setmode(GPIO.BCM)                  # choose BCM or BOARD  
    GPIO.setup(gpioNum, GPIO.OUT)           # set GPIO as an output
    if status == 1:
        print("turning it ON")
        GPIO.output(gpioNum, GPIO.HIGH)
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.LOW) 
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.HIGH)
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.LOW)
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.HIGH)
        print("turned it ON")
    else:
        print("turning it OFF")
        GPIO.output(gpioNum, GPIO.LOW)  
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.HIGH)
        time.sleep(4)
        GPIO.output(gpioNum, GPIO.LOW)
        print("turned it OFF")

    GPIO.cleanup()
