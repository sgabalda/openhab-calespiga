#Libraries
import RPi.GPIO as GPIO
import sys
 
# Get the total number of args passed to the demo.py
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

if total != 2 :
    print("Not valid arguments. Arguments are <GPIO_NUM> <ON|OFF>")
    sys.exit(1)
else: 
    gpioNum = getGPIONum
    status = getStatus
    GPIO.setmode(GPIO.BCM)                  # choose BCM or BOARD  
    GPIO.setup(gpioNum, GPIO.OUT)           # set GPIO as an output
    if status == "ON":
        GPIO.output(24, 1)
    else:
        GPIO.output(24, 0)  
