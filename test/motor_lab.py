import RPi.GPIO as GPIO
from time import sleep
import time
import argparse
import json

def load_config(config_path):
    """Load configuration from the specified JSON file."""
    with open(config_path, 'r') as file:
        return json.load(file)

def setup_gpio(pins):
    """Set up GPIO pins based on the configuration."""
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pins["DIR"], GPIO.OUT)
    GPIO.setup(pins["STEP"], GPIO.OUT)
    GPIO.setup(pins["ENA"], GPIO.OUT)
    GPIO.setup(pins["resetPin"], GPIO.IN)
    GPIO.setup(pins["endPin"], GPIO.IN)
    GPIO.setup(pins["magnetPin"], GPIO.IN)

    GPIO.output(pins["ENA"], GPIO.LOW)  # Enable motor driver

def define_distance_between_stall(distance, pins):
    Steps_for_magnet = 0
    while True:
        GPIO.output(pins["STEP"], True)
        sleep(0.000005)
        GPIO.output(pins["STEP"], False)
        sleep(0.000005)
        Steps_for_magnet += 1

        if Steps_for_magnet % distance == 0:
            sleep(20)

        if GPIO.input(pins["endPin"]) == 0:
            GPIO.output(pins["ENA"], True)
            break

def reset(pins):
    """
    wherever the system is, go back to the dock
    """
    GPIO.output(pins["DIR"], 0)
    waitTime = 0.00001 / 1
    preStop = 0
    stepCount = 0
    while True:
        stopSensor = GPIO.input(pins["magnetPin"])
        endSensor = GPIO.input(pins["endPin"])
        
        # Motor step signal
        GPIO.output(pins["STEP"], True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(pins["STEP"], False)
        
        stepCount += 1
        if stopSensor == 0:
            if stepCount > 10000:
                print(f"Get to the first stall by {stepCount}")
                stepCount = 0
                preStop = 1
        
        if preStop or endSensor == 0:
            back_to_dock(pins)
            break

def get_steps(pins):
    """Calculate steps between points."""
    stepCount = 0
    Steps_resetpoint_to_firstStall = 0
    Steps_firstStall_to_second = 0
    preReset = False
    preStop = False

    GPIO.output(pins["DIR"], 0)
    waitTime = 0.00001 / 0.05

    while True:
        resetSensor = GPIO.input(pins["resetPin"])
        stopSensor = GPIO.input(pins["magnetPin"])

        if resetSensor == 0 and not preReset:
            stepCount = 0
            print("Start to count the steps")
            preReset = True

        GPIO.output(pins["STEP"], True)
        sleep(waitTime)
        GPIO.output(pins["STEP"], False)

        stepCount += 1

        if stopSensor == 0 and preReset and not preStop:
            Steps_resetpoint_to_firstStall = stepCount
            print(f"Steps from reset to first stall: {Steps_resetpoint_to_firstStall}")
            preStop = True

        elif stopSensor == 0 and preStop:
            Steps_firstStall_to_second = stepCount - Steps_resetpoint_to_firstStall
            if Steps_firstStall_to_second > 10000:
                print(f"Steps from first to second stall: {Steps_firstStall_to_second}")
                GPIO.output(pins["ENA"], True)
                break

    GPIO.cleanup()
    return Steps_resetpoint_to_firstStall, Steps_firstStall_to_second

def test_stop(pins):
    """
    Test each stop point whether works
    """
    stepCount = 0
    stall_id = 0
    preReset = False
    
    # 防抖时间
    debounce_time = 10  # 50ms，具体值可根据磁场特性调整
    
    # Set motor rotation direction
    GPIO.output(pins["DIR"], 0)
    waitTime = 0.000001 / 1
    while True:
        resetSensor = GPIO.input(pins["resetPin"])
        stopSensor = GPIO.input(pins["magnetPin"])
        endSensor = GPIO.input(pins["endPin"])
        
        # Check for reset point
        if resetSensor == 0 and not preReset:
            stepCount = 0
            print("Start to count the steps")
            preReset = True

        # Motor step signal
        GPIO.output(pins["STEP"], True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(pins["STEP"], False)
        
        stepCount += 1

        # Check for first stop point
        if stopSensor == 0 and preReset:
            if stepCount > 5000:
                print(f"Steps from reset to first stall: {stepCount}")
                print(f'The NO.{stall_id} step point works.')
                stepCount = 0
                stall_id += 1
                preReset = False
        # Check for the left stop point
        elif stopSensor == 0:
            if stepCount > 5000:
                print(f"Steps from NO.{stall_id - 1} to NO.{stall_id}: {stepCount}")
                print(f'The NO.{stall_id} step point works.')
                
                stepCount = 0
                stall_id += 1
        if endSensor == 0:
            back_to_dock(pins)
            # GPIO.output(ENA, True)      # Disable motor driver
            break  
    # Release GPIO resources if needed
    GPIO.cleanup()

def back_to_dock(pins):
    GPIO.output(pins["DIR"], 1)
    print("Start to go back to the dock.")
    while True:
        resetSensor = GPIO.input(pins["resetPin"])

        GPIO.output(pins["STEP"], True)
        sleep(0.000001)
        GPIO.output(pins["STEP"], False)

        if resetSensor == 0:
            GPIO.output(pins["ENA"], True)
            print("Back in the dock now.")
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motor control script for Raspberry Pi.")
    parser.add_argument("command", nargs="?", choices=["ahead", "back", "reset", "test", "delay"],
                        help="System go ahead to the end or back to the reset point.")
    parser.add_argument("--distance", type=int, default=100,
                        help="Total distance for counting the interval between two stall.")
    parser.add_argument("--step", action="store_true",
                        help="Calculate the needed steps between reset point and the first magnet point, and between magnet points.")
    parser.add_argument("--test", action="store_true",
                        help="Test each stop point whether it works.")
    parser.add_argument("--config", type=str, default="/home/pi/Desktop/ROBOTSOFTWARE/farm_config_lab.json",
                        help="Path to the JSON configuration file.")

    args = parser.parse_args()

    try:
        # Load configuration and setup GPIO
        farm_config = load_config(args.config)
        pins = farm_config["pins"]
        setup_gpio(pins)

        if args.command == "back":
            back_to_dock(pins)
        elif args.command == "ahead":
            define_distance_between_stall(args.distance, pins)
        elif args.command == "reset":
            reset(pins)
        elif args.command == "test":
            test_stop(pins)
        elif args.step:
            get_steps(pins)

    finally:
        GPIO.cleanup()
