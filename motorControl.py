import RPi.GPIO as GPIO
from time import sleep
import time

class MotorControl:
    def __init__(self, dir_pin, step_pin, ena_pin, reset_pin, end_pin, stop_pin):
        self.DIR = dir_pin
        self.STEP = step_pin
        self.ENA = ena_pin
        self.resetPin = reset_pin
        self.endPin = end_pin
        self.stopPin = stop_pin

        self.resetSensor = 1
        self.endSensor = 1

    def go_ahead_to_end(self):
        stepCount = 0
        Step_to_leave_resetpoint = 0
        GPIO.output(self.DIR, 0)

        motor_start_time = time.time()
        while True:
            self.resetSensor = GPIO.input(self.resetPin)
            self.endSensor = GPIO.input(self.endPin)

            if self.resetSensor == 0:
                stepCount = 0
                print("Start to count the step")
                Step_to_leave_resetpoint += 1
                print(Step_to_leave_resetpoint)

            GPIO.output(self.STEP, True)
            sleep(0.000001)
            GPIO.output(self.STEP, False)

            stepCount += 1
            print(stepCount)

            if self.endSensor == 0:
                GPIO.output(self.ENA, True)
                motor_end_time = time.time()
                break

        total_time = motor_end_time - motor_start_time
        print(f"Total distance cost {total_time}")

    def back_to_dock(self):
        GPIO.output(self.DIR, 1)
        print("Start to go back to the dock.")
        while True:
            self.resetSensor = GPIO.input(self.resetPin)
            GPIO.output(self.STEP, True)
            sleep(0.000001)
            GPIO.output(self.STEP, False)

            if self.resetSensor == 0:
                GPIO.output(self.ENA, True)
                print("Back in the dock now.")
                break
