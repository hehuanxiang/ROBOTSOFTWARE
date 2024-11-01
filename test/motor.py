import RPi.GPIO as GPIO
from time import sleep

# sw5 = off
# sw6 = on
# sw7 = off
# sw8 = on
# Pulse/rec = 6400

GPIO.setwarnings(False) 

DIR = 15
STEP= 29
ENA = 11
resetPin = 18
endPin = 16

CW=1
CCW=0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(STEP,GPIO.OUT)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(resetPin, GPIO.IN)
GPIO.setup(endPin, GPIO.IN)

# GPIO.output(DIR,1)

# ENB低电平为通电，高电平断电
GPIO.output(ENA,GPIO.LOW)

resetSensor = 1
endSensor = 1
stepCount = 0






def define_distance_between_stall(distance):
    Steps_for_magenet = 0
    while True:
        GPIO.output(STEP,True)
        sleep(0.000005)
        GPIO.output(STEP,False)
        sleep(0.000005)
        Steps_for_magenet +=1
        
        if Steps_for_magenet % distance == 0:
            sleep(20)
            
        if endSensor == 0:
            GPIO.output(ENA, True)
            break   
    
def count_total_distance():
    Step_to_leave_resetpoint = 0
    # 设置电机旋转方向
    GPIO.output(DIR,0)
    while True:
        resetSensor = GPIO.input(resetPin)
        endSensor = GPIO.input(endPin)
        if resetSensor == 0:
            stepCount = 0
            print("Start to count the step")
            Step_to_leave_resetpoint += 1
            print(Step_to_leave_resetpoint)
            


        GPIO.output(STEP,True)
        sleep(0.000005)
        GPIO.output(STEP,False)
        sleep(0.000005)
        
        stepCount += 1
        

        print(stepCount)

        print("Steps needed to leave reset point: {}\n".format(Step_to_leave_resetpoint))
        print("Total steps is {}".format(stepCount))
        
        if endSensor == 0:
            GPIO.output(ENA, True)
            break   





