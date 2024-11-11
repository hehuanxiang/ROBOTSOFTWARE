import RPi.GPIO as GPIO
from time import sleep
import time

# sw5 = off
# sw6 = on
# sw7 = off
# sw8 = on
# Pulse/rec = 6400

GPIO.setwarnings(False) 

DIR = 15
STEP= 31
ENA = 11
resetPin = 18
endPin = 16
stopPin = 37

CW=1
CCW=0

GPIO.setmode(GPIO.BOARD)
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(STEP,GPIO.OUT)
GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(resetPin, GPIO.IN)
GPIO.setup(endPin, GPIO.IN)
GPIO.setup(stopPin, GPIO.IN)

# GPIO.output(DIR,1)

# ENB低电平为通电，高电平断电
GPIO.output(ENA,GPIO.LOW)

resetSensor = 1
endSensor = 1

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
    stepCount = 0
    Step_to_leave_resetpoint = 0
    # 设置电机旋转方向
    GPIO.output(DIR,0)
    
    motor_start_time = time.time()
    while True:
        resetSensor = GPIO.input(resetPin)
        endSensor = GPIO.input(endPin)
        if resetSensor == 0:
            stepCount = 0
            print("Start to count the step")
            Step_to_leave_resetpoint += 1
            print(Step_to_leave_resetpoint)
            
        GPIO.output(STEP,True)
        sleep(0.000001)
        # sleep(1)
        GPIO.output(STEP,False)
        # sleep(0.0005)
        
        stepCount += 1
        

        print(stepCount)

        print("Steps needed to leave reset point: {}\n".format(Step_to_leave_resetpoint))
        print("Total steps is {}".format(stepCount))
        
        if endSensor == 0:
            GPIO.output(ENA, True)
            motor_end_time = time.time()
            break   
    total_time = motor_end_time - motor_start_time
    
    print(f"Totla distance cost {total_time}")

def slow_brake_test():
    # 设置步进方向
    GPIO.output(DIR, 0)
    
    # 初始化PWM，控制STEP引脚
    pwm = GPIO.PWM(STEP, 128000000000)  # 根据需要调整频率
    pwm.start(100)  # 全速启动
    
    try:
        while True:
            # 读取停止传感器状态
            stopSensor = GPIO.input(stopPin)
            if stopSensor == 0:
                # 检测到停止传感器，逐步减速
                for duty_cycle in range(100, 0, -5):
                    pwm.ChangeDutyCycle(duty_cycle)
                    time.sleep(0.1)  # 每次降低占空比后延时
                pwm.stop()  # 完全停止电机
                break  # 退出循环
            else:
                GPIO.output(STEP, True)
                sleep(0.00000005)
                GPIO.output(STEP, False)
                sleep(0.00000005)
    finally:
        GPIO.cleanup()



# slow_brake_test()
count_total_distance()


