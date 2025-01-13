import RPi.GPIO as GPIO
from time import sleep
import time
import argparse
import json

with open('/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json', 'r') as file:
        farm_config = json.load(file)
        


# 读取电机的控制以及各个sensor的pin码
pins = farm_config["pins"]

endPin = pins["endPin"]
resetPin = pins["resetPin"]
stopPin = pins["magnetPin"]
DIR = pins["DIR"]
ENA = pins["ENA"]
STEP =pins["STEP"]

GPIO.setwarnings(False) 
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
stopSensor = 1

def define_distance_between_stall(distance):
    # distance is the total distance between reset point and end point
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
        
def get_steps():
    """
    Calculate the needed steps between the reset point and the first magnet point, 
    and the steps between the first and second magnet points.
    
    Returns:
        int: Steps between the reset point and the first magnet point
        int: Steps between the first and second magnet points
        
    Steps from reset to first stall: 78012
    Steps from first to second stall: 40801
    """
    stepCount = 0
    Steps_resetpoint_to_firstStall = 0
    Steps_firstStall_to_second = 0
    preReset = False
    preStop = False
    
    # 防抖时间
    debounce_time = 10  # 50ms，具体值可根据磁场特性调整
    
    # Set motor rotation direction
    GPIO.output(DIR, 0)
    waitTime = 0.00001 / 0.05
    while True:
        resetSensor = GPIO.input(resetPin)
        stopSensor = GPIO.input(stopPin)
        
        # Check for reset point
        if resetSensor == 0 and not preReset:
            stepCount = 0
            print("Start to count the steps")
            preReset = True

        # Motor step signal
        GPIO.output(STEP, True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(STEP, False)
        
        stepCount += 1
        print(stepCount)

        # Check for first stop point
        if stopSensor == 0 and preReset and not preStop:
            Steps_resetpoint_to_firstStall = stepCount
            print(f"Steps from reset to first stall: {Steps_resetpoint_to_firstStall}")
            preStop = True

        # Check for second stop point
        elif stopSensor == 0 and preStop:
            Steps_firstStall_to_second = stepCount - Steps_resetpoint_to_firstStall
            if Steps_firstStall_to_second > 10000:
                Steps_firstStall_to_second = stepCount - Steps_resetpoint_to_firstStall
                print(f"Steps from reset to first stall: {Steps_resetpoint_to_firstStall}")
                print(f"Steps from first to second stall: {Steps_firstStall_to_second}")
                GPIO.output(ENA, True)  # Disable motor driver
                break

    # Release GPIO resources if needed
    GPIO.cleanup()

    return Steps_resetpoint_to_firstStall, Steps_firstStall_to_second

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
        GPIO.output(STEP,False)
        
        stepCount += 1
        print(stepCount)
        
        if endSensor == 0:
            GPIO.output(ENA, True)
            motor_end_time = time.time()
            break   
    total_time = motor_end_time - motor_start_time
    
    print(f"Totla distance cost {total_time}")
    
def count_stop_delay():
    '''
    用于统计磁感应器的范围
    '''
    stepCount = 0
    Step_to_leave_resetpoint = 0
    # 设置电机旋转方向
    GPIO.output(DIR,0)
    preStop = False
    
    while True:
        stopSensor = GPIO.input(stopPin)
        if stopSensor == 0:
            stepCount += 1
            preStop = True
            print(stepCount)
            # print("Start to count the step")
            
        GPIO.output(STEP,True)
        sleep(0.000001)
        GPIO.output(STEP,False)
        
        if preStop and stopSensor == 1:
            break
        
    
    print(f"Totla distance cost {stepCount}")

def back_to_dock():
    # 设置电机旋转方向
    GPIO.output(DIR,1)
    print("Start to go back to the dock.")
    while True:
        resetSensor = GPIO.input(resetPin)
            
        GPIO.output(STEP,True)
        sleep(0.000001)
        # sleep(1)
        GPIO.output(STEP,False)
        if resetSensor == 0:
            GPIO.output(ENA, True)
            print("Back in the dock now.")
            break

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

def test_stop():
    """
    Test each stop point whether works
    """
    stepCount = 0
    stall_id = 0
    preReset = False
    
    # 防抖时间
    debounce_time = 10  # 50ms，具体值可根据磁场特性调整
    
    # Set motor rotation direction
    GPIO.output(DIR, 0)
    waitTime = 0.000001 / 1
    while True:
        resetSensor = GPIO.input(resetPin)
        stopSensor = GPIO.input(stopPin)
        endSensor = GPIO.input(endPin)
        
        # Check for reset point
        if resetSensor == 0 and not preReset:
            stepCount = 0
            print("Start to count the steps")
            preReset = True

        # Motor step signal
        GPIO.output(STEP, True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(STEP, False)
        
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
            back_to_dock()
            # GPIO.output(ENA, True)      # Disable motor driver
            break  
    # Release GPIO resources if needed
    GPIO.cleanup()
    
def reset():
    """
    wherever the system is, go back to the dock
    """
    GPIO.output(DIR, 0)
    waitTime = 0.00001 / 1
    preStop = 0
    stepCount = 0
    while True:
        stopSensor = GPIO.input(stopPin)
        endSensor = GPIO.input(endPin)
        
        # Motor step signal
        GPIO.output(STEP, True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(STEP, False)
        
        stepCount += 1
        if stopSensor == 0:
            if stepCount > 10000:
                print(f"Get to the first stall by {stepCount}")
                stepCount = 0
                preStop = 1
        
        if preStop or endSensor == 0:
            back_to_dock()
            break

        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motor control script for Raspberry Pi.")
    parser.add_argument("command", nargs="?", choices=["ahead", "back", "reset", "test", "delay"],
                        help="System go ahead to the end or back to the reset point.")
    parser.add_argument("--distance", type=int, default=100, help="Total distance for counting the interval between two stall.")
    parser.add_argument("--step", action="store_true", help = "Caculate the needed steps between resetponint and the first magnet point, steps between the first magenet point and the second one")
    parser.add_argument("--test", action="store_true", help = "Test each stop point whether works")
    args = parser.parse_args()
    
    try:
        if args.command == "back":
            back_to_dock()
        elif args.command == "ahead":
            count_total_distance()
        elif args.command == "reset":
            reset()
        elif args.command == "test":
            test_stop()
        elif args.command == "delay":
            count_stop_delay()
        if args.step:
            get_steps()

    finally:
        GPIO.cleanup()

