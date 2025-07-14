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
    waitTime = 0.0001 / 1
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

def get_second_stop():
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
    stopCount = 0
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
                stopCount += 1
        # Check for the left stop point
        elif stopSensor == 0:
            if stepCount > 5000:
                print(f"Steps from NO.{stall_id - 1} to NO.{stall_id}: {stepCount}")
                print(f'The NO.{stall_id} step point works.')
                
                stepCount = 0
                stall_id += 1
                stopCount += 1
        if stopCount == 2:
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

def test_motor_reverse():
    """
    Test motor forward and reverse rotation for a short distance.
    """
    stepCount = 20000  # 调整这个值控制旋转的距离
    delay = 0.00001   # 每步的延迟时间，可根据电机响应能力微调

    # 启动电机
    GPIO.output(ENA, GPIO.LOW)

    print("Forward rotation...")
    GPIO.output(DIR, 0)  # 正转方向
    for _ in range(stepCount):
        GPIO.output(STEP, True)
        sleep(delay)
        GPIO.output(STEP, False)
        sleep(delay)

    sleep(1)  # 停一下再反转

    print("Reverse rotation...")
    GPIO.output(DIR, 1)  # 反转方向
    for _ in range(stepCount):
        GPIO.output(STEP, True)
        sleep(delay)
        GPIO.output(STEP, False)
        sleep(delay)

    print("Test complete.")
    GPIO.output(ENA, GPIO.HIGH)  # 停止电机
    
def test_position():
    """
    电机前进，检测到新磁点且步数大于 min_steps_between_stalls 后停机。
    防止重复触发同一个磁点。
    """
    GPIO.output(DIR, 0)
    waitTime = 0.00001
    min_steps_between_stalls = 5000
    steps_since_last_stall = 0
    visited_stalls = 0

    GPIO.output(ENA, GPIO.LOW)
    print("Start moving through stalls...")

    try:
        while True:
            stopSensor = GPIO.input(stopPin)
            endSensor = GPIO.input(endPin)

            # 步进
            GPIO.output(STEP, True)
            time.sleep(waitTime)
            GPIO.output(STEP, False)
            time.sleep(waitTime)
            steps_since_last_stall += 1

            # 检查新磁点
            if stopSensor == 0 and steps_since_last_stall >= min_steps_between_stalls:
                visited_stalls += 1
                print(f"Reached new stall #{visited_stalls} after {steps_since_last_stall} steps.")
                GPIO.output(ENA, GPIO.HIGH)  # 断电停机
                break  # 程序停止，等待人工介入

            if endSensor == 0:
                print("Reached the end point.")
                GPIO.output(ENA, GPIO.HIGH)
                break

    except KeyboardInterrupt:
        print("\nManual stop detected. Exiting safely.")
        GPIO.output(ENA, GPIO.HIGH)

    finally:
        GPIO.cleanup()



def enter_test():
    while True:
        print("1")
        input("Press Enter to continue...")
        
def test_cycle():
    """
    无限循环测试：从起点出发，检查每个 stop 是否有效，走到终点后返回起点。直到用户手动终止 (Ctrl+C)。
    """
    cycle = 1
    try:
        while True:
            print(f"\n==== Starting test cycle {cycle} ====")
            stepCount = 0
            stall_id = 0
            preReset = False
            waitTime = 0.000001 / 1

            GPIO.output(DIR, 0)  # 设置前进方向
            while True:
                resetSensor = GPIO.input(resetPin)
                stopSensor = GPIO.input(stopPin)
                endSensor = GPIO.input(endPin)

                # 检测是否刚离开 reset 点
                if resetSensor == 0 and not preReset:
                    stepCount = 0
                    print("Start to count steps from reset.")
                    preReset = True

                # 电机前进一步
                GPIO.output(STEP, True)
                sleep(waitTime)
                GPIO.output(STEP, False)

                stepCount += 1

                # 检测 stop 点
                if stopSensor == 0 and preReset:
                    if stepCount > 5000:
                        print(f"Steps from reset to stall #{stall_id}: {stepCount}")
                        print(f'Stall #{stall_id} is active.')
                        stall_id += 1
                        stepCount = 0

                        # 等待传感器恢复
                        while GPIO.input(stopPin) == 0:
                            sleep(0.01)

                # 到达终点
                if endSensor == 0:
                    print("Reached the end point.")
                    break

            # 返回起点
            back_to_dock()
            print(f"==== Cycle {cycle} complete ====")
            cycle += 1

    except KeyboardInterrupt:
        print("\nManual stop received. Exiting test cycle.")
        GPIO.output(ENA, True)
        GPIO.cleanup()



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Motor control script for Raspberry Pi.")
    parser.add_argument("command", choices=[
        "ahead",      # 向前行驶到终点
        "back",       # 返回起点
        "reset",      # 检测到stop点后再返回
        "test",       # 检测每个stop点是否有效（一次性）
        "delay",      # 检查stop delay
        "second",     # 检查两个stop点
        "step",       # 计算reset到stop1，stop1到stop2的步数
        "reverse",    # 测试正反转
        "magnet",     # 检查磁铁位置
        "cycle"       # 无限循环测试所有stop点
    ], help="Command to execute.")

    args = parser.parse_args()

    
    # enter_test()
    
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
        elif args.command == "second":
            get_second_stop()
        elif args.command == "step":
            get_steps()
        elif args.command == "reverse":
            test_motor_reverse()
        elif args.command == "magnet":
            test_position()
        elif args.command == "cycle":
            test_cycle()

    finally:
        GPIO.cleanup()


