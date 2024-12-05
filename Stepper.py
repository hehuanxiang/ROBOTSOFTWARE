# CURRENT APPLICATION INFO
# 200 steps/rev
# 12V, 350mA
# Big Easy driver = 1/16 microstep mode
# Turn a 200 step motor left one full revolution: 3200
import sys
from time import sleep
import RPi.GPIO as gpio  # https://pypi.python.org/pypi/RPi.GPIO
#from mfrc522 import SimpleMFRC522
import logging
import multiprocessing
from logging.handlers import QueueHandler


# import exitHandler #uncomment this and line 58 if using exitHandler

def setup_logger(logger_name, log_queue):
    logger = logging.getLogger(logger_name)
    if not logger.handlers:  # 如果没有已存在的 Handler，则添加
        handler = logging.handlers.QueueHandler(log_queue)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger

class Stepper:
    # instantiate stepper
    # pins = [stepPin, directionPin, enablePin]
    def __init__(self, pins, log_queue):
        # setup pins
        self.pins = pins
        self.stepPin = self.pins[0]         # 31,原来是29线断在里面了，换了一个
        self.directionPin = self.pins[1]    # 15
        self.enablePin = self.pins[2]       # 11
        self.endPin = self.pins[3]          # 16
        self.resetPin = self.pins[4]        # 18
        self.magPin=self.pins[5]            # 37
        
        self.logger = setup_logger(f"Stepper", log_queue)
        
        # use the broadcom layout for the gpio
        gpio.setmode(gpio.BOARD)
        gpio.setwarnings(False) 
        # set gpio pins
        gpio.setup(self.stepPin, gpio.OUT)
        gpio.setup(self.directionPin, gpio.OUT)
        gpio.setup(self.enablePin, gpio.OUT)
        gpio.setup(self.endPin, gpio.IN)
        gpio.setup(self.resetPin, gpio.IN)
        gpio.setup(self.magPin, gpio.IN)

        # set enable to high (i.e. power is NOT going to the motor)
        gpio.output(self.enablePin, True)

        #print("Stepper initialized (step=" + self.stepPin + ", direction=" + self.directionPin + ", enable=" + self.enablePin + ")")

    # clears GPIO settings
    def cleanGPIO(self):
        gpio.cleanup()

    # step the motor
    # steps = number of steps to take
    # dir = direction stepper will move
    # speed = defines the denominator in the waitTime equation: waitTime = 0.000001/speed. As "speed" is increased, the waitTime between steps is lowered
    # stayOn = defines whether or not stepper should stay "on" or not. If stepper will need to receive a new step command immediately, this should be set to "True." Otherwise, it should remain at "False."
    # docking = defines whether or not the system stop at the magnet position
    def step(self, steps, dir, speed, stayOn=False, docking = False):
        # set enable to low (i.e. power IS going to the motor)
        gpio.output(self.enablePin, False)
        
        # get the sensor status
        preStatus = gpio.input(self.magPin)
        preResetStatus = gpio.input(self.resetPin)  # 1为高电平，未感应到磁铁
        preEndStatus = gpio.input(self.endPin)
        
        # forward, direction == False; back, direction == True
        direction = False            
        if (dir == 'back'):
            direction = True         
        elif (dir != 'forward'):
            self.logger.info("未指明电机运动方向")
            return True
        
        # set the direction for spining
        gpio.output(self.directionPin, direction)

        stepCounter = 0
        keepGoing = True

        waitTime = 0.00001 / speed  # waitTime controls speed, the less watitime, the faster for spining
        
        #preThreeStatus = [preStatus,preStatus,preStatus,preStatus,preStatus]
        preMagStatus = [1,1,1]
        resetStatus = [1,1,1]
        endStatus = [1,1,1]
        
        #reader = SimpleMFRC522()
        while keepGoing:
            
            # docking判断当前是否要回到reset点，
            # false意味着是要去每一个stall，如果超过了预设步数的两倍，
            # 意味着中间某个stall的磁铁失效了
            # 这个应该是用于来回拍的
            if (stepCounter > 2*steps and docking == False):
                # 如果向右转到极限位置，返回 "right_end" 并进行对接（docking=True）。
                if(dir=="back"):
                    # 走到了最右边的尽头，即前进的尽头
                    print("right dead end")
                    direction=False
                    stepCounter=0
                    steps=800000
                    docking=True
                    return "right_end"
                elif(dir=="forward"):
                    print("left dead end")
                    direction=False
                    stepCounter=0
                    steps=50000
                    docking=True
                    return "docked"
                
                gpio.output(self.directionPin, direction)
                sleep(1)
                #input('Press <ENTER> to continue')
            
            # 每次将 stepPin 置为 True 再置为 False，完成一个脉冲，电机会执行一步。
            gpio.output(self.stepPin, True)
            sleep(waitTime)     # 等待时间控制速度：sleep(waitTime) 用于调节脉冲之间的时间间隔。
            gpio.output(self.stepPin, False)
            
            
            stepCounter += 1
        
            # 状态缓冲：每次循环更新最近三次的传感器状态，便于判断状态变化。
            preMagStatus[0] = preMagStatus[1]
            preMagStatus[1] = preMagStatus[2]
            preMagStatus[2] = gpio.input(self.magPin)
            
            # test
            # print(preMagStatus)

            resetStatus[0] = resetStatus[1]
            resetStatus[1] = resetStatus[2]           
            resetStatus[2] = gpio.input(self.resetPin)
            
            endStatus[0] = endStatus[1]
            endStatus[1] = endStatus[2]  
            endStatus[2] = gpio.input(self.endPin)
            #print(gpio.input(self.magPin))
            if sum(endStatus) == 0:
                sleep(1)
                direction=False          # 到了终点，电机应该停止前进，True为前进
                steps=1000000
                docking = True
                print("Hit end, returning to dock1")
                return "right_end"
            if(sum(resetStatus)== 0 and preResetStatus==1 and docking == True):
                sleep(1)
                direction=True
                steps=220000
                stepCounter=0
                gpio.output(self.directionPin, direction)
                preResetStatus=0
                self.logger.info(f"检测到reset point")
                docking=False
                return "docked"
                #keepGoing = False
                #break
                #sys.exit()

            if(stepCounter > 1.2*steps and docking == False):
                keepGoing = False

            # print("Current magnetic status is {}".format(gpio.input(self.magPin)))
            # print("Current end status is {}".format(gpio.input(self.endPin)))
            # print("Current reset status is {}".format(gpio.input(self.resetPin)))
            if (preStatus == 1):
                if (sum(preMagStatus) == 0 and docking == False):
                    # if (stepCounter>0.3*steps and docking == False):
                    if (docking == False):
                        self.logger.info(f"检测到检查点，准备开始进行拍照，消耗步数：{stepCounter}")
                        keepGoing= False
                        return None
            #elif(preStatus == 0):
                #if (sum(preThreeStatus)== 5):
                    #if (stepCounter>0.5*steps and docking == False):
                        #print("stepCounter:  "+ str(stepCounter))

                        #keepGoing= False

            if (stepCounter > steps or docking == True):
                if(sum(resetStatus)== 0): 
                    print("Docked, stepCounter:  "+ str(stepCounter))
                    keepGoing = False
                    return "docked"

            # wait before taking the next step thus controlling rotation speed

        #if (stayOn == False):
            # set enable to high (i.e. power is NOT going to the motor)
         #   gpio.output(self.enablePin, True)
        #print(gpio.input(self.stopPin))
        
        # 将 enablePin 置为 高电平，停止电机供电。
        gpio.output(self.enablePin, True)

        # self.logger.info("未指明电机运动方向")
        self.logger.info(f"当前步进结束，开始{dir}, 完全使用预设步数为：{stepCounter}")

        return(stepCounter)