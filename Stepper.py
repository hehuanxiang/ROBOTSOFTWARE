# CURRENT APPLICATION INFO
# 200 steps/rev
# 12V, 350mA
# Big Easy driver = 1/16 microstep mode
# Turn a 200 step motor left one full revolution: 3200
import sys
from time import sleep
import RPi.GPIO as gpio  # https://pypi.python.org/pypi/RPi.GPIO
#from mfrc522 import SimpleMFRC522


# import exitHandler #uncomment this and line 58 if using exitHandler

class Stepper:
    # instantiate stepper
    # pins = [stepPin, directionPin, enablePin]
    def __init__(self, pins):
        # setup pins
        self.pins = pins
        self.stepPin = self.pins[0]         # 29
        self.directionPin = self.pins[1]    # 15
        self.enablePin = self.pins[2]       # 11
        self.endPin = self.pins[3]          # 16
        self.resetPin = self.pins[4]        # 18
        self.magPin=self.pins[5]            # 37
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
        print("Original magnetic prestatus:{}".format(preStatus))
        print("docking status:{}".format(docking))
        sleep(10)
        preResetStatus = gpio.input(self.resetPin)  # 1为高电平，未感应到磁铁
        preEndStatus = gpio.input(self.endPin)
        
        # set the output to true for left and false for right
        turnLeft = False            # 将motor的初始点位设置为低电平，则电机将逆时针旋转，整个系统将后退
        if (dir == 'right'):
            turnLeft = True         # 将motor的初始点位设置为高电平，则电机将顺时针旋转，整个系统将前进
        elif (dir != 'left'):
            print("STEPPER ERROR: no direction supplied")
            return True
        
        # set the direction for spining
        gpio.output(self.directionPin, turnLeft)

        stepCounter = 0
        keepGoing = True

        waitTime = 0.00001 / speed  # waitTime controls speed, the less watitime, the faster for spining
        
        #preThreeStatus = [preStatus,preStatus,preStatus,preStatus,preStatus]
        preMagStatus = [1,1,1]
        resetStatus = [1,1,1]
        endStatus = [1,1,1]
        
        # test
        print("The goal of step is {}".format(steps))
        sleep(10)
        
        #reader = SimpleMFRC522()
        while keepGoing:
            
            if (stepCounter > 2*steps and docking == False):
                # 如果向右转到极限位置，返回 "right_end" 并进行对接（docking=True）。
                if(dir=="right"):
                    # 走到了最右边的尽头，即前进的尽头
                    print("right dead end")
                    turnLeft=False
                    stepCounter=0
                    steps=800000
                    docking=True
                    return "right_end"
                elif(dir=="left"):
                    # 如果向左旋转到极限，返回 "docked"，表示对接成功。
                    # 走到了最左边的尽头，起始点附近
                    print("left dead end")
                    turnLeft=False
                    stepCounter=0
                    steps=50000
                    docking=True
                    return "docked"
                
                gpio.output(self.directionPin, turnLeft)
                sleep(1)
                #input('Press <ENTER> to continue')
            
            # 每次将 stepPin 置为 True 再置为 False，完成一个脉冲，电机会执行一步。
            gpio.output(self.stepPin, True)
            sleep(waitTime)     # 等待时间控制速度：sleep(waitTime) 用于调节脉冲之间的时间间隔。
            gpio.output(self.stepPin, False)
            
            
            stepCounter += 1
            
            # test
            # print("StepCounter is {}".format(stepCounter))
        

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
                turnLeft=False          # 到了终点，电机应该停止前进，True为前进
                steps=1000000
                docking = True
                print("Hit end, returning to dock1")
                return "right_end"
            if(sum(resetStatus)== 0 and preResetStatus==1 and docking == True):
                sleep(1)
                turnLeft=True
                steps=220000
                stepCounter=0
                gpio.output(self.directionPin, turnLeft)
                preResetStatus=0
                print("reset needed1")
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
                    if (stepCounter>0.3*steps and docking == False):
                        print("detected stop")
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

        print("stepperDriver complete (turned " + dir + " " + str(stepCounter) + " steps)")

        return(stepCounter)
