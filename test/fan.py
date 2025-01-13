import RPi.GPIO as GPIO
import time

# 设置GPIO针脚
FAN_PIN = 15
GPIO.setmode(GPIO.BCM)
GPIO.setup(FAN_PIN, GPIO.OUT)

# 设置PWM频率为25kHz
fan = GPIO.PWM(FAN_PIN, 25000)
fan.start(50)  # 初始占空比50%
speed = int(input("输入风扇速度(0-100): "))
try:
    while True:
        fan.ChangeDutyCycle(speed)
except KeyboardInterrupt:
    print("退出")
finally:
    fan.stop()
    GPIO.cleanup()
