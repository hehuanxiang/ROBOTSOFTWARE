import RPi.GPIO as GPIO
from time import sleep
class Sensor:
    def __init__(self, pin, name="Sensor"):
        self.pin = pin
        self.name = name
        GPIO.setmode(GPIO.BOARD)  # 使用 BOARD 编号模式
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # 配置为输入并启用上拉电阻

    def read_status(self):
        return GPIO.input(self.pin)  # 读取 GPIO 引脚状态

    def test_sensor(self):
        status = self.read_status()
        if status == 0:
            print(f"{self.name} 传感器：磁铁靠近，输出低电平 {status}")
        else:
            print(f"{self.name} 传感器：磁铁远离，输出高电平 {status}")

# 示例用法：灵活测试不同传感器
def main():
    # 初始化不同的传感器
    magnet_sensor = Sensor(pin=37, name="Magnet")
    
    # 2024年11月15日10点34分，调换stop和reset
    # stop_sensor = Sensor(pin=16, name="Stop")
    # reset_sensor = Sensor(pin=18, name="Reset")
    stop_sensor = Sensor(pin=18, name="Stop")
    reset_sensor = Sensor(pin=16, name="Reset")
    
    # 测试每个传感器的状态
    i = 0
    while i < 10:
        magnet_sensor.test_sensor()
        stop_sensor.test_sensor()
        reset_sensor.test_sensor()
        sleep(2)
        
        i += 1

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        GPIO.cleanup()  # 确保退出时清理 GPIO 状态
