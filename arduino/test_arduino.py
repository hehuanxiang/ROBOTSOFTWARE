import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
time.sleep(2)  # 等待 Arduino 重启

def send(cmd):
    ser.write((cmd + '\n').encode())
    print("Sent:", cmd)
    time.sleep(0.05)
    while ser.in_waiting:
        print("Received:", ser.readline().decode().strip())

send("E0")   # 上电
send("D0")   # 设置方向正转
for _ in range(10):
    send("S")  # 步进
send("E1")   # 断电
