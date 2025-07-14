import serial
import time

# 打开串口连接
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)  # 等待 Arduino 初始化

def move(steps, freq):
    cmd = f"MOVE {steps} {freq}\n"
    ser.write(cmd.encode())
    print("Sent:", cmd.strip())
    time.sleep(0.5)  # 等待 Arduino 执行
    while ser.in_waiting:
        print("Arduino:", ser.readline().decode().strip())

# 上电（使能）
ser.write(b"E0\n")

# 设置方向正转
ser.write(b"D0\n")

# 分别测试不同频率（从慢到快）
test_cases = [
    (2000, 200),   # 慢速
    (2000, 500),   # 中速
    (2000, 1000),  # 快速
    (2000, 2000),  # 非常快（确保电机跟得上）
]

for steps, freq in test_cases:
    print(f"\n==> Moving {steps} steps @ {freq} Hz")
    move(steps, freq)
    time.sleep(1)

# 断电（失能）
ser.write(b"E1\n")
print("\n✅ Test complete")
