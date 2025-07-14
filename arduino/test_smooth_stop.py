import serial
import time

def send_cmd(ser, cmd):
    ser.write((cmd + '\n').encode())
    print(f">> Sent: {cmd}")
    time.sleep(0.1)
    while ser.in_waiting:
        resp = ser.readline().decode().strip()
        print(f"<< Received: {resp}")

def main():
    try:
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        time.sleep(2)  # 等待 Arduino 重启
        print("✅ Serial connected")

        send_cmd(ser, "E0")            # 上电
        send_cmd(ser, "D0")            # 设置为正转
        send_cmd(ser, "START 8000")    # 开始运动
        time.sleep(2)                  # 运行一段时间

        send_cmd(ser, "STOP 60 8")     # 缓慢减速停止

        send_cmd(ser, "E1")            # 下电

    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
