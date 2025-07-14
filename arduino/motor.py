import time
import serial
import json
import RPi.GPIO as GPIO
import argparse

# ===== 参数 =====
PORT = "/dev/ttyACM0"  # 串口号，视实际情况调整
BAUDRATE = 9600
FREQ = 5000            # 回退频率
MAX_STEP_TIME = 100     # 最多等待 10 秒防止死循环

# ===== 读取引脚配置 =====
with open('/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json', 'r') as f:
    config = json.load(f)
pins = config["pins"]
resetPin = pins["resetPin"]
endPin = pins["endPin"]

# ===== 初始化 GPIO（仅用于读取 resetPin） =====
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(resetPin, GPIO.IN)
GPIO.setup(endPin, GPIO.IN)
GPIO.setup(pins["magnetPin"], GPIO.IN)  # 磁铁位置引脚

# ===== 串口通信函数 =====
def send_arduino_cmd(ser, cmd):
    ser.write((cmd + '\n').encode())
    time.sleep(0.001)

def send_and_wait_ack(ser, cmd, expect_ack, timeout=3):
    ser.flushInput()
    ser.write((cmd + '\n').encode())
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line == expect_ack:
                return True
    return False

# ===== 主函数：回退至 reset 点 =====
def back_to_dock():
    try:
        print("🔌 正在连接 Arduino ...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print("✅ 串口连接成功")

        send_arduino_cmd(ser, "D1")  # 设置为回退方向
        if not send_and_wait_ack(ser, f"START {FREQ}", "ACK_START"):
            print("❌ 启动回退失败")
            return

        print("⏳ 开始回退，等待 resetPin 触发 ...")
        while GPIO.input(resetPin) != 0:
            time.sleep(0.001)

        print("✅ 检测到归位点，发送 STOP")
        if not send_and_wait_ack(ser, "STOP", "ACK_STOP"):
            print("⚠️ 未收到 ACK_STOP，但已发送 STOP")

        ser.close()
        print("✅ 完成回退")

    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        GPIO.cleanup()
        
def count_total_distance():
    try:
        print("🔌 正在连接 Arduino ...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print("✅ 串口连接成功")

        send_arduino_cmd(ser, "D0")
        if not send_and_wait_ack(ser, f"START {FREQ}", "ACK_START"):
            print("❌ 启动失败")
            return

        print("🏁 等待离开 resetPin 后开始计时 ...")

        print("⏱️ 已离开 resetPin，开始计时")
        start_time = time.time()

        # 等待 endPin == 0
        while GPIO.input(endPin) != 0:
            time.sleep(0.001)

        end_time = time.time()
        send_and_wait_ack(ser, "STOP", "ACK_STOP")

        duration = end_time - start_time
        print(f"✅ 到达终点，用时 {duration:.3f} 秒")
        ser.close()

    except Exception as e:
        print(f"❌ 串口错误: {e}")
    finally:
        GPIO.cleanup()
        
def test_stop():
    try:
        print("🔌 正在连接 Arduino ...")
        ser = serial.Serial(PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print("✅ 串口连接成功")

        send_arduino_cmd(ser, "D0")  # 设置为正转方向
        if not send_and_wait_ack(ser, f"START {FREQ}", "ACK_START"):
            print("❌ 启动失败")
            return

        stepCount = 0
        stall_id = 0
        preReset = False
        min_step_between_stalls = 2000

        print("🚩 开始扫描每个 stop 点 ...")

        while True:
            resetSensor = GPIO.input(resetPin)
            stopSensor = GPIO.input(pins["magnetPin"])
            endSensor = GPIO.input(endPin)

            # 检测 reset 起点（只触发一次）
            if resetSensor == 0 and not preReset:
                stepCount = 0
                print("🟢 检测到 reset，开始计数")
                preReset = True

            stepCount += 1
            time.sleep(0.001)

            # 检测磁点 stop（包括第一次和后续）
            if stopSensor == 0 and stepCount > min_step_between_stalls:
                if preReset:
                    print(f"📍 从 reset 到第 {stall_id} 个 stall：{stepCount} 步")
                    preReset = False
                else:
                    print(f"📍 从第 {stall_id - 1} 到第 {stall_id} 个 stall：{stepCount} 步")

                stall_id += 1
                stepCount = 0
                
                return

                # ✅ 等待磁传感器释放，防止重复触发
                while GPIO.input(pins["magnetPin"]) == 0:
                    time.sleep(0.01)

            # 检测终点
            if endSensor == 0:
                send_and_wait_ack(ser, "STOP", "ACK_STOP")
                print("🏁 已到终点，开始回退")
                ser.close()
                back_to_dock()  # 回退函数（已定义）
                break

    except Exception as e:
        print(f"❌ 错误: {e}")
    finally:
        GPIO.cleanup()



# ===== 执行入口 =====
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
    # back_to_dock()
    # count_total_distance()

    
    if args.command == "back":
        back_to_dock()
    elif args.command == "ahead":
        count_total_distance()
    # elif args.command == "reset":
    #     reset()
    elif args.command == "test":
        test_stop()
    # elif args.command == "delay":
    #     count_stop_delay()
    # elif args.command == "second":
    #     get_second_stop()
    # elif args.command == "step":
    #     get_steps()
    # elif args.command == "reverse":
    #     test_motor_reverse()
    # elif args.command == "magnet":
    #     test_position()
    # elif args.command == "cycle":
    #     test_cycle()