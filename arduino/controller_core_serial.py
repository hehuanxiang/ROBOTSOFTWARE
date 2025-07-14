# controller_core_serial.py — 使用 START/STOP 控制电机持续运动
import RPi.GPIO as GPIO
import time
from datetime import date, datetime
from time import sleep
import logging
import serial
import json
from datetime import datetime, timedelta


LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log/Sowbot_record_{}.log".format(date.today())

def send_arduino_cmd(ser, cmd):
    ser.write((cmd + '\n').encode())
    time.sleep(0.001)
    
def send_and_wait_ack(ser, cmd, expect_ack, timeout=3):
    """
    向 Arduino 发送命令，并等待特定 ACK 响应。
    - ser: 串口对象
    - cmd: 要发送的字符串命令（如 START 5000）
    - expect_ack: 预期的确认响应（如 ACK_START, ACK_STOP）
    - timeout: 最长等待时间（秒）
    """
    ser.flushInput()
    ser.write((cmd + '\n').encode())
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line == expect_ack:
                return True
            elif line.startswith("ERR") or line.startswith("SLOWDOWN"):
                continue  # 可选：打印中间状态或错误
    print(f"⏱️ 超时未收到 {expect_ack} for '{cmd}'")
    return False  # 超时未收到 ACK


def setup_gpio(pins):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pins["resetPin"], GPIO.IN)
    GPIO.setup(pins["endPin"], GPIO.IN)
    GPIO.setup(pins["magnetPin"], GPIO.IN)

def setup_motor_logger():
    logger = logging.getLogger("Sowbot")
    if not logger.handlers:
        handler = logging.FileHandler(LOG_FILE)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger

def reset(pins, logger, ser, freq):
    logger.info("🔁 开始归零流程（reset）")
    # send_arduino_cmd(ser, 'E0')     # 电机上电
    send_arduino_cmd(ser, 'D0')     # 设置为前进方向
    # send_arduino_cmd(ser, f'START {freq}')  # 开始持续运动
    
    if not send_and_wait_ack(ser, f'START {freq}', 'ACK_START'):
        logger.error("❌ 启动失败（reset 阶段未收到 ACK_START）")
        return

    stepCount = 0
    preStop = False

    while True:
        stopSensor = GPIO.input(pins["magnetPin"])
        endSensor = GPIO.input(pins["endPin"])
        time.sleep(0.001)
        stepCount += 1

        if stopSensor == 0 and stepCount > 1000:
            logger.info(f"🔍 检测到磁点，step={stepCount}")
            preStop = True

        if preStop or endSensor == 0:
            # send_arduino_cmd(ser, 'STOP')
            if not send_and_wait_ack(ser, 'STOP', 'ACK_STOP'):
                logger.warning("⚠️ 停止失败（reset 阶段未收到 ACK_STOP）")
            logger.info("↩️ 到达终点，准备回退至归位点")
            # send_arduino_cmd(ser, 'E1')     # 电机下电
            sleep(2)
            break

        if stepCount >= 200000:
            send_arduino_cmd(ser, 'STOP')
            sleep(2)
            logger.error("❌ Reset 步数超限，异常中止")
            return

    # send_arduino_cmd(ser, 'E0')     # 上电
    send_arduino_cmd(ser, 'D1')  # 设置为回退方向
    # send_arduino_cmd(ser, f'START {freq}')
    if not send_and_wait_ack(ser, f'START {freq}', 'ACK_START'):
        logger.error("❌ 回退启动失败（未收到 ACK_START）")
        return
    while GPIO.input(pins["resetPin"]) != 0:
        time.sleep(0.001)
    # send_arduino_cmd(ser, 'STOP')
    if not send_and_wait_ack(ser, 'STOP', 'ACK_STOP'):
        logger.warning("⚠️ 回退停止失败（未收到 ACK_STOP）")
    # send_arduino_cmd(ser, 'E1')     # 电机下电
    sleep(2)
    logger.info("✅ 成功回到归位点（resetPin）")
    
def wait_until_next_five_minutes(logger):
    now = datetime.now()
    next_minute = (now.minute // 5 + 1) * 5
    next_time = now.replace(minute=0, second=0, microsecond=0) + timedelta(minutes=next_minute)
    sleep_seconds = (next_time - now).total_seconds()
    logger.info(f"⏳ 当前时间 {now.strftime('%H:%M:%S')}，等待 {sleep_seconds:.1f} 秒直到下一个 5 分钟周期（{next_time.strftime('%H:%M:%S')}）...")
    time.sleep(sleep_seconds)

def run_motor(pins, stallNumber, pig_ids, queue, logger, stop_event, port, freq=5000):
    setup_gpio(pins)
    
    

    # ✅ 多次尝试连接串口
    max_retries = 15
    delay_between = 2
    ser = None

    for attempt in range(1, max_retries + 1):
        try:
            logger.info(f"🔌 [尝试 {attempt}/{max_retries}] 连接 Arduino ({port}) ...")
            ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)  # 等 Arduino 初始化
            logger.info(f"✅ 成功连接 Arduino 串口: {port}")
            break
        except Exception as e:
            logger.warning(f"⚠️ [尝试 {attempt}/{max_retries}] 连接失败: {e}")
            time.sleep(delay_between)

    if ser is None or not ser.is_open:
        logger.error("❌ 多次尝试连接 Arduino 串口失败，放弃启动电机控制")
        return
    
    send_arduino_cmd(ser, "SETTIME 800")     # 设置加/减速时间为 2 秒
    # send_arduino_cmd(ser, 'E0')  # 电机上电
    reset(pins, logger, ser, freq)

    send_arduino_cmd(ser, 'D0')  # 设置方向为正转

    stall_id = 0
    cycle_count = 0
    cycle_start_time = None
    detected_stalls = set()

    while not stop_event.is_set():
        wait_until_next_five_minutes(logger)
        
        cycle_start_time = time.time()
        logger.info(f"⏱️ 开始新周期 {cycle_count+1}，时间 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        # 每一轮都重新上电并设置方向
        # send_arduino_cmd(ser, 'E0')  # 上电
        send_arduino_cmd(ser, 'D0')  # 正转
        logger.info(f"🚶 正在前往 Stall_{stall_id}")
        stepCount = 0

        # send_arduino_cmd(ser, f'START {freq}')
        if not send_and_wait_ack(ser, f'START {freq}', 'ACK_START'):
            logger.warning("⚠️ 启动失败（未收到 ACK_START）")

        while True:
            time.sleep(0.001)  # 1ms 轮询磁点
            stepCount += 1
            # if GPIO.input(pins["magnetPin"]) == 0:
            #     print(f"🔍 检测到磁点，step={stepCount}")
            if GPIO.input(pins["magnetPin"]) == 0 and stepCount > 1500:
                # send_arduino_cmd(ser, 'STOP')
                logger.info(f"📍 Stall_{stall_id} 检测到磁铁，step={stepCount}")

                try:
                    queue.put_nowait({
                        "stall": stall_id,
                        "pig_id": pig_ids[stall_id]
                    })
                    logger.debug(f"📸 添加任务: stall={stall_id}, pig_id={pig_ids[stall_id]}")
                except Exception as e:
                    logger.warning(f"⚠️ 加入队列失败: {e}")

                detected_stalls.add(stall_id)
                stall_id = (stall_id + 1) % stallNumber
                stepCount = 0  # 重置步数计数

                # if stall_id == 0 and cycle_start_time is not None:
                #     cycle_count += 1
                #     elapsed = time.time() - cycle_start_time
                #     logger.info(f"✅ 完成第 {cycle_count} 轮数据采集，用时 {elapsed:.2f}s，共检测到 {len(detected_stalls)} 个检查点")
                #     detected_stalls = set()  # ✅ 完成后重置集合
                # break

            if GPIO.input(pins["endPin"]) == 0:
                # send_arduino_cmd(ser, 'STOP')
                if not send_and_wait_ack(ser, 'STOP', 'ACK_STOP'):
                    logger.warning("⚠️ 停止失败（未收到 ACK_STOP）")

                logger.info("🚧 到达轨道末端，暂停 3 秒...")
                sleep(3)

                logger.info("↩️ 开始回退归位...")
                send_arduino_cmd(ser, 'D1')  # 方向设为回退
                # send_arduino_cmd(ser, f'START {freq}')
                if not send_and_wait_ack(ser, f'START {freq}', 'ACK_START'):
                    logger.warning("⚠️ 回退启动失败")

                while GPIO.input(pins["resetPin"]) != 0:
                    time.sleep(0.001)
                # send_arduino_cmd(ser, 'STOP')
                
                if not send_and_wait_ack(ser, 'STOP', 'ACK_STOP'):
                    logger.warning("⚠️ 回退停止失败")


                logger.info("✅ 成功回到 resetPin（归位点）")
                
                sleep(5)
                # send_arduino_cmd(ser, 'E1')  # ⛔ 电机断电

                stall_id = 0
                break
        
        
        cycle_count += 1
        elapsed = time.time() - cycle_start_time
        logger.info(f"✅ 完成周期 {cycle_count}，耗时 {elapsed:.2f}s, 共检测到 {len(detected_stalls)} 个检查点")
        detected_stalls = set()  # ✅ 完成后重置集合

            
    # 清理资源：串口关闭
    if ser.is_open:
        ser.close()
        logger.info("🔒 已关闭串口连接")
