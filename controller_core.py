# controller_core.py
# 电机控制主逻辑 + reset 函数复位 + 任务队列通信 + 日志与邮件告警

import RPi.GPIO as GPIO
import time
from datetime import date, datetime
from time import sleep
import logging
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.header import Header
import smtplib
import json

# 设置统一的日志文件名
LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log/Sowbot_record_{}.log".format(date.today())

def load_config():
    # 加载 farm_config.json 配置文件
    with open("/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json", "r") as f:
        return json.load(f)


def send_alert_email(body="设备在复位过程中超过200000步，可能存在故障，请及时检查。"):
    mail_host = 'smtp.gmail.com'
    mail_user = 'hxh0326@gmail.com'
    mail_pass = 'pnkvseixcwzvlqlu'
    mail_to = 'hxh0326@gmail.com'
    subject = "设备异常通知"

    try:
        message = MIMEMultipart()
        message['From'] = mail_user
        message['To'] = mail_to
        message['Subject'] = Header(subject, 'utf-8')
        message.attach(MIMEText(body, 'plain', 'utf-8'))

        server = smtplib.SMTP(mail_host, 587)
        server.starttls()
        server.login(mail_user, mail_pass)
        server.sendmail(mail_user, mail_to, message.as_string())
        server.quit()
        print("📧 邮件发送成功")
    except Exception as e:
        print(f"❌ 邮件发送失败: {e}")

def setup_motor_logger():
    logger = logging.getLogger("Sowbot")
    if not logger.handlers:
        handler = logging.FileHandler(LOG_FILE)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger

def setup_gpio(pins):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pins["DIR"], GPIO.OUT)
    GPIO.setup(pins["STEP"], GPIO.OUT)
    GPIO.setup(pins["ENA"], GPIO.OUT)
    GPIO.setup(pins["resetPin"], GPIO.IN)
    GPIO.setup(pins["endPin"], GPIO.IN)
    GPIO.setup(pins["magnetPin"], GPIO.IN)

def reset(pins, logger, waitTime):
    logger.info("Starting reset sequence...")
    GPIO.output(pins["DIR"], 0)
    GPIO.output(pins["ENA"], GPIO.LOW)

    stepCount = 0
    maxStepCount = 200000
    preStop = False

    while True:
        stopSensor = GPIO.input(pins["magnetPin"])
        endSensor = GPIO.input(pins["endPin"])

        GPIO.output(pins["STEP"], True)
        sleep(waitTime)
        GPIO.output(pins["STEP"], False)
        sleep(waitTime)
        stepCount += 1

        if stopSensor == 0 and stepCount > 10000:
            logger.info(f"Passed magnet after {stepCount} steps")
            preStop = True

        if preStop or endSensor == 0:
            logger.info("Reached end. Reversing to dock...")
            break

        if stepCount >= maxStepCount:
            logger.error("Reset step count exceeded 200000. Reset failed.")
            send_alert_email("Reset 超过200000步，未能归零，请检查设备")
            return

    # Reverse to resetPin
    GPIO.output(pins["DIR"], 1)
    while GPIO.input(pins["resetPin"]) != 0:
        GPIO.output(pins["STEP"], True)
        sleep(waitTime)
        GPIO.output(pins["STEP"], False)
        sleep(waitTime)

    logger.info("Reset completed. Reached dock.")

def back_to_dock(pins, logger, waitTime):
    GPIO.output(pins["DIR"], 1)
    logger.info("Returning to dock...")
    while GPIO.input(pins["resetPin"]) != 0:
        GPIO.output(pins["STEP"], True)
        sleep(waitTime)
        GPIO.output(pins["STEP"], False)
        sleep(waitTime)
    sleep(2)
    logger.info("Arrived at dock.")

def run_motor(pins, stallNumber, pig_ids, queue, logger, stop_event):

    setup_gpio(pins)
    waitTime = 0.000001
    reset(pins, logger, waitTime)

    GPIO.output(pins["ENA"], GPIO.LOW)
    GPIO.output(pins["DIR"], 0)

    stall_id = 0
    maxStepCount = 200000
    

    cycle_count = 0
    cycle_start_time = None
    cycle_start_fmt = None
    detected_stalls = set()

    while True:
        GPIO.output(pins["DIR"], 0)
        stepCount = 0
        logger.info(f"Moving toward Stall_{stall_id}")
        
        

        while True:
            GPIO.output(pins["STEP"], True)
            sleep(waitTime)
            GPIO.output(pins["STEP"], False)
            sleep(waitTime)
            stepCount += 1

            if GPIO.input(pins["magnetPin"]) == 0:
                # print(stepCount)
                if stepCount > 2000:
                    logger.info(f"Detected magnet sensor at Stall_{stall_id} after {stepCount} steps, triggering image capture")
                    stepCount = 0
                    

                    try:
                        queue.put_nowait({
                            "stall": stall_id,
                            "pig_id": pig_ids[stall_id]
                        })
                        logger.debug(f"Capture task queued: stall={stall_id}, pig_id={pig_ids[stall_id]}")
                    except Exception as e:
                        logger.warning(f"Failed to enqueue capture task: {e}")

                    # 🚀 出发时（完成 Stall_0）开始计时
                    if stall_id == 0:
                        cycle_start_time = time.time()
                        cycle_start_fmt = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        detected_stalls = set()  # 新周期，重置
                        # detected_stalls.add(stall_id)
                        
                    detected_stalls.add(stall_id)

                    stall_id = (stall_id + 1) 

                    # 🏁 回到 Stall_0 说明周期结束
                    if stall_id == 0 and cycle_start_time is not None:
                        cycle_count += 1
                        cycle_end_time = time.time()
                        cycle_end_fmt = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        elapsed = cycle_end_time - cycle_start_time

                        detected_count = len(detected_stalls)
                        # print(detected_stalls)
                        if detected_count == stallNumber:
                            logger.info(f"✅ Completed cycle {cycle_count} | All {stallNumber} stalls detected | Start: {cycle_start_fmt} | End: {cycle_end_fmt} | Elapsed: {elapsed:.2f}s")
                        else:
                            logger.warning(f"⚠️ Completed cycle {cycle_count} | Only detected {detected_count}/{stallNumber} stalls! | Start: {cycle_start_fmt} | End: {cycle_end_fmt} | Elapsed: {elapsed:.2f}s")
                            if detected_count < stallNumber:
                                send_alert_email(f"本轮周期仅检测到 {detected_count}/{stallNumber} 个磁点，可能存在传感器问题")

                        if cycle_count % 20 == 0:
                            logger.info(f"🌙 Completed {cycle_count} cycles. Taking a 60-second rest at Stall_0...")
                            sleep(60)
                        
                        cycle_start_time = None
                        cycle_start_fmt = None
                        detected_stalls = set()

                    break

            if GPIO.input(pins["endPin"]) == 0:
                logger.info("Reached end of rail. Returning to dock.")
                sleep(2)
                back_to_dock(pins, logger, waitTime)
                stall_id = 0

                # 🛑 若意外中止，也可记录一次周期（若开始过）
                if cycle_start_time is not None:
                    cycle_count += 1
                    cycle_end_time = time.time()
                    cycle_end_fmt = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    elapsed = cycle_end_time - cycle_start_time

                    detected_count = len(detected_stalls)
                    if detected_count == stallNumber:
                        logger.info(f"✅ Completed cycle {cycle_count} (via endPin) | All {stallNumber} stalls detected | Start: {cycle_start_fmt} | End: {cycle_end_fmt} | Elapsed: {elapsed:.2f}s")
                    else:
                        logger.warning(f"⚠️ Completed cycle {cycle_count} (via endPin) | Only detected {detected_count}/{stallNumber} stalls! | Start: {cycle_start_fmt} | End: {cycle_end_fmt} | Elapsed: {elapsed:.2f}s")
                        # if detected_count < stallNumber * 0.8:
                            # send_alert_email(f"本轮周期仅检测到 {detected_count}/{stallNumber} 个磁点，可能存在传感器问题")

                    cycle_start_time = None
                    cycle_start_fmt = None
                    detected_stalls = set()

                break

            if stepCount >= maxStepCount:
                logger.error("Step count exceeded threshold. Triggering fault handling.")
                send_alert_email("步数超过上限，疑似卡住，已停止并归位")
                GPIO.output(pins["ENA"], GPIO.HIGH)
                back_to_dock(pins, logger)
                stall_id = 0
                stop_event.set()
                break


# if __name__ == "__main__":
#     config = load_config()
#     pins = config["pins"]
#     pig_ids = config["pigNumber"]
#     stallNumber = config["stallNumber"]
    
#     run_motor(pins, stallNumber, pig_ids)