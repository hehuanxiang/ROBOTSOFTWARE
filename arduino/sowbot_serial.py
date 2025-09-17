import multiprocessing
import os
import json
import signal
from controller_core_serial import run_motor  # ✅ 使用新版 controller_core
from camera_core import run_camera_system
import serial
import serial.tools.list_ports 
from time import sleep
from logger_setup import get_file_logger
from logging.handlers import QueueHandler, QueueListener
import logging
from multiprocessing import Queue
from datetime import datetime


def setup_worker_logger(log_queue, submodule_name):
    logger = logging.getLogger(f"Sowbot.{submodule_name}")
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    logger.propagate = False
    logger.addHandler(QueueHandler(log_queue))
    return logger



def load_config():
    with open("/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json", "r") as f:
        return json.load(f)

def start_motor(pins, stallNumber, pig_ids, queue, logger, stop_event, port):
    os.sched_setaffinity(0, {3})  # 绑定 CPU 3
    run_motor(pins, stallNumber, pig_ids, queue, logger, stop_event, port)

def start_camera(queue, logger, stop_event):
    os.sched_setaffinity(0, {0, 1, 2})  # 相机绑 CPU 0-2
    run_camera_system(queue, logger, stop_event)
    
    # ✅ 动态查找 Arduino 端口
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "Arduino" in port.description or "ttyACM" in port.device or "ttyUSB" in port.device:
            return port.device
    return None

if __name__ == "__main__":
    config = load_config()
    pins = config["pins"]
    pig_ids = config["pigNumber"]
    stallNumber = config["stallNumber"]

    manager = multiprocessing.Manager()
    task_queue = manager.Queue(maxsize=20)
    stop_event = multiprocessing.Event()

    log_queue = Queue()
    main_logger = get_file_logger()
    log_listener = QueueListener(log_queue, *main_logger.handlers)
    log_listener.start()

    motor_logger = setup_worker_logger(log_queue, "motor")
    camera_logger = setup_worker_logger(log_queue, "camera")
    
    # sleep(10)       # 等待arduino启动
    port = find_arduino_port()
    if port is None:
        raise Exception("Arduino port not found")

    motor_proc = multiprocessing.Process(
        target=start_motor,
        args=(pins, stallNumber, pig_ids, task_queue, motor_logger, stop_event, port)
    )

    camera_proc = multiprocessing.Process(
        target=start_camera,
        args=(task_queue, camera_logger, stop_event)
    )

    try:
        motor_logger.info("🟢 Starting motor controller on CPU 3")
        camera_logger.info("🟢 Starting camera system on CPU 0, 1, 2")

        motor_proc.start()
        camera_proc.start()

        motor_proc.join()
        camera_proc.join()

    except KeyboardInterrupt:
        motor_logger.warning("🛑 KeyboardInterrupt received. Stopping...")
        stop_event.set()
        motor_proc.terminate()
        camera_proc.terminate()
        motor_proc.join()
        camera_proc.join()

        # 可选：发送 STOP 和 E1 指令，确保电机停下（需额外打开串口）
        try:
            
            ser = serial.Serial(port, 9600, timeout=1)
            ser.write(b'STOP\n')
            ser.write(b'E1\n')
            ser.close()
            motor_logger.info("🛑 Emergency STOP and DISABLE sent to Arduino.")
        except Exception as e:
            motor_logger.warning(f"⚠️ Failed to stop motor via serial: {e}")
    finally:
        motor_logger.info("✅ Motor process ended")
        camera_logger.info("✅ Camera process ended")
        log_listener.stop()  
