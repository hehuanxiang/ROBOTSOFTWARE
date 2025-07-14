import multiprocessing
import os
import json
from controller_core import run_motor
from camera_core import run_camera_system
from logger_setup_old import setup_logger

def load_config():
    with open("/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json", "r") as f:
        return json.load(f)

def start_motor(pins, stallNumber, pig_ids, queue, logger, stop_event):
    os.sched_setaffinity(0, {0})  # 绑定到 CPU 0
    run_motor(pins, stallNumber, pig_ids, queue, logger, stop_event)

def start_camera(queue, logger, stop_event):
    os.sched_setaffinity(0, {1, 2, 3})  # 相机绑 CPU 1,2,3
    run_camera_system(queue, logger, stop_event)

if __name__ == "__main__":
    config = load_config()
    pins = config["pins"]
    pig_ids = config["pigNumber"]
    stallNumber = config["stallNumber"]

    manager = multiprocessing.Manager()
    task_queue = manager.Queue(maxsize=20)
    stop_event = multiprocessing.Event()  # ❗共享退出信号

    motor_logger = setup_logger("motor")
    camera_logger = setup_logger("camera")

    motor_proc = multiprocessing.Process(
        target=start_motor,
        args=(pins, stallNumber, pig_ids, task_queue, motor_logger, stop_event)
    )

    camera_proc = multiprocessing.Process(
        target=start_camera,
        args=(task_queue, camera_logger, stop_event)
    )

    motor_logger.info("Starting motor controller on CPU 0")
    camera_logger.info("Starting camera system on CPU 1, 2, 3")

    motor_proc.start()
    camera_proc.start()

    motor_proc.join()
    camera_proc.join()

    motor_logger.info("Motor process ended")
    camera_logger.info("Camera process ended")
