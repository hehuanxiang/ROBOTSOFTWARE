# test_controller.py
import json
import multiprocessing
from controller_core_pigpio import run_motor, setup_motor_logger
from camera_core import run_camera_system, setup_camera_logger
import time

if __name__ == "__main__":
    with open("/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json", "r") as f:
        config = json.load(f)

    pins = config["pins"]
    pig_ids = config["pigNumber"]
    stallNumber = config["stallNumber"]

    logger = setup_motor_logger()
    queue = multiprocessing.Manager().Queue(maxsize=10)

    # 启动控制逻辑（仅控制电机 + 填充任务）
    run_motor(pins, stallNumber, pig_ids, queue, logger)


    # queue = multiprocessing.Queue()
    # logger = setup_camera_logger()

    # # 启动相机系统进程（只传 queue 和 logger）
    # camera_proc = multiprocessing.Process(target=run_camera_system, args=(queue, logger))
    # camera_proc.start()

    # # 模拟任务输入
    # pig_ids = {0: "Pig_A", 1: "Pig_B"}
    # time.sleep(1)
    # queue.put({"stall": 0, "pig_id": pig_ids[0]})
    # time.sleep(3)
    # queue.put({"stall": 1, "pig_id": pig_ids[1]})
    # time.sleep(3)

    # camera_proc.terminate()
    # camera_proc.join()
    
    # 可在此打印测试输出（排查）
    # while not queue.empty():
    #     print(queue.get())
