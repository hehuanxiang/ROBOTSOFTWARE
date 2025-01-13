import io, serial, serial.tools.list_ports, socket, struct, sys, datetime, os
import RPi.GPIO as GPIO
from PIL import Image
from time import sleep
import threading
import queue
from Stepper import Stepper
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from pyntcloud import PyntCloud
import argparse
import imageio
from realsense_device_manager import DeviceManager, post_process_depth_frame
import time
from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate
import scipy.io
import traceback
import json
import logging
import multiprocessing
from logging.handlers import QueueHandler
import psutil


def setup_logger(logger_name, log_queue):
    logger = logging.getLogger(logger_name)
    if not logger.handlers:  # 如果没有已存在的 Handler，则添加
        handler = logging.handlers.QueueHandler(log_queue)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger

def callback(threadID, result, log_queue):
    """回调函数，在线程完成后调用"""
    logger = setup_logger(f"saveDataThread-{threadID}", log_queue)
    logger.info(f"线程 {threadID} 已完成，结果: {result}")

def save_data(frameset, i, path, PIG_ID, time_stamp, intr, log_queue, stallId, callback=None):
    """保存数据的主函数"""
    logger = setup_logger(f"saveData-{stallId}", log_queue)
    data_saved = False  # 初始为 False，判断是否成功保存数据

    try:
        # 设置裁剪深度
        depth_threshold = rs.threshold_filter(min_dist=0.1, max_dist=2.5)
        colorizer = rs.colorizer()
        t = time_stamp
        j = 1
        imgname = f"{PIG_ID}_{t.year}_{t.month}_{t.day}_{t.hour}_{t.minute}_{t.second}_"
        logger.info(f"开始保存 Stall_{stallId}，猪 ID: {PIG_ID} 的数据到路径: {path}")

        index = 0

        for frame in frameset:
            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()
            ir_frame = frame.get_infrared_frame()

            # 深度图像
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_image = np.asanyarray(ir_frame.get_data())

            colorized = colorizer.process(ir_frame)

            name = imgname + str(j)
            XX = np.zeros((480, 640))
            YY = np.zeros((480, 640))
            ZZ = np.zeros((480, 640))

            for y in range(480):
                for x in range(480):
                    dist = depth_frame.get_distance(x + 80, y)
                    [X, Y, Z] = convert_depth_pixel_to_metric_coordinate(dist, x, y, intr)
                    XX[y, x] = X
                    YY[y, x] = Y
                    ZZ[y, x] = Z

            obj = np.stack((XX, YY, ZZ))

            if not os.path.exists(path + "DM"):
                os.makedirs(path + "DM")

            if not os.path.exists(path + "depth"):
                os.makedirs(path + "depth")

            if not os.path.exists(path + "RGB"):
                os.makedirs(path + "RGB")

            if not os.path.exists(path + "IR"):
                os.makedirs(path + "IR")

            matfile = path + f"DM/{name}_{index + 1}.mat"
            scipy.io.savemat(matfile, mdict={'out': obj}, oned_as='row')
            imageio.imwrite(path + f"depth/{name}_{index}.png", depth_image)
            imageio.imwrite(path + f"RGB/{name}_{index}.png", color_image)
            imageio.imwrite(path + f"IR/{name}_{index}.png", ir_image)

            index += 1

        if callback:
            callback(None, f"Stall_{stallId}，猪 ID {PIG_ID} 数据保存完成", log_queue)

        data_saved = True

    except Exception as e:
        data_saved = False
        logger.error(f"Stall_{stallId}，保存猪 ID {PIG_ID} 数据时发生错误", exc_info=True)
        raise RuntimeError(f"数据未成功保存，Stall_{stallId}，猪_{PIG_ID}")

    return data_saved

            

def streamSensor(pigID, cameraPipeline, profile, stallId, log_queue):
    logger = setup_logger("streamSensor", log_queue)
    
    pipeline = cameraPipeline
    profile = profile
    pig_ID=pigID

    # str.zfill(2)：
    # 将 pigID 转换为字符串，不足两位时在前面填充 0。
    # 示例：1.zfill(2) -> "01"。
    path = f"./Data/Data_Estrus_2024_12/attention/ID_{str(stallId).zfill(2)}_{str(pigID)}/"
    try:
        os.makedirs(path, exist_ok=True)
        logger.info(f"确保文件夹已存在: {path}")
    except Exception as e:
        logger.error(f"创建文件夹失败: {path}", exc_info=True)
        
    try:
        colorizer=rs.colorizer()

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor.set_option(rs.option.min_distance,0)
        depth_sensor.set_option(rs.option.enable_max_usable_range,0)
        logger.info("成功配置深度传感器")
        
        depth_scale = depth_sensor.get_depth_scale()
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
        logger.info(f"获取相机内参:{intr}")
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 3.25 #1 meter
        clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        align_to = rs.stream.depth
        align = rs.align(align_to)

        tt = time.monotonic()
        t = datetime.datetime.now()

        frameset=[]
        interval=30

        logger.info(f"进行处理猪 ID: {pigID} 在位置 {stallId}，开始捕获 {interval} 帧深度图像")
        for x in range(interval*1):
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image
            st =int(time.monotonic()-tt)
            if st >= 21:
                print("captured failed")
                break
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            #aligned_ir_frame=aligned_frames.get_infrared_frame()
            #color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame:
                logger.warning(f"第 {x+1} 帧对齐失败，跳过")
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            key = cv2.waitKey(1)

            if x%interval==0: #60
                frameset.append(aligned_frames)
                logger.info("深度图像捕获完成")
                
        dataSaved = save_data(frameset,int(x/interval),path, pig_ID, t,intr, log_queue, stallId, callback=callback)
    except:   
        # pipeline.stop()
        logger.info("相机无法正常获取图像")


def setupCamera():
    cameraPipeline = rs.pipeline()
    cameraConfig = rs.config()
    cameraConfig.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cameraConfig.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cameraConfig.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

    return cameraPipeline, cameraConfig

def handle_stop(sign, testStepper):
    # print("Get in the handle_stop function.")
    testStepper = testStepper
    if sign != None:
        if sign == "docked":
            #move forward slightly
            print("docked2")
            action = testStepper.step(3000, "forward", 0.05, docking = False)
            handle_stop(action, testStepper)
            
        elif sign == "right_end":
            print("end3")
            #action = testStepper.step(5000, "right", 50, docking = False)
            action = testStepper.step(10000000, "back", 50, docking = True)        
            handle_stop(action, testStepper)

def logger_process(log_queue):
    """日志处理进程"""
    time = datetime.datetime.now()
    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s - %(levelname)s - %(message)s",
        filename=f"robot_log/{time.date()}_robot_log.log",
        filemode="a"
    )
    logger = logging.getLogger()
    while True:
        try:
            record = log_queue.get()
            if record is None:  # None 为结束信号
                break
            logger.handle(record)
        except Exception as e:
            print(f"日志进程错误: {e}")

def Camera_start(log_queue, max_retries = 3, retry_delay = 30):
    logger = setup_logger("streamSensor", log_queue)
    cameraPipeline, cameraConfig = setupCamera()
    for attempt in range(max_retries):
        try:
            logger.info(f"尝试启动相机pipeline (第 {attempt + 1} 次)")
            profile = cameraPipeline.start(cameraConfig)
            logger.info("相机启动成功")
            
            return profile, cameraPipeline
        
        except Exception as e:
            logger.error(f"相机pipeline启动失败 (第 {attempt + 1} 次): {e}")
            
            # 如果还未达到最大重试次数，进行清理并等待
            if attempt < max_retries - 1:
                try:
                    logger.info("尝试停止相机pipeline以清理状态")
                    cameraPipeline.stop()
                except Exception as stop_error:
                    logger.warning(f"相机pipeline停止失败: {stop_error}")
                
                logger.info(f"{retry_delay} 秒后重试...")
                time.sleep(retry_delay)  # 等待后再次尝试
            else:
                logger.critical("多次尝试启动相机pipeline均失败，放弃重试")
                raise e  # 抛出异常以供上层处理

# Function to initialize GPIO
def initialize_gpio(pins):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pins["DIR"], GPIO.OUT)
    GPIO.setup(pins["ENA"], GPIO.OUT)
    GPIO.setup(pins["STEP"], GPIO.OUT)
    GPIO.output(pins["ENA"], GPIO.HIGH)
    logging.info("GPIO initialized successfully.")

# Function to configure the depth sensor
def configure_depth_sensor(profile):
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.min_distance, 0)
    depth_sensor.set_option(rs.option.enable_max_usable_range, 0)
    logging.info("Depth sensor configured successfully.")
    
    depth_scale = depth_sensor.get_depth_scale()
    intrinsic = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    logging.info(f"Camera Intrinsics: {intrinsic}")

    return depth_scale, intrinsic

# Function to create a folder if it doesn't exist
def ensure_folder_exists(path):
    try:
        os.makedirs(path, exist_ok=True)
        logging.info(f"Folder ensured: {path}")
    except Exception as e:
        logging.error(f"Failed to create folder: {path}", exc_info=True)

def main(log_queue):
    """主程序逻辑"""
    logger = logging.getLogger("main")
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.setLevel(logging.DEBUG)
    
    target_stall = 1

    try:
        logger.info("加载 farm_config.json 配置文件")
        with open('/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json', 'r') as file:
            farm_config = json.load(file)

        pigNumber = farm_config["pigNumber"]
        pins = farm_config["pins"]


        logger.info("start to initialize GPIO")

        logger.info("初始化步进电机")
        testStepper = Stepper([pins["STEP"], pins["DIR"], pins["ENA"], pins["endPin"], pins["resetPin"], pins["magnetPin"]], log_queue)

        logger.info("程序启动，等待 2 秒...")
        sleep(2)

        action = testStepper.step(40000 * 10, "forward", 0.1, docking=True)
        logger.info(f"首次步进操作结果: {action}")
        handle_stop(action, testStepper)
        logger.info("返回对接位置")

        
        
        stallNumber = farm_config["stallNumber"]
        
        cycle = 0
        
        # 定义每个成像周期的计数器和休息逻辑
        start_cycle_time = time.time()  # 记录循环开始的时间
        rest_interval = 5 * 60 * 60  # 每隔5小时 (单位为秒)
        rest_duration = 10 * 60  # 休息5分钟 (单位为秒)
        
        for i in range(stallNumber):
            try:
                if i == 0:
                    logger.info(f"快速接近：Stall_{i}")
                    # action = testStepper.step(90000, "forward", 1, docking=False)
                    action = testStepper.step(55000, "forward", 1, docking=False)
                    logger.info(f"即将抵达：Stall_{i}，减速接近")
                    action = testStepper.step(35000, "forward", 0.05, docking=False)
                else:
                    logger.info(f"快速接近：Stall_{i}")
                    # action = testStepper.step(70000, "forward", 1, docking=False)
                    action = testStepper.step(25000, "forward", 1, docking=False)
                    logger.info(f"即将抵达：Stall_{i}，减速接近")
                    action = testStepper.step(30000, "forward", 0.05, docking=False)

                handle_stop(action, testStepper)
                logger.info(f"抵达：Stall_{i}")
                
                if i == target_stall:
                    logger.info(f"Arrived at target_stall, ready to monitor Piggy in Stall_{i}，ID: {pigNumber[target_stall]} ")
                    break
            except:
                pass
            
        profile, cameraPipeline = Camera_start(log_queue)
        
        path = f"./Data/Data_Estrus_2024_12/attention/ID_{str(target_stall).zfill(2)}_{str(pigNumber[target_stall])}/"
        ensure_folder_exists(path)
        
        while True:
            streamSensor(pigNumber[target_stall], cameraPipeline, profile, target_stall, log_queue)
            logger.info("Restart the system after 30s.")
            sleep(30)
        
    except Exception as e:
        logger.critical("主程序发生致命错误", exc_info=True)
            
    finally:
        GPIO.cleanup()
        logger.info("清理 GPIO 引脚，程序结束")

if __name__ == "__main__":
    log_queue = multiprocessing.Queue()
    log_process = multiprocessing.Process(target=logger_process, args=(log_queue,))
    log_process.start()

    main(log_queue)

    log_queue.put(None)  # 结束信号
    log_process.join()