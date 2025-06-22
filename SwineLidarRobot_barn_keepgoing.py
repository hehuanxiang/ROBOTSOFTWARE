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
from concurrent.futures import ThreadPoolExecutor
import argparse
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.header import Header
from time import sleep


def setup_logger(logger_name, log_queue):
    logger = logging.getLogger(logger_name)
    if not logger.handlers:  # 如果没有已存在的 Handler，则添加
        handler = logging.handlers.QueueHandler(log_queue)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger


def save_data(frameset, i, path, PIG_ID, time_stamp, intr, log_queue, stallId, callback=None):
    """
    同步执行数据保存任务，不使用线程。
    """
    try:
        # 日志设置
        logger = setup_logger(f"saveData-Stall_{stallId}", log_queue)
        logger.info(f"开始保存 Stall_{stallId}，猪 ID: {PIG_ID} 的数据到路径: {path}")

        # 相机内参和路径设置
        depth_threshold = rs.threshold_filter(min_dist=0.1, max_dist=2.5)
        colorizer = rs.colorizer()
        t = time_stamp
        j = 1
        imgname = f"{PIG_ID}_{t.year}_{t.month}_{t.day}_{t.hour}_{t.minute}_{t.second}"

        for frame in frameset:
            # 提取帧数据
            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()
            ir_frame = frame.get_infrared_frame()

            # 转换帧为 numpy 数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_image = np.asanyarray(ir_frame.get_data())

            # 计算点云数据
            XX, YY, ZZ = np.zeros((480, 640)), np.zeros((480, 640)), np.zeros((480, 640))
            for y in range(480):
                for x in range(640):  # 修正错误：将 x 范围改为 640
                    dist = depth_frame.get_distance(x, y)
                    X, Y, Z = convert_depth_pixel_to_metric_coordinate(dist, x, y, intr)
                    XX[y, x] = X
                    YY[y, x] = Y
                    ZZ[y, x] = Z

            obj = np.stack((XX, YY, ZZ))

            # 创建保存路径
            os.makedirs(os.path.join(path, "DM"), exist_ok=True)
            os.makedirs(os.path.join(path, "depth"), exist_ok=True)
            os.makedirs(os.path.join(path, "RGB"), exist_ok=True)
            os.makedirs(os.path.join(path, "IR"), exist_ok=True)

            # 保存数据
            matfile = os.path.join(path, "DM", f"{imgname}.mat")
            scipy.io.savemat(matfile, mdict={"out": obj}, oned_as="row")
            imageio.imwrite(os.path.join(path, "depth", f"{imgname}.png"), depth_image)
            imageio.imwrite(os.path.join(path, "RGB", f"{imgname}.png"), color_image)
            imageio.imwrite(os.path.join(path, "IR", f"{imgname}.png"), ir_image)

            logger.info(f"成功保存第 {j} 帧数据：{imgname}{j}")
            j += 1

        logger.info(f"完成保存 Stall_{stallId}，猪 ID {PIG_ID} 的数据。")
        return True

    except Exception as e:
        logger = setup_logger(f"saveData-Stall_{stallId}", log_queue)
        logger.error(f"保存 Stall_{stallId}，猪 ID {PIG_ID} 数据时发生错误", exc_info=True)
        return False

def build_point_cloud_and_save_as_mat(depth_image_path, intrinsics, output_path):
    """
    根据深度图和相机内参构建点云数据，并保存为 .mat 文件。

    参数:
        depth_image_path (str): 深度图像文件的路径（.png）。
        intrinsics (rs.intrinsics): RealSense相机的内参，用于像素坐标到三维坐标的转换。
        output_path (str): 输出 .mat 文件的路径。
    """
    # 读取深度图
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    if depth_image is None:
        raise FileNotFoundError(f"深度图像未找到: {depth_image_path}")

    # 获取深度图的形状
    height, width = depth_image.shape

    # 生成像素网格
    y, x = np.meshgrid(np.arange(height), np.arange(width), indexing='ij')

    # 将深度图像数据转换为实际深度值（单位：米）
    depth_scale = 0.001  # 假设深度图保存时以毫米为单位（根据实际情况调整）
    depth_values = depth_image * depth_scale

    # 初始化点云数组
    XX = np.zeros_like(depth_values, dtype=np.float32)
    YY = np.zeros_like(depth_values, dtype=np.float32)
    ZZ = np.zeros_like(depth_values, dtype=np.float32)

    # 将像素坐标和深度值转换为点云坐标
    for i in range(height):
        for j in range(width):
            depth = depth_values[i, j]
            if depth > 0:  # 确保深度值有效
                X, Y, Z = rs.rs2_deproject_pixel_to_point(intrinsics, [j, i], depth)
                XX[i, j] = X
                YY[i, j] = Y
                ZZ[i, j] = Z

    # 保存点云数据到 .mat 文件
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    scipy.io.savemat(output_path, {"XX": XX, "YY": YY, "ZZ": ZZ})
    print(f"点云数据已保存到: {output_path}")

def streamSensorLite(pigID, cameraPipeline, profile, stallId, log_queue):
    logger = setup_logger("streamSensor", log_queue)
    logger.info("采集数据")
    pipeline = cameraPipeline
    profile = profile
    pig_ID=pigID

    # str.zfill(2)：
    # 将 pigID 转换为字符串，不足两位时在前面填充 0。
    # 示例：1.zfill(2) -> "01"。
    path = f"/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_05/ID_{str(stallId).zfill(2)}_{str(pigID)}/"
    try:
        os.makedirs(path, exist_ok=True)
        logger.info(f"确保文件夹已存在: {path}")
    except Exception as e:
        logger.error(f"创建文件夹失败: {path}", exc_info=True)

    frameset=[]

    t = datetime.datetime.now()
    logger.info(f"进行处理猪 ID: {pigID} 在位置 {stallId}，开始捕获图像")
    imgname = f"{pig_ID}_{t.year}_{t.month}_{t.day}_{t.hour}_{t.minute}_{t.second}"
    get_intrinsic = profile.get_stream(rs.stream.depth)
    intr=get_intrinsic.as_video_stream_profile().get_intrinsics()

    for x in range(20):
        # frames = pipeline.wait_for_frames() # 阻塞
        frames = pipeline.poll_for_frames()  # 非阻塞模式
        if not frames:
            continue  # 如果没有捕获到帧，直接跳过当前循环

        logger.info(f"成功捕获第{x}帧")

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        ir_frame = frames.get_infrared_frame()
        
        # 转换帧为 numpy 数组
        logger.info("转换帧为 numpy 数组")
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())

        # 创建保存路径
        os.makedirs(os.path.join(path, "depth"), exist_ok=True)
        os.makedirs(os.path.join(path, "RGB"), exist_ok=True)
        os.makedirs(os.path.join(path, "IR"), exist_ok=True)

        # 保存数据
        cv2.imwrite(os.path.join(path, "depth", f"{imgname}.png"), depth_image)
        cv2.imwrite(os.path.join(path, "RGB", f"{imgname}.png"), color_image)
        cv2.imwrite(os.path.join(path, "IR", f"{imgname}.png"), ir_image)
        
        logger.info(f"位置 {stallId} 猪 ID: {pigID} 数据保存成功在{path}")
        break
            
def setupCamera():
    cameraPipeline = rs.pipeline()
    cameraConfig = rs.config()
    cameraConfig.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cameraConfig.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cameraConfig.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

    return cameraPipeline, cameraConfig

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
            

def back_to_dock(pins, log_queue):
    logger = setup_logger("back_to_dock", log_queue)
    GPIO.output(pins["DIR"], 1)
    logger.info("Start to go back to the dock.")
    while True:
        resetSensor = GPIO.input(pins["resetPin"])

        GPIO.output(pins["STEP"], True)
        sleep(0.000001)
        GPIO.output(pins["STEP"], False)

        if resetSensor == 0:
            GPIO.output(pins["DIR"], 0)
            logger.info("Back in the dock now.")
            break

def setup_gpio(pins, log_queue):
    """Set up GPIO pins based on the configuration."""
    logger = setup_logger("cameraStart", log_queue)
    logger.info("初始化 GPIO 引脚")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(pins["DIR"], GPIO.OUT)
    GPIO.setup(pins["STEP"], GPIO.OUT)
    GPIO.setup(pins["ENA"], GPIO.OUT)
    GPIO.setup(pins["resetPin"], GPIO.IN)
    GPIO.setup(pins["endPin"], GPIO.IN)
    GPIO.setup(pins["magnetPin"], GPIO.IN)

    

def load_config(config_path):
    """Load configuration from the specified JSON file."""
    with open(config_path, 'r') as file:
        return json.load(file)

def camera_start(cameraConfig, cameraPipeline, log_queue):
    logger = setup_logger("cameraStart", log_queue)
    max_retries = 3  # 设置最大重试次数
    retry_delay = 30  # 每次重试的间隔时间（秒）

    for attempt in range(max_retries):
        try:
            logger.info(f"尝试启动相机pipeline (第 {attempt + 1} 次)")
            profile = cameraPipeline.start(cameraConfig)
            logger.info("相机启动成功")
            return profile
            # break  # 启动成功后跳出循环
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

def test_stop(pins, pigNumber, log_queue, cameraPipeline, profile, stallNumber):
    """
    Test each stop point whether works
    """
    # 创建线程池，最大线程数根据实际情况调整
    max_workers =  4  # 可根据树莓派的性能调整
    executor = ThreadPoolExecutor(max_workers=max_workers)
    logger = setup_logger("cameraStart", log_queue)
    stepCount = 0
    stall_id = 0
    preReset = False
    
    # 防抖时间
    debounce_time = 10  # 50ms，具体值可根据磁场特性调整
    maxStepCount = 200000  # 设置单个 Stall 位置的最大步数限制
    waitTime = 0.000001 / 1
    
    cycle = 0
    start = 0
    
    # 电机运行参数
    run_time = 3600  # 连续运行时间（秒）
    cool_down_time = 60  # 冷却时间（秒）
    
        
    # 扫描一轮 20 个位置
    while True:
        logger.info(f"开始第 {cycle + 1} 轮成像采集")
        preReset = False
        stall_id = 0  # 重置位置计数
        
        endReached = False
        
        for stall_id in range(stallNumber):  # 明确遍历每个 stall_id
            logger.info(f"开始移动到 Stall_{stall_id}")
            stepCount = 0 
        
            while True:
                # 启动电机
                GPIO.output(pins["ENA"], GPIO.LOW)
                GPIO.output(pins["DIR"], 0)
                
                # 传感器检测
                resetSensor = GPIO.input(pins["resetPin"])
                stopSensor = GPIO.input(pins["magnetPin"])
                endSensor = GPIO.input(pins["endPin"])
                
                # 成像周期开始
                if resetSensor == 0 and not preReset:
                    start = datetime.datetime.now()
                    logger.info(f"{start}, 开始新成像周期：{cycle + 1}")
                    stepCount = 0
                    preReset = True
                    logger.info(f"开始前往：Stall_{stall_id}")
                
                # 电机步进信号
                GPIO.output(pins["STEP"], True)
                sleep(waitTime)
                GPIO.output(pins["STEP"], False)
                stepCount += 1
                
                # 检查 Stall 位置
                if stopSensor == 0 and preReset:
                    if stepCount > 5000:
                        logger.info(f"抵达：Stall_{stall_id}")
                        logger.info(f"处理在Stall_{stall_id}的猪，ID: {pigNumber[stall_id]} ")
                        if executor._work_queue.qsize() < max_workers:
                            executor.submit(streamSensorLite, pigNumber[stall_id], cameraPipeline, profile, stall_id, log_queue)
                        else:
                            logger.warning("线程池已满，等待现有任务完成")

                        stepCount = 0
                        logger.info(f"开始前往：Stall_{stall_id}")
                        break
                
                # 终止位置
                if endSensor == 0:
                    sleep(5)
                    logger.info(f"抵达终止点，已完成本轮数据采集")
                    preReset = False
                    stall_id = 0
                    endReached = True
                    back_to_dock(pins, logger.info)
                    end = datetime.datetime.now()
                    logger.info(f"上一轮数据采集总耗时{end - start}")
                    break
                    
                #检查步数异常
                if stepCount >= maxStepCount:
                    logger.error(f"🚨 异常：超过 {maxStepCount} 步未能抵达下一个位置（Stall_{stall_id}），可能存在故障，直接返回dock")
                    back_to_dock(pins, logger.info)
                    stepCount = 0
                    send_alert_email(body = f"超过 {maxStepCount} 步未能抵达下一个位置（Stall_{stall_id}），可能存在故障，直接返回dock")
                    GPIO.output(pins["ENA"], GPIO.HIGH)  # 禁用电机
                    logger.info("电机已禁用，停止数据采集")
                    
                    # 重置状态并继续下一轮
                    preReset = False

                    time.sleep(30)  # 可以增加冷却时间，防止连续错误
                    logger.info("准备开始下一轮")
                    break  # 停止当前采集循环
            
            if endReached:
                break
            
        logger.info("完成一轮采集，回到起点")
        back_to_dock(pins, logger.info)
        end = datetime.datetime.now()
        logger.info(f"本轮数据采集耗时：{end - start}")
        cycle += 1


def reset(pins, log_queue):
    """
    wherever the system is, go back to the dock
    """
    logger = setup_logger("reset", log_queue)
    logger.info("重置系统位置")
    GPIO.output(pins["DIR"], 0)
    waitTime = 0.00001 / 1
    preStop = 0
    stepCount = 0
    GPIO.output(pins["ENA"], GPIO.LOW)
    maxStepCount = 200000  # Set step limit for error detection
    while True:
        stopSensor = GPIO.input(pins["magnetPin"])
        endSensor = GPIO.input(pins["endPin"])
        
        # Motor step signal
        GPIO.output(pins["STEP"], True)
        sleep(waitTime)  # Adjust delay as per motor specs
        GPIO.output(pins["STEP"], False)
        
        stepCount += 1
        if stopSensor == 0:
            if stepCount > 10000:
                logger.info(f"Get to the first stall by {stepCount}")
                stepCount = 0
                preStop = 1
        
        if preStop or endSensor == 0:
            back_to_dock(pins, log_queue)
            GPIO.output(pins["ENA"], GPIO.HIGH)
            break
        
        # Check for step count limit
        # if stepCount >= maxStepCount:
        #     logger.error("设备出现异常，超过150000步未能复位")
        #     stepCount = 0
        #     send_alert_email()
        #     GPIO.output(pins["ENA"], GPIO.HIGH)
        #     break

def send_alert_email(body = "设备在复位过程中超过200000步，可能存在故障，请及时检查。"):
    # Email configuration
    mail_host = 'smtp.gmail.com'
    mail_user = 'hxh0326@gmail.com'
    mail_pass = 'pnkvseixcwzvlqlu'
    mail_to = 'hxh0326@gmail.com'
    subject = "设备异常通知"
    body = "设备在复位过程中超过200000步，可能存在故障，请及时检查。"

    try:
        # Set up the email message
        message = MIMEMultipart()
        message['From'] = mail_user
        message['To'] = mail_to
        message['Subject'] = Header(subject, 'utf-8')
        message.attach(MIMEText(body, 'plain', 'utf-8'))

        # Connect to the server and send the email
        server = smtplib.SMTP(mail_host, 587)
        server.starttls()
        server.login(mail_user, mail_pass)
        server.sendmail(mail_user, mail_to, message.as_string())
        server.quit()

        print("📧 邮件发送成功")
    except Exception as e:
        print(f"❌ 邮件发送失败: {e}")

def main(log_queue, pins, pigNumber, stallNumber):
    
    """主程序逻辑"""
    logger = logging.getLogger("main")
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.setLevel(logging.DEBUG)
    
    logger.info("程序启动，等待 2 秒...")
    sleep(2)
    
    logger.info("初始化相机")
    cameraPipeline, cameraConfig = setupCamera()
        
    stallNumber = farm_config["stallNumber"]
    
    setup_gpio(pins, log_queue)
    profile = camera_start(cameraConfig, cameraPipeline, log_queue)
    reset(pins, log_queue)
    
    test_stop(pins, pigNumber, log_queue, cameraPipeline, profile, stallNumber)
    
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SwineRobot")
    parser.add_argument("--config", type=str, default="/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json",
                        help="Path to the JSON configuration file.")
    
    args = parser.parse_args()
    
    log_queue = multiprocessing.Queue()
    log_process = multiprocessing.Process(target=logger_process, args=(log_queue,))
    log_process.start()
    
    farm_config = load_config(args.config)
    pins = farm_config["pins"]
    pigNumber = farm_config["pigNumber"]
    stallNumber = farm_config["stallNumber"]
    
    
    main(log_queue, pins, pigNumber, stallNumber)

    log_queue.put(None)  # 结束信号
    log_process.join()