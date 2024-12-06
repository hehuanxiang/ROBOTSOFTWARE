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

class saveDataThread(threading.Thread):
    def __init__(self, frameset, i, path, PIG_ID, time_stamp, intr, log_queue, stallId):
        threading.Thread.__init__(self)
        self.threadID = threading.get_ident()
        self.frameset=frameset
        self.path=path
        self.i=i
        self.PIG_ID=PIG_ID
        self.time_stamp=time_stamp
        self.intr=intr
        self.logger = setup_logger(f"saveDataThread-{self.threadID}", log_queue)
        self.data_saved = False  # 新增属性，初始为 False，判断是否成功保存数据
        self.stallId = stallId

    def run(self):
        try: 
            #set clip depth
            
            intr=self.intr
            depth_threshold=rs.threshold_filter(min_dist=0.1, max_dist=2.5)
            colorizer=rs.colorizer()
            path=self.path
            frames=self.frameset
            t=self.time_stamp
            PIG_ID=self.PIG_ID
            j=1
            imgname = str(PIG_ID) + "_"+ str(t.year)+"_"+ str(t.month)+"_"+ str(t.day)+ "_"+str(t.hour)+"_"+str(t.minute) + "_"+str(t.second)+"_"
            self.logger.info(f"开始保存stall：Stall_{self.stallId}，猪 ID: {PIG_ID} 的数据到路径: {path}")

            for frame in frames:
                color_frame=frame.get_color_frame()
                depth_frame=frame.get_depth_frame()
                ir_frame=frame.get_infrared_frame()
            #depth image
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                ir_image = np.asanyarray(ir_frame.get_data())

                colorized=colorizer.process(ir_frame)
                index=(self.i)
                index=j
                name=imgname+str(j)
                XX=np.zeros((480,640))
                YY=np.zeros((480,640))
                ZZ=np.zeros((480,640))

                coverage = [0]*64
                for y in range(480):
                    for x in range(480):
                        dist = depth_frame.get_distance(x+80, y)
                        [X,Y,Z] = convert_depth_pixel_to_metric_coordinate(dist, x,y, intr)
                        XX[y,x] = X
                        YY[y,x] = Y
                        ZZ[y,x] = Z


                obj=np.stack((XX,YY,ZZ))
                if not os.path.exists(path+"DM"):
                    os.makedirs(path+"DM")

                if not os.path.exists(path+"depth"):
                    os.makedirs(path+"depth")
                if not os.path.exists(path+"RGB"):
                    os.makedirs(path+"RGB")
                if not os.path.exists(path+"IR"):
                    os.makedirs(path+"IR")

                #points.export_to_ply(path+"pcl/"+name+".ply", color_frame)
                matfile=path+"DM/"+ name + ".mat"
                scipy.io.savemat(matfile, mdict={'out': obj}, oned_as='row')
                imageio.imwrite(path+"depth/"+name+".png", depth_image)
                imageio.imwrite(path+"RGB/"+name+".png", color_image)
                imageio.imwrite(path+"IR/"+name+".png", ir_image)
                j=j+1
            self.logger.info(f"Stall_{self.stallId}，猪 ID {PIG_ID} 数据保存完成，线程 ID: {self.threadID}")
            self.data_saved = True
        except Exception as e:
            self.data_saved = False
            self.logger.error(f"Stall_{self.stallId}，保存猪 ID {self.PIG_ID} 数据时发生错误", exc_info=True)
            raise RuntimeError(f"数据未成功保存，Stall_{self.stallId}，猪_{PIG_ID}")

def streamSensor(pigID, cameraPipeline, profile, stallId, log_queue):
    logger = setup_logger("streamSensor", log_queue)
    
    pipeline = cameraPipeline
    profile = profile
    pig_ID=pigID

    # str.zfill(2)：
    # 将 pigID 转换为字符串，不足两位时在前面填充 0。
    # 示例：1.zfill(2) -> "01"。
    path = f"./Data/Data_Estrus_2024/ID_{str(stallId).zfill(2)}_{str(pigID)}/"
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
        interval=20

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
    except:   
        # pipeline.stop()
        logger.info("相机无法正常获取图像")
    try:
        thread1 = saveDataThread(frameset,int(x/interval),path, pig_ID, t,intr, log_queue, stallId)
        thread1.start()
        logger.info(f"启动保存线程: {thread1.threadID} 开始保存猪_{pig_ID}的数据")
    except:
        thread1.stop()
        logger.error("保存线程启动失败", exc_info=True)
    logger.info("图像捕获成功并正在保存数据")

def get_sensor():
    ctx = rs.context()
        #print(ctx)
    camera_id = []
    intrinsics = []
    configurations = []
    pipelines =[]
    for d in ctx.devices:
        print(d.get_info(rs.camera_info.serial_number))
        camera_id.append(d.get_info(rs.camera_info.serial_number))

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(d.get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
        config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
        config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

        pipelines.append(pipeline)
        profile = pipeline.start(config)
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intrinsics.append(get_intrinsic.as_video_stream_profile().get_intrinsics())
        pipeline.stop()
        configurations.append(config)
    return camera_id,intrinsics,configurations,pipelines

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

def main(log_queue):
    """主程序逻辑"""
    logger = logging.getLogger("main")
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.setLevel(logging.DEBUG)

    try:
        logger.info("加载 farm_config.json 配置文件")
        with open('/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json', 'r') as file:
            farm_config = json.load(file)

        pigNumber = farm_config["pigNumber"]
        pins = farm_config["pins"]

        logger.info("初始化 GPIO 引脚")
        pins = farm_config["pins"]

        endPin = pins["endPin"]
        resetPin = pins["resetPin"]
        magnetPin = pins["magnetPin"]
        DIR = pins["DIR"]
        ENA = pins["ENA"]
        STEP =pins["STEP"]
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(DIR,GPIO.OUT)
        GPIO.setup(ENA,GPIO.OUT)
        GPIO.setup(STEP,GPIO.OUT)
        GPIO.output(ENA,GPIO.HIGH)

        logger.info("初始化步进电机")
        testStepper = Stepper([pins["STEP"], pins["DIR"], pins["ENA"], pins["endPin"], pins["resetPin"], pins["magnetPin"]], log_queue)

        logger.info("程序启动，等待 2 秒...")
        sleep(2)

        action = testStepper.step(40000 * 10, "forward", 0.1, docking=True)
        logger.info(f"首次步进操作结果: {action}")
        handle_stop(action, testStepper)
        logger.info("返回对接位置")

        cameraPipeline, cameraConfig = setupCamera()
        
        stallNumber = farm_config["stallNumber"]
        
        cycle = 0
        
        # 定义每个成像周期的计数器和休息逻辑
        start_cycle_time = time.time()  # 记录循环开始的时间
        rest_interval = 5 * 60 * 60  # 每隔5小时 (单位为秒)
        rest_duration = 10 * 60  # 休息5分钟 (单位为秒)
        while True:
            # t1 = datetime.datetime.now()
            # if t1.minute % 1 == 0:
            start_time = time.time()
            logger.info(f"{datetime.datetime.now()},开始新成像周期：{cycle + 1}")
            
            logger.info("启动相机pipeline")
            profile = cameraPipeline.start(cameraConfig)
            for i in range(stallNumber):
                try:
                    if i == 0:
                        logger.info(f"快速接近：Stall_{i}")
                        action = testStepper.step(55000, "forward", 1, docking=False)
                        logger.info(f"即将抵达：Stall_{i}，减速接近")
                        action = testStepper.step(35000, "forward", 0.05, docking=False)
                    else:
                        logger.info(f"快速接近：Stall_{i}")
                        action = testStepper.step(25000, "forward", 1, docking=False)
                        logger.info(f"即将抵达：Stall_{i}，减速接近")
                        action = testStepper.step(30000, "forward", 0.05, docking=False)

                    handle_stop(action, testStepper)
                    logger.info(f"抵达：Stall_{i}")

                    if pigNumber[i] != 9998:
                        logger.info(f"处理在Stall_{i}的猪，ID: {pigNumber[i]} ")
                        streamSensor(pigNumber[i], cameraPipeline, profile, i, log_queue)

                except Exception as e:
                    logger.error(f"当前：Stall_{i}猪猪成像过程中发生错误", exc_info=True)
                    
            logger.info(f"开始返回起始点，时间: {datetime.datetime.now()}")
            action = testStepper.step(150000 * 24, "back", 1000, docking=True)
            logger.info(f"返回到起始点，时间: {datetime.datetime.now()}")
            
            end_time = time.time()
            total_time = end_time - start_time
            logger.info(f"第{cycle + 1}个成像周期完成于：{datetime.datetime.now()}，用时 {total_time:.2f} 秒")
            cameraPipeline.stop()
            logger.info("关闭相机pipeline")
            
            memory = psutil.virtual_memory()
            logger.info(f"Memory used: {memory.percent}%")
            
            #检查是否需要休息
            # elapsed_time_since_start = time.time() - start_cycle_time
            # if elapsed_time_since_start >= rest_interval:
            #     logger.info(f"已运行 {elapsed_time_since_start / 3600:.2f} 小时，休息 10 分钟。")
            #     time.sleep(rest_duration)  # 休息 5 分钟
            #     start_cycle_time = time.time()  # 重置计时器

            handle_stop(action, testStepper)

            cycle = cycle + 1
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