# camera_core.py
# 相机系统逻辑，负责从队列读取任务并采集图像

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import os
import datetime
import logging
from time import sleep
import time

# 设置统一的日志文件名
LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log/Sowbot_record_{}.log".format(datetime.date.today())

def setup_camera_logger():
    logger = logging.getLogger("Sowbot")
    if not logger.handlers:
        handler = logging.FileHandler(LOG_FILE)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)
    return logger

def setup_camera(logger):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)
    
    try:
        time.sleep(2)  # 等待摄像头初始化
        profile = pipeline.start(config)
        logger.info("✅ RealSense camera started successfully.")
        return pipeline, profile
    except Exception as e:
        error_msg = f"❌ Failed to start RealSense camera: {str(e)}"
        logger.error(error_msg)
        exit(1) 

def capture_images(pipeline, profile, pig_id, stall_id, logger):
    timestamp = datetime.datetime.now()
    path = f"/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_07/ID_{str(stall_id).zfill(2)}_{str(pig_id)}"
    os.makedirs(os.path.join(path, "depth"), exist_ok=True)
    os.makedirs(os.path.join(path, "RGB"), exist_ok=True)
    os.makedirs(os.path.join(path, "IR"), exist_ok=True)
    imgname = f"{pig_id}_{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}"

    logger.debug(f"Preparing to capture image: {imgname} at stall {stall_id}")

    align_to = rs.stream.depth
    align = rs.align(align_to)
    
    for _ in range(20):
        frames = pipeline.poll_for_frames()
        if not frames:
            continue

        # 对齐 RGB 到 Depth
        aligned_frames = align.process(frames)
        depth = aligned_frames.get_depth_frame()
        color = aligned_frames.get_color_frame()
        ir = aligned_frames.get_infrared_frame()  
        
        if not color or not depth or not ir:
            continue

        # 保存图像
        cv2.imwrite(os.path.join(path, "depth", f"{imgname}.png"), np.asanyarray(depth.get_data()))
        cv2.imwrite(os.path.join(path, "RGB", f"{imgname}.png"), np.asanyarray(color.get_data()))
        cv2.imwrite(os.path.join(path, "IR", f"{imgname}.png"), np.asanyarray(ir.get_data()))

        logger.debug(f"📸 Captured images for pig {pig_id} at stall {stall_id} saved as {imgname}")

        break

def run_camera_system(queue, logger, stop_event):
    pipeline, profile = setup_camera(logger)
    logger.info("Camera system started. Waiting for capture tasks...")

    while not stop_event.is_set():
        try:
            task = queue.get(timeout=1)
            stall = task.get("stall")
            pig_id = task.get("pig_id")
            logger.debug(f"Received task: stall={stall}, pig_id={pig_id}")
            capture_images(pipeline, profile, pig_id, stall, logger)
        except Exception:
            # pipeline.stop()
            pass

