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
import threading

# 设置统一的日志文件名
# LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log/Sowbot_record_{}.log".format(datetime.date.today())

# def setup_camera_logger():
#     logger = logging.getLogger("Sowbot")
#     if not logger.handlers:
#         handler = logging.FileHandler(LOG_FILE)
#         formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
#         handler.setFormatter(formatter)
#         logger.addHandler(handler)
#         logger.setLevel(logging.DEBUG)
#     return logger

def setup_camera(logger):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)
    
    try:
        
        profile = pipeline.start(config)
        time.sleep(10)  # 等待摄像头初始化
        logger.info("✅ RealSense camera started successfully.")
        return pipeline, profile
    except Exception as e:
        error_msg = f"❌ Failed to start RealSense camera: {str(e)}"
        logger.error(error_msg)
        exit(1) 

# def capture_images(pipeline, profile, pig_id, stall_id, logger):
#     timestamp = datetime.datetime.now()
#     path = f"/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_08/ID_{str(stall_id).zfill(2)}_{str(pig_id)}"
#     os.makedirs(os.path.join(path, "depth"), exist_ok=True)
#     os.makedirs(os.path.join(path, "RGB"), exist_ok=True)
#     os.makedirs(os.path.join(path, "IR"), exist_ok=True)
#     os.makedirs(os.path.join(path, "RGB_noAligned"), exist_ok=True)
#     os.makedirs(os.path.join(path, "IR_noAligned"), exist_ok=True)
#     imgname = f"{pig_id}_{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}"

#     logger.debug(f"Preparing to capture image: {imgname} at stall {stall_id}")

#     align_to = rs.stream.depth
#     align = rs.align(align_to)
    
#     for _ in range(20):
#         frames = pipeline.poll_for_frames()
#         if not frames:
#             continue

#         # 对齐 RGB 到 Depth
#         aligned_frames = align.process(frames)
#         depth = aligned_frames.get_depth_frame()
#         color = aligned_frames.get_color_frame()
#         raw_color = frames.get_color_frame()
#         ir = aligned_frames.get_infrared_frame()  
#         raw_ir = frames.get_infrared_frame()  
        
#         if not color or not depth or not ir:
#             continue

#         # 保存图像
#         cv2.imwrite(os.path.join(path, "depth", f"{imgname}.png"), np.asanyarray(depth.get_data()))
#         cv2.imwrite(os.path.join(path, "RGB", f"{imgname}.png"), np.asanyarray(color.get_data()))
#         cv2.imwrite(os.path.join(path, "IR", f"{imgname}.png"), np.asanyarray(ir.get_data()))
#         cv2.imwrite(os.path.join(path, "RGB_noAligned", f"{imgname}.png"), np.asanyarray(raw_color.get_data()))
#         cv2.imwrite(os.path.join(path, "IR_noAligned", f"{imgname}.png"), np.asanyarray(raw_ir.get_data()))

#         logger.debug(f"📸 Captured images for pig {pig_id} at stall {stall_id} saved as {imgname}")

#         break
#     else:
#         logger.error("❌ Failed to capture images for pig {pig_id} at stall {stall_id}")


def capture_images(pipeline, profile, pig_id, stall_id, logger):
    timestamp = datetime.datetime.now()
    path = f"/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_08/ID_{str(stall_id).zfill(2)}_{str(pig_id)}"
    # 创建目录（保留原有结构）
    for subdir in ["depth", "RGB", "IR", "RGB_noAligned", "IR_noAligned", "depth_noAligned"]:
        os.makedirs(os.path.join(path, subdir), exist_ok=True)
    
    imgname = f"{pig_id}_{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}"
    logger.debug(f"⚡ 快速捕获: {imgname} at stall {stall_id}")

    # align_to = rs.stream.depth
    # align = rs.align(align_to)
    
    # 1. 快速丢弃旧帧（仅2帧）
    # for _ in range(2):
    #     try:
    #         pipeline.wait_for_frames(timeout_ms=30)
    #     except:
    #         pass
    
    # 2. 单次获取有效帧
    try:
        # 确保获取新帧
        pipeline.wait_for_frames(timeout_ms=50)  # 丢弃一帧
        
        # 获取新帧
        frames = pipeline.wait_for_frames(timeout_ms=100)
        
        if not frames:
            # 快速重试一次
            frames = pipeline.wait_for_frames(timeout_ms=50)
            if not frames:
                raise RuntimeError("获取帧超时")
        
        # 3. 对齐处理
        # aligned_frames = align.process(frames)
        
        # 4. 并行获取所有帧
        # depth = aligned_frames.get_depth_frame()
        # color = aligned_frames.get_color_frame()
        # ir = aligned_frames.get_infrared_frame()
        raw_depth = frames.get_depth_frame()
        raw_color = frames.get_color_frame()
        raw_ir = frames.get_infrared_frame()
        
        # 5. 快速验证
        if not all([raw_depth, raw_color, raw_ir]):
            raise RuntimeError("帧数据不完整")
        
        # 6. 并行保存（使用线程加速）
        def save_image(img, path, name):
            cv2.imwrite(os.path.join(path, name), img)
        
        threads = []
        # threads.append(threading.Thread(target=save_image, args=(np.asanyarray(depth.get_data()), path+"/depth", f"{imgname}.png")))
        # threads.append(threading.Thread(target=save_image, args=(np.asanyarray(color.get_data()), path+"/RGB", f"{imgname}.png")))
        # threads.append(threading.Thread(target=save_image, args=(np.asanyarray(ir.get_data()), path+"/IR", f"{imgname}.png")))
        threads.append(threading.Thread(target=save_image, args=(np.asanyarray(raw_depth.get_data()), path+"/depth_noAligned", f"{imgname}.png")))
        threads.append(threading.Thread(target=save_image, args=(np.asanyarray(raw_color.get_data()), path+"/RGB_noAligned", f"{imgname}.png")))
        threads.append(threading.Thread(target=save_image, args=(np.asanyarray(raw_ir.get_data()), path+"/IR_noAligned", f"{imgname}.png")))
        
        for t in threads:
            t.start()
        
        # 等待所有保存完成（最大等待500ms）
        start_time = time.time()
        for t in threads:
            t.join(timeout=0.1)
            if time.time() - start_time > 0.5:
                logger.warning("⚠️ 部分图像保存超时")
                break
        
        logger.debug(f"📸 捕获完成: {imgname}")
        return  # 立即退出！
        
    except Exception as e:
        logger.error(f"❌ 捕获失败: {e}")
        # 可选：记录失败但继续运行

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
