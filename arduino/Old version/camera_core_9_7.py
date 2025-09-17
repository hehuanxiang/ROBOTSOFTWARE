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
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)

    try:
        time.sleep(2)  # 等待摄像头初始化
        profile = pipeline.start(config)

        # ---- 改动点 1：用短超时轮询首帧（总预算约 3s）----
        frames = None
        deadline = time.time() + 3.0
        while time.time() < deadline:
            try:
                frames = pipeline.wait_for_frames(200)  # 200ms/次
                if frames:
                    depth = frames.get_depth_frame()
                    color = frames.get_color_frame()
                    ir    = frames.get_infrared_frame()
                    if depth and color and ir:
                        break  # 三路齐全即可
            except Exception:
                pass
        if not frames or not (depth and color and ir):
            raise RuntimeError("First frames not ready (depth/color/ir) within 3.0s")

        # ---- 改动点 2：预热几帧（稳定曝光/对齐）----
        for _ in range(5):
            pipeline.wait_for_frames(200)

        # ---- 改动点 3：确认就绪后再打印“启动成功”与探针日志 ----
        logger.info("✅ RealSense camera started successfully.")
        logger.info("🧪 Depth stream: OK")
        logger.info("🧪 Color stream: OK")
        logger.info("🧪 Infrared stream: OK")

        return pipeline, profile

    except Exception as e:
        logger.error(f"❌ Failed to start RealSense camera: {e}")
        # 尽量干净停掉，避免占用句柄影响下一轮
        try:
            pipeline.stop()
        except Exception:
            pass
        # 最小改动：不 exit，交由上层判断是否跳过本周期
        return None, None

def capture_images(pipeline, profile, pig_id, stall_id, logger):
    timestamp = datetime.datetime.now()
    path = f"/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2025_08/ID_{str(stall_id).zfill(2)}_{str(pig_id)}"
    os.makedirs(os.path.join(path, "depth"), exist_ok=True)
    os.makedirs(os.path.join(path, "RGB"), exist_ok=True)
    os.makedirs(os.path.join(path, "IR"), exist_ok=True)
    os.makedirs(os.path.join(path, "RGB_noAligned"), exist_ok=True)
    os.makedirs(os.path.join(path, "IR_noAligned"), exist_ok=True)
    imgname = f"{pig_id}_{timestamp.year}_{timestamp.month}_{timestamp.day}_{timestamp.hour}_{timestamp.minute}_{timestamp.second}"

    logger.info(f"Preparing to capture image: {imgname} at stall {stall_id}")

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
        raw_color = frames.get_color_frame()
        ir = aligned_frames.get_infrared_frame()  
        raw_ir = frames.get_infrared_frame()  
        
        if not color or not depth or not ir:
            continue

        # 保存图像
        cv2.imwrite(os.path.join(path, "depth", f"{imgname}.png"), np.asanyarray(depth.get_data()))
        cv2.imwrite(os.path.join(path, "RGB", f"{imgname}.png"), np.asanyarray(color.get_data()))
        cv2.imwrite(os.path.join(path, "IR", f"{imgname}.png"), np.asanyarray(ir.get_data()))
        cv2.imwrite(os.path.join(path, "RGB_noAligned", f"{imgname}.png"), np.asanyarray(raw_color.get_data()))
        cv2.imwrite(os.path.join(path, "IR_noAligned", f"{imgname}.png"), np.asanyarray(raw_ir.get_data()))

        logger.debug(f"📸 Captured images for pig {pig_id} at stall {stall_id} saved as {imgname}")

        break
    else:
        logger.error("❌ Failed to capture images for pig {pig_id} at stall {stall_id}")


def run_camera_system(queue, logger, stop_event):
    pipeline = None
    profile = None
    started_at = None  # 记录本轮开始时间，便于调试
    IDLE_AUTO_STOP_SEC = 180  # 可选：若长时间没有任务，自动停相机以防闲置

    logger.info("Camera system started. Waiting for capture tasks...")

    def ensure_started():
        nonlocal pipeline, profile, started_at
        if pipeline is None:
            pipeline, profile = setup_camera(logger)
            started_at = time.time()

    def ensure_stopped():
        nonlocal pipeline, profile, started_at
        if pipeline is not None:
            try:
                pipeline.stop()
                logger.info("🔌 Camera pipeline stopped cleanly.")
            except Exception as e:
                logger.warning(f"⚠️ Failed to stop pipeline cleanly: {e}")
            pipeline = None
            profile = None
            started_at = None

    try:
        while not stop_event.is_set():
            # 可选：空闲自动停机，避免“热着不取帧”
            if pipeline is not None and (time.time() - started_at > IDLE_AUTO_STOP_SEC):
                logger.info("⏸️ idle timeout, auto stop camera to avoid long idle")
                ensure_stopped()

            try:
                task = queue.get(timeout=1)
            except Exception:
                continue

            t = task.get("type", "capture")  # 兼容旧消息（没有 type 字段时按拍照处理）

            if t == "cycle_start":
                logger.debug("📥 received: cycle_start → start camera")
                ensure_started()

            elif t == "capture":
                stall = task.get("stall")
                pig_id = task.get("pig_id")
                logger.debug(f"📥 received: capture stall={stall}, pig_id={pig_id}")
                # 兜底：如果 motor 先发了 capture 没发 start，也能自动开启
                ensure_started()
                try:
                    capture_images(pipeline, profile, pig_id, stall, logger)
                except Exception as e:
                    logger.exception(f"❌ capture failed: {e}")
                    # 保险：出现异常重启一次 pipeline
                    ensure_stopped()
                    time.sleep(0.2)
                    ensure_started()

            elif t == "cycle_end":
                logger.debug("📥 received: cycle_end → stop camera")
                ensure_stopped()

            else:
                logger.warning(f"⚠️ unknown task type: {t}")

    finally:
        # 进程退出兜底
        ensure_stopped()
        logger.info("Camera process exiting.")


