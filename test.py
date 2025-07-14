import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()

config.enable_device('f1370816')  # 你设备的序列号
config.enable_stream(rs.stream.depth, 320, 240, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

try:
    profile = pipeline.start(config)
    print("✅ 启动成功")
    pipeline.stop()
except RuntimeError as e:
    print("❌ 启动失败：", e)
