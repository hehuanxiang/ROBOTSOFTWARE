import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import os

# 创建test文件夹用于存储图片
output_folder = "data_test"
os.makedirs(output_folder, exist_ok=True)

# 创建管道
pipeline = rs.pipeline()
config = rs.config()

# 启用深度、彩色和红外流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 深度流
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 彩色流
config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)  # 红外流

# 启动流
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_sensor.set_option(rs.option.min_distance,0)
depth_sensor.set_option(rs.option.enable_max_usable_range,0)
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: {}\n" .format(depth_scale))
get_intrinsic = profile.get_stream(rs.stream.depth)
intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
print("Camera intrinsics is: {}\n".format(intr))

frame_count = 0  # 用于命名保存的图片

try:
    while frame_count < 10:
        # 等待帧并获取数据
        frames = pipeline.wait_for_frames()

        # 获取深度、彩色和红外帧
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        infrared_frame = frames.get_infrared_frame()

        if not depth_frame or not color_frame or not infrared_frame:
            continue

        # 将数据转换为numpy数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        infrared_image = np.asanyarray(infrared_frame.get_data())

        # 显示各个数据流
        cv2.imshow('Depth Stream', depth_image)
        cv2.imshow('Color Stream', color_image)
        cv2.imshow('Infrared Stream', infrared_image)

        # 保存每一帧到test文件夹
        cv2.imwrite(f"{output_folder}/depth_{frame_count}.png", depth_image)
        cv2.imwrite(f"{output_folder}/color_{frame_count}.png", color_image)
        cv2.imwrite(f"{output_folder}/infrared_{frame_count}.png", infrared_image)

        frame_count += 1  # 更新帧计数

        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # 停止流并清理资源
    pipeline.stop()
    cv2.destroyAllWindows()
