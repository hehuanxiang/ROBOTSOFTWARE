import pyrealsense2.pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)
frames = pipeline.wait_for_frames()

# 获取深度和彩色的 stream profile
depth_profile = frames.get_depth_frame().profile
color_profile = frames.get_color_frame().profile

# 获取外参：从 depth 坐标系 → color 坐标系
extrinsics = depth_profile.get_extrinsics_to(color_profile)

print("Extrinsics from DEPTH to COLOR:")
print("Rotation (3x3):")
print(np.array(extrinsics.rotation).reshape(3, 3))
print("Translation (tvec):")
print(np.array(extrinsics.translation))  # 单位为米

pipeline.stop()
