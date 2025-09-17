import pyrealsense2.pyrealsense2 as rs

pipe = rs.pipeline()
cfg = rs.config()

# 在线摄像头（注释掉下一行）或离线 .bag（保留下一行）
# cfg.enable_device_from_file("your_recording.bag", repeat_playback=False)

cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
profile = pipe.start(cfg)

depth_sensor = profile.get_device().first_depth_sensor()
scale = depth_sensor.get_depth_scale()   # ← 这里就是 depth scale（以米为单位）
print("Depth scale =", scale)            # L515 应该打印 0.00025
pipe.stop()