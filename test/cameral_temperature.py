import pyrealsense2.pyrealsense2 as rs

def get_camera_temperature():
    # 创建上下文
    context = rs.context()
    
    # 检查连接的设备
    devices = context.query_devices()
    if len(devices) == 0:
        print("No RealSense devices found!")
        return
    
    for device in devices:
        print(f"Device: {device.get_info(rs.camera_info.name)}")
        # 获取传感器信息
        sensors = device.query_sensors()
        for sensor in sensors:
            if sensor.supports(rs.option.ambient_temperature):
                temp = sensor.get_option(rs.option.ambient_temperature)
                print(f"Ambient Temperature: {temp} °C")
            else:
                print(f"Sensor {sensor.get_info(rs.camera_info.name)} does not support temperature reading.")

get_camera_temperature()
