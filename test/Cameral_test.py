import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
import os
import time, datetime
import threading
from time import sleep
import scipy.io
import imageio
import sys
sys.path.append('..')
from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate



# 创建test文件夹用于存储图片
output_folder = "data_test"
os.makedirs(output_folder, exist_ok=True)
def cameral_test():
    # 创建管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 启用深度、彩色和红外流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # 深度流
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 彩色流
    config.enable_stream(rs.stream.infrared, 0, 640, 480, rs.format.y8, 30)  # 红外流
    # config.enable_stream(rs.stream.infrared,0,848,480,rs.format.y8,30)

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
            # frames = pipeline.wait_for_frames()
            frames = pipeline.poll_for_frames()  # 非阻塞

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
            # cv2.imshow('Depth Stream', depth_image)
            # cv2.imshow('Color Stream', color_image)
            # cv2.imshow('Infrared Stream', infrared_image)

            # 保存每一帧到test文件夹
            cv2.imwrite(f"{output_folder}/depth_{frame_count}.png", depth_image)
            cv2.imwrite(f"{output_folder}/color_{frame_count}.png", color_image)
            cv2.imwrite(f"{output_folder}/infrared_{frame_count}.png", infrared_image)

            frame_count += 1  # 更新帧计数

            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        print("拍照结束")

    finally:
        # 停止流并清理资源
        pipeline.stop()
        cv2.destroyAllWindows()

def setupCamera():
    cameraPipeline = rs.pipeline()
    cameraConfig = rs.config()
    cameraConfig.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)
    cameraConfig.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cameraConfig.enable_stream(rs.stream.infrared,0,1024,768,rs.format.y8,30)

    return cameraPipeline, cameraConfig

def streamSensorRaw(pigID, cameraPipeline, cameraConfig, stallId, frame_count=50):
    """
    采集并直接保存原始帧数据（未解析）。
    :param pigID: 猪只 ID
    :param cameraPipeline: RealSense 相机管道
    :param cameraConfig: RealSense 配置
    :param stallId: 栏位 ID
    :param frame_count: 需要采集的总帧数
    """
    pipeline = cameraPipeline
    config = cameraConfig
    pig_ID = pigID

    # 创建保存路径
    path = f"./Data/Data_Estrus_2024/ID_{str(stallId).zfill(2)}_{str(pigID)}/Raw"
    os.makedirs(path, exist_ok=True)

    try:
        # 启动相机管道
        profile = pipeline.start(config)
        print(f"Pipeline started. Capturing {frame_count} frames...")

        captured_frames = 0

        while captured_frames < frame_count:
            # 从管道获取帧
            frames = pipeline.wait_for_frames()

            # 遍历帧集合的子帧
            for frame in frames:
                # 提取帧类型和数据
                frame_type = frame.get_profile().stream_type()
                frame_data = frame.as_frame().get_data()

                # 根据帧类型选择保存路径
                if frame_type == rs.stream.depth:
                    frame_path = os.path.join(path, f"depth_{captured_frames}.raw")
                elif frame_type == rs.stream.color:
                    frame_path = os.path.join(path, f"color_{captured_frames}.raw")
                elif frame_type == rs.stream.infrared:
                    frame_path = os.path.join(path, f"ir_{captured_frames}.raw")
                else:
                    continue  # 跳过未知帧类型

                # 保存帧数据
                with open(frame_path, "wb") as f:
                    f.write(bytearray(frame_data))

            captured_frames += 1
            print(f"Captured and saved frame {captured_frames}/{frame_count}.")

    finally:
        # 停止相机管道
        pipeline.stop()
        print("Pipeline stopped. Raw frame capture complete.")

def streamSensor(pigID, pipeline, profile, stallId):

    pipeline
    pig_ID=pigID
    # print("ID:"+str(pig_ID))
    
    # path ="./Data/Data_Estrus_2024/" +"ID_" +str(stallId).zfill(2)+ "_" + str(pigID) + "/"#+imgname
    path = f"./Data/Data_Estrus_2024/ID_{str(stallId).zfill(2)}_{str(pigID)}/"
    os.makedirs(path, exist_ok=True)

    try:
        profile = profile
        colorizer=rs.colorizer()

        tt = time.monotonic()
        t = datetime.datetime.now()

        frameset=[]
        interval=20
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

            if not aligned_depth_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            key = cv2.waitKey(1)

            if x%interval==0: #60
                print(int(x/interval))
                frameset.append(aligned_frames)

    finally:   
        pipeline.stop()
    try:
        thread1=saveDataThread((x+interval)/interval, frameset,int(x/interval),path, pig_ID, t,intr)
        thread1.start()
        sleep(2)
    except:
        thread1.stop()
    print("captured succeed, trying to save")
    
class saveDataThread(threading.Thread):
    def __init__(self, threadID, frameset,i,path, PIG_ID,time_stamp,intr):
        threading.Thread.__init__(self)
        self.threadID=threadID
        self.frameset=frameset
        self.path=path
        self.i=i
        self.PIG_ID=PIG_ID
        self.time_stamp=time_stamp
        self.intr=intr

    def run(self):
        #set clip depth
        intr=self.intr
        depth_threshold=rs.threshold_filter(min_dist=0.1, max_dist=2.5)
        colorizer=rs.colorizer()
        path=self.path
        print(path)
        frames=self.frameset
        t=self.time_stamp
        PIG_ID=self.PIG_ID
        j=1
        imgname = str(PIG_ID) + "_"+ str(t.year)+"_"+ str(t.month)+"_"+ str(t.day)+ "_"+str(t.hour)+"_"+str(t.minute) + "_"+str(t.second)+"_"

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
        #print(depth_image.shape)
            imageio.imwrite(path+"RGB/"+name+".png", color_image)
        #print(color_image.shape)
            imageio.imwrite(path+"IR/"+name+".png", ir_image)
            j=j+1
        print("save  PIG ID_" + str(PIG_ID) + " data success "+str(self.threadID))

def record_bag(output_file, record_time=1):
    """
    录制 RealSense 数据到 .bag 文件。

    Args:
        output_file (str): 保存的 .bag 文件路径。
        record_time (int): 录制时长，单位秒。
    """
    # 配置管道
    pipeline = rs.pipeline()
    config = rs.config()

    # 配置流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 设置录制文件路径
    config.enable_record_to_file(output_file)

    # 启动管道
    pipeline.start(config)
    print(f"Recording started: {output_file}")

    try:
        start_time = time.time()
        last_frame_time = start_time
        frame_interval = 1 / 30  # 每帧间隔（秒）

        while time.time() - start_time < record_time:
            current_time = time.time()
            if current_time - last_frame_time >= frame_interval:
                frames = pipeline.wait_for_frames()
                last_frame_time = current_time
                print("*")

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print(f"Recording stopped. File saved: {output_file}")
        
class BagsaveDataThread(threading.Thread):
    def __init__(self, threadID, frameset, i, path, PIG_ID, time_stamp, intr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.frameset = frameset
        self.path = path
        self.i = i
        self.PIG_ID = PIG_ID
        self.time_stamp = time_stamp
        self.intr = intr

    def run(self):
        intr = self.intr
        depth_threshold = rs.threshold_filter(min_dist=0.1, max_dist=2.5)
        colorizer = rs.colorizer()
        path = self.path
        frames = self.frameset
        t = self.time_stamp
        PIG_ID = self.PIG_ID
        j = 1
        imgname = f"{PIG_ID}_{t.year}_{t.month}_{t.day}_{t.hour}_{t.minute}_{t.second}_"

        for frame in frames:
            color_frame = frame.get_color_frame()
            depth_frame = frame.get_depth_frame()
            ir_frame = frame.get_infrared_frame()

            # 获取帧数据
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_image = np.asanyarray(ir_frame.get_data())

            XX, YY, ZZ = np.zeros((480, 640)), np.zeros((480, 640)), np.zeros((480, 640))

            for y in range(480):
                for x in range(480):
                    dist = depth_frame.get_distance(x, y)
                    X, Y, Z = convert_depth_pixel_to_metric_coordinate(dist, x, y, intr)
                    XX[y, x] = X
                    YY[y, x] = Y
                    ZZ[y, x] = Z

            obj = np.stack((XX, YY, ZZ))

            # 创建目录
            for sub_dir in ["DM", "depth", "RGB", "IR"]:
                os.makedirs(os.path.join(path, sub_dir), exist_ok=True)

            # 保存数据
            matfile = os.path.join(path, "DM", f"{imgname}{j}.mat")
            scipy.io.savemat(matfile, mdict={"out": obj}, oned_as="row")
            imageio.imwrite(os.path.join(path, "depth", f"{imgname}{j}.png"), depth_image)
            imageio.imwrite(os.path.join(path, "RGB", f"{imgname}{j}.png"), color_image)
            imageio.imwrite(os.path.join(path, "IR", f"{imgname}{j}.png"), ir_image)
            j += 1

        print(f"save  PIG ID_{PIG_ID} data success {self.threadID}")

# 主程序
def process_bag_file(bag_file, output_dir, PIG_ID):
    """
    处理 .bag 文件，并使用多线程保存帧数据。

    Args:
        bag_file (str): .bag 文件路径。
        output_dir (str): 保存路径。
        PIG_ID (int): 猪的 ID。
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_file)

    pipeline.start(config)

    try:
        thread_id = 1
        while True:
            frames = []
            for _ in range(20):  # 每次处理 5 帧
                frameset = pipeline.wait_for_frames()
                frames.append(frameset)

            # 获取相机内参
            intr = frameset.get_depth_frame().profile.as_video_stream_profile().intrinsics

            # 创建时间戳
            time_stamp = datetime.datetime.now()

            # 启动线程
            thread = saveDataThread(
                threadID=thread_id,
                frameset=frames,
                i=thread_id,
                path=output_dir,
                PIG_ID=PIG_ID,
                time_stamp=time_stamp,
                intr=intr
            )
            thread.start()
            thread_id += 1

    except RuntimeError:
        print("End of file reached or error occurred.")
    finally:
        pipeline.stop()

def count_frames_in_bag(bag_file, stream_type=rs.stream.depth):
    """
    计算 .bag 文件中的帧数。

    Args:
        bag_file (str): .bag 文件路径。
    Returns:
        int: 文件中的总帧数。
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_file)

    frame_count = 0

    try:
        pipeline.start(config)
        print(f"Counting frames in {bag_file}...")
        while True:
            frames = pipeline.wait_for_frames()
            frame = frames.first_or_default(stream_type)  # 获取指定类型的流
            if frame:
                print(frame_count)
                frame_count += 1
                
    except RuntimeError:
        # 到达文件结尾会抛出 RuntimeError
        print(f"End of bag file reached. Total frames: {frame_count}")
    finally:
        pipeline.stop()

    return frame_count

def get_stream_profiles(bag_file):
    """
    获取 .bag 文件中数据流的配置，例如分辨率和帧率。

    Args:
        bag_file (str): .bag 文件路径。
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_file)

    try:
        pipeline.start(config)
        profile = pipeline.get_active_profile()
        for stream in profile.get_streams():
            video_stream = stream.as_video_stream_profile()
            print(f"Stream: {stream.stream_type()} | Resolution: {video_stream.width()}x{video_stream.height()} | FPS: {video_stream.fps()}")
    finally:
        pipeline.stop()

def extract_frames_from_bag(bag_file, output_dir):
    """
    从 .bag 文件中提取深度、红外和 RGB 帧，并保存到指定目录。

    Args:
        bag_file (str): .bag 文件路径。
        output_dir (str): 输出保存帧的目录。
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bag_file)

    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    depth_dir = os.path.join(output_dir, "depth")
    infrared_dir = os.path.join(output_dir, "infrared")
    color_dir = os.path.join(output_dir, "color")
    os.makedirs(depth_dir, exist_ok=True)
    os.makedirs(infrared_dir, exist_ok=True)
    os.makedirs(color_dir, exist_ok=True)

    try:
        pipeline.start(config)
        print(f"Extracting frames from {bag_file}...")
        frame_count = 0

        while True:
            frames = pipeline.wait_for_frames()

            # 提取深度帧
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_file = os.path.join(depth_dir, f"depth_{frame_count:04d}.png")
                cv2.imwrite(depth_file, depth_image)

            # 提取红外帧
            infrared_frame = frames.get_infrared_frame()
            if infrared_frame:
                infrared_image = np.asanyarray(infrared_frame.get_data())
                infrared_file = os.path.join(infrared_dir, f"infrared_{frame_count:04d}.png")
                cv2.imwrite(infrared_file, infrared_image)

            # 提取彩色帧
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_file = os.path.join(color_dir, f"color_{frame_count:04d}.jpg")
                cv2.imwrite(color_file, color_image)

            frame_count += 1

    except RuntimeError:
        # 结束文件处理
        print(f"Finished extracting frames. Total frames: {frame_count}")
    finally:
        pipeline.stop()

if __name__ == "__main__":
    # start = time.time()
    # cameraPipeline, cameraConfig = setupCamera()
    # profile = cameraPipeline.start(cameraConfig)
    
    # device = device = profile.get_device()
    # recorder = device.as_recorder()
    # recorder.start_recording("record.bag")
    
    # end_time = time.time()
    # streamSensor(1, cameraPipeline, profile, 1)
    # streamSensorRaw(1, cameraPipeline, cameraConfig, 1, 30)
    
    # print(f"运行时间: {end_time - start} 秒")
    cameral_test()
    # path = 'output_bag/'
    # os.makedirs(path, exist_ok=True)
    # record_bag("output_bag/record.bag", record_time=1)
    
    # bag_file = "/home/pi/Desktop/ROBOTSOFTWARE/Data/Data_Estrus_2024/ID_03_50221/bag/pig_50221_2024_12_09_14_05_57.bag"
    # bag_file = 'output_bag/record.bag'
    # output_dir = "output_bag/processed"
    # PIG_ID = 12345

    # process_bag_file(bag_file, output_dir, PIG_ID)
    
    # 示例调用
    # bag_file = "output/record.bag"
    # total_frames = count_frames_in_bag(bag_file)
    # print(f"Total frames in {bag_file}: {total_frames}")
    # get_stream_profiles(bag_file)
    # extract_frames_from_bag(bag_file, output_dir)
