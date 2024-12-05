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
    config.enable_stream(rs.stream.depth, 1024, 768, rs.format.z16, 30)  # 深度流
    config.enable_stream(rs.stream.color, 1920,1080, rs.format.bgr8, 30)  # 彩色流
    config.enable_stream(rs.stream.infrared, 0, 1024,768, rs.format.y8, 30)  # 红外流
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
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
        # depth_sensor = profile.get_devicedepth_sensor().first_depth_sensor()
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor.set_option(rs.option.min_distance,0)
        depth_sensor.set_option(rs.option.enable_max_usable_range,0)
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , depth_scale)
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
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

if __name__ == "__main__":
    # start = time.time()
    # cameraPipeline, cameraConfig = setupCamera()
    # profile = cameraPipeline.start(cameraConfig)
    # end_time = time.time()
    # streamSensor(1, cameraPipeline, profile, 1)
    # # streamSensorRaw(1, cameraPipeline, cameraConfig, 1, 30)
    
    # print(f"运行时间: {end_time - start} 秒")
    cameral_test()