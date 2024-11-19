import io, serial, serial.tools.list_ports, socket, struct, sys, datetime, os
import RPi.GPIO as GPIO
from PIL import Image
from time import sleep
import threading
import queue
from Stepper import Stepper
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2
from pyntcloud import PyntCloud
import argparse
import imageio
from realsense_device_manager import DeviceManager, post_process_depth_frame
import time
from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate
import scipy.io
import traceback
import json

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

def streamSensor(pigID, cameraPipeline, cameraConfig, stallId):
    
    pipeline = cameraPipeline
    config = cameraConfig
    pig_ID=pigID
    print("ID:"+str(pig_ID))
    
    # str.zfill(2)：
    # 将 pigID 转换为字符串，不足两位时在前面填充 0。
    # 示例：1.zfill(2) -> "01"。
    path ="./Data/Data_Estrus_2024/" +"ID_" +str(stallId).zfill(2)+ "_" + str(pigID) + "/"#+imgname

    try:
        if not os.path.exists(path):
            os.makedirs(path)
    except:
        path ="./Data/Data_Estrus_2024/" + "ID_" + str(stallId).zfill(2) + "_" + str(pigID) + "/"#+imgname

        if not os.path.exists(path):
            os.makedirs(path)

    try:
        profile = pipeline.start(config)
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
        #aligned_ir_frame=aligned_frames.get_infrared_frame()
        #color_frame = aligned_frames.get_color_frame()
            if not aligned_depth_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        #color_image = np.asanyarray(color_frame.get_data())
        #ir_image=np.asanyarray(aligned_ir_frame.get_data())        
        #images = np.hstack((ir_image,ir_image))

            # 2024.11.1
            # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('Align Example', depth_colormap)
            key = cv2.waitKey(1)

            if x%interval==0: #60
            #depth_frameset.append(aligned_depth_frame)
            #color_frameset.append(color_frame)
            #ir_frameset.append(aligned_ir_frame)
                print(int(x/interval))
                frameset.append(aligned_frames)
            #thread1=saveDataThread((x+interval)/interval, aligned_frames,int(x/interval))
            #thread1.start()
            
            #saveData(aligned_frames, x/30)
        #print(x)
       
    finally:   
        pipeline.stop()
    try:
        thread1=saveDataThread((x+interval)/interval, frameset,int(x/interval),path, pig_ID, t,intr)
        thread1.start()
        sleep(2)
    except:
        thread1.stop()
    print("captured succeed, trying to save")

def handle_stop(sign):
    print("Get in the handle_stop function.")
    if sign != None:
        testStepper = Stepper([STEP,DIR,ENA,endPin,resetPin,magnetPin]) 
        print("Current sign for handle_stop function {}".format(sign))
        if sign == "docked":
            #move forward slightly
            print("docked2")
            action = testStepper.step(3000, "left", 0.05, docking = False)
            handle_stop(action)

            
        elif sign == "right_end":
            print("end3")
            #action = testStepper.step(5000, "right", 50, docking = False)
            action = testStepper.step(10000000, "right", 50, docking = True)        
            handle_stop(action)

def get_sensor():
    ctx = rs.context()
        #print(ctx)
    camera_id = []
    intrinsics = []
    configurations = []
    pipelines =[]
    for d in ctx.devices:
        print(d.get_info(rs.camera_info.serial_number))
        camera_id.append(d.get_info(rs.camera_info.serial_number))
            
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(d.get_info(rs.camera_info.serial_number))
        config.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
        config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
        config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)

        pipelines.append(pipeline)
        profile = pipeline.start(config)
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intrinsics.append(get_intrinsic.as_video_stream_profile().get_intrinsics())
        pipeline.stop()
        configurations.append(config)
    return camera_id,intrinsics,configurations,pipelines

def setupCamera():
    cameraPipeline = rs.pipeline()
    cameraConfig = rs.config()
    cameraConfig.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    cameraConfig.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cameraConfig.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)
    
    return cameraPipeline, cameraConfig


if __name__ == "__main__":
    # 读取猪的id和点击配置PIN码
    with open('/home/pi/Desktop/ROBOTSOFTWARE/farm_config.json', 'r') as file:
        farm_config = json.load(file)
        
    # 设置猪的ID
    pigNumber = farm_config["pigNumber"]
    
    # 读取电机的控制以及各个sensor的pin码
    pins = farm_config["pins"]

    endPin = pins["endPin"]
    resetPin = pins["resetPin"]
    magnetPin = pins["magnetPin"]
    DIR = pins["DIR"]
    ENA = pins["ENA"]
    STEP =pins["STEP"]
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(DIR,GPIO.OUT)
    GPIO.setup(ENA,GPIO.OUT)
    GPIO.setup(STEP,GPIO.OUT)
    GPIO.output(ENA,GPIO.HIGH)
    
    oldtime = datetime.datetime.now()

    testStepper = Stepper([STEP,DIR,ENA,endPin,resetPin,magnetPin])         # the true pin number
    print("robot start in 2 sec, stop it now to manually move the robot")
    sleep(2)

    # when docking is set as true, it 
    # action = testStepper.step(110000*24, "left", 100, docking = True)    # original one
    action = testStepper.step(5000, "left", 0.1, docking = True)
    print(action)
    handle_stop(action)
    print("returning to dock")
    #camera_id,intrinsics,configurations,pipelines=get_sensor()
  
    # set up cammera
    cameraPipeline, cameraConfig = setupCamera()
    
    stallNumber = farm_config["stallNumber"]        
    while True:
        t1 = datetime.datetime.now()

        if t1.minute % 10 == 0:             # 每十分钟拍一次
            start_time = time.time()
            for i in range (0,stallNumber):
                if i ==0:
                    #add it back
                    # action = testStepper.step(30000, "left", 0.05, docking = False)
                    action = testStepper.step(100000, "left", 0.05, docking = False)     # 2024年11月15日13点32分
                    handle_stop(action)
                    print("moved to ", i+1)
                else:

                    action = testStepper.step(5000, "left", 0.05, docking = False)
                    # action = testStepper.step(110000, "left", 0.5, docking = False)
                    
                    # 猪场的设定是110000，对于lab的测试环境，每个stall的距离大约是20000，因此需要减小steps
                    action = testStepper.step(100000, "left", 0.05, docking = False)
                    handle_stop(action)
                    print("moved to ", i+1)

                pigID = i
                        #initPYGAME(pigID+1)
                if pigNumber[i]!=9998:          # 设定某个为不拍摄的id，可以补齐周期，比如总共有20头猪
                    try:
                        print("pigID is {}".format(pigID))
                        streamSensor(pigNumber[i], cameraPipeline, cameraConfig, pigID)
                    except Exception as e:
                        print("An error occurred:")
                        traceback.print_exc()


                    if pigNumber[i]==999:
                        sleep(1)            
                    #capturePictures()
            end_time = time.time()
            total_time = end_time - start_time
            print(f"Finish one imaging cycle in {total_time}")
                            
            action = testStepper.step(150000*24, "right", 1000, docking = True)
            handle_stop(action)
            print("return to dock")
         
