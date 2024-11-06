import pyrealsense2.pyrealsense2 as rs
import os
import time
import datetime
import cv2
import numpy as np
from time import sleep
import scipy.io
import threading
import sys
sys.path.append('/home/dayi/Desktop/ROBOTSOFTWARE')
from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate
import imageio

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 848,480, rs.format.z16,30)
config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
config.enable_stream(rs.stream.infrared,1,848,480,rs.format.y8,30)
config.enable_stream(rs.stream.infrared,2,848,480,rs.format.y8,30)

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
        
            #depth_frame=depth_threshold.process(depth_frame)
            #filtered_depth_frame=post_process_depth_frame(depth_frame, temporal_smooth_alpha=0.1, temporal_smooth_delta=50)
            colorized=colorizer.process(ir_frame)
            #pc=rs.pointcloud()
            #pc.map_to(ir_frame)
            #points=pc.calculate(filtered_depth_frame)
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
        #print(index)
            #if not os.path.exists(path+"pcl"):
             #   os.makedirs(path+"pcl")
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

def streamSensor(pigID):
    ###########################################################################################Set saving path
    
    
    pig_ID=pigID
    print("ID:"+str(pig_ID))
    #path ="/media/pi/9F4B-4B4A/Data_Estrus/" +"ID_"+ str(pigID) + "/"#+imgname
    path ="./Data/Data_Estrus/" +"ID_"+ str(pigID) + "/"#+imgname

    try:
        if not os.path.exists(path):
            os.makedirs(path)
            print("Create the folder {}".format(path))
    except:
        path ="./Data/Data_Estrus_2022/" + "ID_"+str(pigID) + "/"#+imgname

        if not os.path.exists(path):
            os.makedirs(path)

    try:
        profile = pipeline.start(config)
        colorizer=rs.colorizer()
    # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_devicedepth_sensor().first_depth_sensor()
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
    #for x in range(30):
        #pipeline.wait_for_frames()
        
        tt = time.monotonic()
        t = datetime.datetime.now()
    #depth_frameset=[]
    #color_frameset=[]
    #ir_frameset=[]
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

            cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('Align Example', depth_colormap)
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

if __name__ == "__main__":
    streamSensor(111)