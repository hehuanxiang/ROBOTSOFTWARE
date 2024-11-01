from OperationSetting_GUI import operation_setting_gui
import PySimpleGUI as sg
#import csv
from os.path import exists
import numpy as np
from Setup_Farm_Layout_GUI import setup_farm_layout_gui
from re import search
import json
from datetime import date, datetime
#import re
#import itertools
import ast
import os
import cv2
import scipy.io
import imageio

from helper_functions import cv_find_chessboard, get_chessboard_points_3D, get_depth_at_pixel, convert_depth_pixel_to_metric_coordinate
from realsense_device_manager import DeviceManager, post_process_depth_frame

import pyrealsense2.pyrealsense2 as rs
from helper import saveDataThread
from time import sleep
import faulthandler

def main_robot_gui_window(ids, monitoring, stall_sow_id):
    c,r = ids.shape
    stall_summary = [[sg.Frame("Monitoring status", 
        [
            #[sg.OptionMenu(values = sow_lists, default_value = ids_s[i][j],size = (int(w/(10*c*4)),3)) for i in range(c)] for j in range(r)
            [sg.Text(ids[i][j] +": "+ str(stall_sow_id[i][j]), size = (12,1), key="stall_sows"+str(i)+str(j), background_color = ["lightgray", "green"][monitoring[i,j]]) for i in range(c)] for j in range(r)
        ])
        ], [sg.Button("Edit",size = (12,1))]
    ]
    
    robot_image_control = [[sg.Frame("Robot Control",[
                                     [sg.Text("Manual Control"),  sg.Button("Stream_L"),sg.Button("Stream_R"),sg.Button("STOP STREAM"), sg.Button("Capture")],
                                     #[sg.Text("Image Viewer")],
                                     [sg.Image(filename="", key='image', size = (300,300)), sg.Text(key='-info-')],
                                     [sg.Text("Auto Pilot"), sg.Button("Start"), sg.Button("Pause"), sg.Button("Resume"), sg.Button("End")]
                                     ]
                                     )
                            ]]
    
    robot_window_layout = [[
            [sg.Column(stall_summary, expand_y = True,size=(250,80)),
            sg.VSeparator(),
             sg.Column(robot_image_control, expand_y = True, size = (950,80))
             ],
            [sg.HSeparator(pad =(5,5))]
         ]]
    window = sg.Window(title="SOW ROBOT", layout=robot_window_layout, resizable=True, no_titlebar=False, size=(1000,650), auto_size_text = True, element_justification='c',finalize=True)
    return window
def saveData(frames, intr, folder_location, fileName):
    depth_threshold=rs.threshold_filter(min_dist=0.15, max_dist=2.5)
    colorizer=rs.colorizer()        

    color_frame=frames.get_color_frame()
    depth_frame=frames.get_depth_frame()
    ir_frame=frames.get_infrared_frame()
        #depth image
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    ir_image = np.asanyarray(ir_frame.get_data())
        
        #depth_frame=depth_threshold.process(depth_frame)
    filtered_depth_frame=post_process_depth_frame(depth_frame, temporal_smooth_alpha=0.1, temporal_smooth_delta=50)

    XX=np.zeros((480,480))
    YY=np.zeros((480,480))
    ZZ=np.zeros((480,480))
       
    coverage = [0]*64
    for y in range(480):
        for x in range(480):
            dist = depth_frame.get_distance(x+80, y)
                #print(dist)
            [X,Y,Z] = convert_depth_pixel_to_metric_coordinate(dist, x,y, intr)
            XX[y,x] = X
            YY[y,x] = Y
            ZZ[y,x] = Z
    obj=np.stack((XX,YY,ZZ))
    matfile=folder_location +"/"+"DM" +fileName+ ".mat"
    scipy.io.savemat(matfile, mdict={'out': obj}, oned_as='row')
    imageio.imwrite(folder_location+"/"+"DP"+fileName+".png", depth_image)
    imageio.imwrite(folder_location+"/"+"IR"+fileName+".png", ir_image)
    imageio.imwrite(folder_location+"/"+"RGB"+fileName+".png", color_image)
    
def read_config():
    ids = np.genfromtxt('Config/layout_id.csv', delimiter = ',', dtype = None, encoding=None)
    monitoring = np.genfromtxt('Config/monitor_status.csv', dtype = int, delimiter = ',')
    stall_sow_id = np.genfromtxt('Config/sow_stall_id.csv', dtype = int, delimiter = ',')

    if len(ids.shape)==1:
        ids = np.transpose(ids.reshape(-1,1))
        monitoring = np.transpose(monitoring.reshape(-1,1))
        stall_sow_id = np.transpose(stall_sow_id.reshape(-1,1))

    mask = stall_sow_id>0
    stall_sow_id = stall_sow_id.astype(str)
    stall_sow_id[mask] = (np.char.zfill(stall_sow_id[mask], 5))
    return ids, monitoring, stall_sow_id
#print(stall_sow_id)

faulthandler.enable()    
    
    
setting_exists = exists("Config/farm_setting.txt") or exists("Config/layout_id.csv")
if not setting_exists:
    setup_farm_layout_gui()

ids, monitoring, stall_sow_id = read_config()
if sum(monitoring.flatten())==0:
    #print(sum(monitoring.flatten())==0)
    operation_setting_gui(ids, monitoring, stall_sow_id)
    
ids, monitoring, stall_sow_id = read_config()
c,r = ids.shape
main_robot_windows = main_robot_gui_window(ids, monitoring,stall_sow_id)
stall_id = 0
stream = False
stream_one = False
autopilot = False
move_robot_forward = False
select_camera = False
leave_docking =False
return_to_dock = False
e2=0
e3=0
###Configure Sensor
ctx = rs.context()
#print(ctx)
camera_id = []
intrinsics = []
configurations = []
for d in ctx.devices:
    print(d.get_info(rs.camera_info.serial_number))
    camera_id.append(d.get_info(rs.camera_info.serial_number))
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(d.get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
    config.enable_stream(rs.stream.color, 1280,720, rs.format.bgr8,30)
    config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)
    profile = pipeline.start(config)
    get_intrinsic = profile.get_stream(rs.stream.depth)
    intrinsics.append(get_intrinsic.as_video_stream_profile().get_intrinsics())
    pipeline.stop()
    configurations.append(config)
#align_to = rs.stream.depth
#align = rs.align(align_to)
#sleep(1)
print(intrinsics)

main_robot_windows["Capture"].update(disabled = True)
empty_image = np.zeros((250,250))
emp_imgbytes = cv2.imencode('.png', empty_image)[1].tobytes()
main_robot_windows['image'].update(data=emp_imgbytes)



row = -1
col = 0
while True:
    event, values = main_robot_windows.read(timeout=30) #10 12hours
    #print(event)
    if len(camera_id)==1:
        main_robot_windows["Stream_R"].update(disabled = True)
    

    if event == sg.WIN_CLOSED or event == "Exit":
        main_window.close()
        break
    if event == "Edit":
        operation_setting_gui(ids, monitoring, stall_sow_id)
        ids, monitoring, stall_sow_id = read_config()
        for i in range(ids.shape[0]):
            for j in range(ids.shape[1]):
                main_robot_windows["stall_sows"+str(i)+str(j)].update(background_color=["lightgray", "green"][int(monitoring[i,j])])
                
    #if event == "Stream":
            
    if search("Stream_", event):
        # Start streaming
        pipeline = rs.pipeline()
        config = rs.config()
        if event == "Stream_L":
            config.enable_device(camera_id[0])
            main_robot_windows["Stream_L"].update(disabled = True)
            main_robot_windows["Stream_R"].update(disabled = False)

        elif event =="Stream_R":
            config.enable_device(camera_id[1])
            main_robot_windows["Stream_R"].update(disabled = True)
            main_robot_windows["Stream_L"].update(disabled = False)
       
        try:
            pipeline.stop()
        except:
            e=0
            #print("")
        profile = pipeline.start(config)
        get_intrinsic = profile.get_stream(rs.stream.depth)
        intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
        #depth_sensor = profile.get_device().first_depth_sensor()
        #depth_scale = depth_sensor.get_depth_scale()
        #temp_filter = rs.temporal_filter(1,1,3)
        #clipping_distance_in_meters = 1 #1 meter
        #clipping_distance = clipping_distance_in_meters / depth_scale
        align_to = rs.stream.depth
        align = rs.align(align_to)

        stream = True
        main_robot_windows["Capture"].update(disabled = False)

        
    if event == "STOP STREAM":
        stream = False
        main_robot_windows["Capture"].update(disabled = True)
        try:
            pipeline.stop()
            
        except:
            e=1
        #try:
            #cv2.destroyAllWindows()
        #except:
            #e=1
        main_robot_windows["Stream_L"].update(disabled = False)
        main_robot_windows["Stream_R"].update(disabled = False)
        main_robot_windows['image'].update(data=emp_imgbytes)

    if event =="Capture":
        stream = False
        folder_path = sg.popup_get_folder("select folder for destination")
        if folder_path != None:
            file_name = sg.popup_get_text("Enter File Name (Will save depth image, RGB image, and IR image)")
        if file_name != None:
            try:
                thread1=saveDataThread(1, aligned_frames,2,intr,folder_path, file_name)
                thread1.start()
            except:
                e=1
                #print("Failed to save image")
        else:
            sg.popup_ok("Didnt enter file name")
        try:
            pipeline.stop()
        except:
            print("")
    if event == "Start":
        main_robot_windows["Stream_L"].update(disabled = True)
        main_robot_windows["Stream_R"].update(disabled = True)
        main_robot_windows["STOP STREAM"].update(disabled = True)
        main_robot_windows["Start"].update(disabled = True)
        print("wait till 5 minute")

        leave_docking = True          
    if event == "End":
        autopilot = False
        move_robot_forward = False
        select_camera = False
        stream_one = False
        main_robot_windows['image'].update(data=emp_imgbytes)
        main_robot_windows["Stream_L"].update(disabled = False)
        main_robot_windows["Stream_R"].update(disabled = False)
        main_robot_windows["STOP STREAM"].update(disabled = False)
        main_robot_windows["Start"].update(disabled = False)

        try:
            pipeline.stop()
        except:
            e=0
        #print("END")
    if stream:
        
        frames = pipeline.wait_for_frames(5000)
        aligned_frames = align.process(frames)
        # Get aligned frames
        #aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        depth_frame = aligned_frames.get_depth_frame()
        if not depth_frame:
            continue
        #filtered = temp_filter.process(depth_frame)
        depth_image = np.asanyarray(depth_frame.get_data())
        #print(depth_image.shape)
        depth_image = depth_image[:, 80:560]
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap = cv2.resize(depth_colormap, dsize = (250,250), interpolation = cv2.INTER_CUBIC)
        imgbytes = cv2.imencode('.png', depth_colormap)[1].tobytes()
        main_robot_windows['image'].update(data=imgbytes)      

    #if autopilot:
        #move_robot_forward = True
        
    if leave_docking == True:
        if(datetime.now().minute % 2 ==0):
            print("start")
            move_robot_forward = True
            leave_docking= False
            stream_one = False
            select_camera = False
            return_to_dock = False
    if return_to_dock == True:
        print("returning to dock")
        return_to_dock = False
        leave_docking = True
        stream_one=False
        move_robot_forward=False
        select_camera = False

        row = -1
    if move_robot_forward == True:
        main_robot_windows['image'].update(data=emp_imgbytes)
        print("Start Moving Robot")
        
        row = row+1
        col=0
        sleep(5)
        return_to_dock = False
        leave_docking = False
        stream_one=False
        select_camera = True
        move_robot_forward = False
        if row>=r:
            print("return robot to station")
            row = -1           
            return_to_dock = True
            select_camera = False
            move_robot_forward = False
        print("Finished Moving Robot")

    
    if select_camera == True:
        print("selecting camera")
        try:
            pipeline.stop()
        except:
            e=0
            #print("e stop pipeline select sensor")

        print(col)
        if sum(monitoring[:,row]) ==0:
            select_camera = False
            stream_one = False
            move_robot_forward = True
            print("skip line")

        else:
            ctx = rs.context()
            t=0
            #print("number of cameras: ", len(ctx.devices))
            
            if sum(monitoring[:,row])==2:
                main_robot_windows["-info-"].update(ids[col,row])
                print("HERE",ids[col,row], " time ", datetime.now())
                
                #config.enable_device(camera_id[col])
                pipeline.start(configurations[col])
                intr = intrinsics[col]
                f_count = 0
                stall_id = ids[col,row]
                col = col+1
                align_to = rs.stream.depth
                align = rs.align(align_to)
                print("s")
                stream_one = True

            elif sum(monitoring[:,row]) ==1:
                temp = [int(item) for item in monitoring[:,row]]
                col = temp.index(1)
                main_robot_windows["-info-"].update(ids[col,row])
                stall_id = ids[col,row]
                print("HERE",ids[col,row], " time ", datetime.now())
                #config.enable_device(camera_id[col])     #Likely this line, segmentatino fault
                pipeline.start(configurations[col])
                intr = intrinsics[col]
                f_count = 0
                col = len(camera_id)
                align_to = rs.stream.depth
                align = rs.align(align_to)
                print("s")
                stream_one = True
            else:
                print(44044)

            
            #get_intrinsic = profile.get_stream(rs.stream.depth)
            #intr=get_intrinsic.as_video_stream_profile().get_intrinsics()
            #print(intr)
            #temp_filter = rs.temporal_filter(1,1,3)
            #align_to = rs.stream.depth
            #align = rs.align(align_to)
            #sleep(1)
            select_camera= False
            #stream_one = True
            print("selected camera")
    
    if stream_one:
        if f_count == 0:
            print("start streaming")
            #e=1
            
        if f_count <50:
            #print(f_count)
            try:
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                if not depth_frame:
                    continue
                depth_image = np.asanyarray(depth_frame.get_data())
                depth_image = depth_image[:, 80:560]
                depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                depth_colormap = cv2.resize(depth_colormap, dsize = (250,250), interpolation = cv2.INTER_CUBIC)
                imgbytes = cv2.imencode('.png', depth_colormap)[1].tobytes()
                main_robot_windows['image'].update(data=imgbytes)
                f_count = f_count+1
            except:
                e2=e2+1
                #print("e2: ", e2)
                stream_one = False
                try:
                    pipeline.stop()
                except:
                    e=0
                select_camera = True
                col = col-1
                f_count = 60
                sleep(5)

        else:
            print("finish stream, saving data")
            #saveData(aligned_frames,intr, "/home/pi/Desktop/ROBOTSOFTWARE/Capture", stall_id)

            try:
                #print("thread", col+2*row)
                thread1=saveDataThread((col+2*row), aligned_frames,intr, "./Capture", stall_id)
                thread1.start()
                #saveData(aligned_frames,intr, "/home/pi/Desktop/ROBOTSOFTWARE/Capture", stall_id)
                e=0
            except:
                e3=e3+1
                #print("e3: ", e3)

            stream_one = False

            select_camera = True
            if col == len(camera_id):
                print("dont select camera, move forward")
                select_camera = False
                move_robot_forward = True
            if row>=r:
                print("return robot to station")
                row = -1           
                return_to_dock = True
                select_camera = False
                move_robot_forward = False
                stream_one=False
                
            
    
                           

                






                    