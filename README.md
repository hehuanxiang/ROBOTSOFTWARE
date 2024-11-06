# SwineLidarRobot 基本控制逻辑

远离电源的方向，motor顺时针旋转，即相较于电源前进。

```python
# 301行附近
action = testStepper.step(5000, "left", 100, docking = True)
handle_stop(action)

# 一开始整个装置的位置在远离电源的最远端，
# reset sensor设置在附近，先让他往左边走（左手边为电源方向）。
# docking = True, 接下来只要reset sensor检测到磁铁，即为初始位置。
```
```python
# 紧接着testStepper会返回docked，开始进入handl_stop
# 234行附近
def handle_stop(sign):
    print("Get in the handle_stop function.")
    if sign != None:
        testStepper = Stepper([29,15,11 ,16,18,37])
        print("Current sign for handle_stop function {}".format(sign))
        if sign == "docked":
            #move forward slightly
            print("docked2")
            action = testStepper.step(3000, "left", 5, docking = False)
            handle_stop(action)
        elif sign == "right_end":
            print("end3")
            #action = testStepper.step(5000, "right", 50, docking = False)
            action = testStepper.step(10000000, "right", 50, docking = True)
            handle_stop(action)

# 由于action此时的值为docked，系统会继续前进一小段距离。
# 距离很短，只有3000 steps，实际上会前进3000 * 1.2 steps
# 结束之后action返回total steps跳出handle_stop的递归
# 按照原始设定，此时系统离第一个stall 还有一段距离，
```
```python
# 按照原始设定，此时系统离第一个stall 还有一段距离，
# 初始化相机，开始进入逐个stall拍摄的循环
while True:
    t1 = datetime.datetime.now()

    if t1.minute % 1 <= 3:
        for i in range (0,s):
            if i ==0:
                #add it back
                # 因为从reset 点出来之后已经走了不少距离
                # 因此第一个stall不需要走那么久，所以step少
                action = testStepper.step(30000, "left", 0.5, docking = False)
                handle_stop(action)
                print("moved to ", i+1)
            else:
                action = testStepper.step(5000, "left", 0.5, docking = False)
                action = testStepper.step(110000, "left", 0.5, docking = False)
                handle_stop(action)
                print("moved to ", i+1)

            pigID = i
                    #initPYGAME(pigID+1)
            if pigNumber[i]!=999:
                       # streamSensor(pigNumber[i])

                try:
                    #streamSensor(pigNumber[i],pipelines[1],configurations[1])
                    streamSensor(pigNumber[i])

                except:
                            
                    print("failed to initialize camera")
                    sleep(3)
                    #camera_id,intrinsics,configurations,pipelines=get_sensor()
                    print("failed to initialize camera")
                    sleep(3)
                    pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                    config.enable_stream(rs.stream.infrared,0,640,480,rs.format.y8,30)


                if pigNumber[i]==999:
                    sleep(1)            
                #capturePictures()

        # 走到尽头之后，直接反走，从左到右走到尽头，回到reset的点                
        action = testStepper.step(150000*24, "right", 1000, docking = True)
        handle_stop(action)
        print("return to dock")
```

# Specification for Lidar L515
Just run in the terminal
```bash
rs-enumerate-devices
```