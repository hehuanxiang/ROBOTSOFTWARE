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

# L515 帧数取舍以及深度帧和RGB帧对齐
每次停留的时候启动函数一次
```python
def streamSensor(pigID)
```

```python
    # 由于RealSense 相机中的深度帧是通过红外传感器获取的，相机会发出红外光，
    # 然后用两个红外传感器（如 D435）或者一个红外传感器（如 L515）进行计算。
    # 红外帧与深度帧已经有很强的关联性，
    # 通常在硬件层面上它们的坐标是一致的，不需要额外对齐。
    # 因此这里只需要将Depth和RGB信息进行对其
    align_to = rs.stream.depth
    align = rs.align(align_to)
    
    tt = time.monotonic()
    t = datetime.datetime.now()

    frameset=[]
    interval=20

    # 在原系统中，对于每只猪只保留了一帧的图像，realsense最高支持30帧拍摄。
    for x in range(interval*1):
        frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image
        st =int(time.monotonic()-tt)
        if st >= 21:
            print("captured failed")
            break

        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() # 
        if not aligned_depth_frame:
            continue
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        key = cv2.waitKey(1)

        if x%interval==0: #60
            print(int(x/interval))
            frameset.append(aligned_frames)
```



# Specification for Lidar L515
Just run in the terminal
```bash
rs-enumerate-devices
```

# 使用SSH远程连接树莓派
+ 在树莓派上打开终端，输入以下命令启用SSH：
    ```bash
    sudo raspi-config
    ```
+ 选择 Interface Options，然后选择 SSH 并启用。
+ 找到树莓派的IP地址
    ```bash
    hostname -I
    ```
+ 在另一台电脑上通过SSH连接

# 更改树莓派的默认python版本
+ 检查系统中已安装的 Python 版本
    ```bash
    ls /usr/bin/python*
    ```
+ 添加 Python 版本到 alternatives： 执行以下命令以将 Python 3.7 添加到 update-alternatives 的管理中
    ```bash
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
    sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.7 2

    ```
+ 选择默认版本
    ```bash
    sudo update-alternatives --config python
    ```
    系统会列出已配置的 Python 版本。输入相应的数字选择 Python 3.7 并按回车。
+ 确认版本
    ```bash
    python --version
    ```
# VScode免密连接树莓派
## 生成SSH密钥对
+ 打开终端（Mac/Linux）或 PowerShell（Windows）
    ```bash
    ssh-keygen -t rsa -b 4096
    ```
+ 按提示设置密钥保存路径（默认为 ~/.ssh/id_rsa），如果已经有密钥，可以覆盖或选择新的路径。
+ 创建密钥时，可以直接按回车跳过密码设置，以便实现免密登录。
## 公钥复制到树莓派
+ 显示公钥内容： 在 PowerShell 中，运行以下命令显示公钥内容（假设公钥存放在 ~/.ssh/id_rsa.pub）
    ```bash
    Get-Content ~/.ssh/id_rsa.pub
    ```
+ 输入密码登录后，在树莓派上创建 ~/.ssh 文件夹（如果不存在），并将公钥添加到 authorized_keys 文件中
    ```bash
    mkdir -p ~/.ssh
    echo "粘贴你复制的公钥内容" >> ~/.ssh/authorized_keys
    chmod 600 ~/.ssh/authorized_keys
    chmod 700 ~/.ssh
    ```

# MobaXterm免密连接树莓派
## 生成SSH密钥对
+ 打开 MobaXterm。
+ 进入 Tools > MobaKeyGen，打开密钥生成工具。
+ 在 MobaKeyGen 窗口中，点击 Generate 按钮生成一对 SSH 密钥（默认会生成 RSA 密钥）。
+ 生成完成后，点击 Save public key 将公钥保存为 id_rsa.pub，再点击 Save private key 将私钥保存为 id_rsa。
+ 你也可以直接复制 Public key 区域的内容。

## 公钥复制到树莓派
同上

## 配置 MobaXterm 使用私钥
+ 在 MobaXterm 的左侧，右键点击 Sessions > New session > SSH。
+ 在弹出的窗口中，输入树莓派的 IP 地址。
+ 在同一窗口的下方，找到 Advanced SSH settings。
    + 勾选 Use private key。
    + 选择你刚才保存的私钥文件（id_rsa）。
+ 点击 OK 保存设置。

# ZerotTier使用（针对WSL, Windows可以直接软件连接）
+ 启动ZeroTier功能】
    ```bash
    sudo systemctl start zerotier-one
    ```
+ 加入ZeroTier网络
    ```bash
    sudo zerotier-cli join <network_id> # network_id = 60ee7c034abdb3c0
    ```
+ 检查连接状态
    ```bash
    sudo zerotier-cli info
    ```

# 远程连接GitHub
## 使用个人访问令牌 (PAT)（推荐）
1. 生成个人访问令牌：

   + 打开 GitHub 的 个人访问令牌设置页面。
   + 点击 Generate new token（生成新令牌）。
   + 选择所需的权限范围（例如，repo 访问权限用于管理仓库）。
   + 生成后保存该令牌（令牌只会显示一次，记得保存）。
2. 使用令牌替代密码： 进行 git push 时，输入以下内容：
   ```bash
    用户名: hehuanxiang
    密码: <你的个人访问令牌>
   ```
3. 保存令牌以免每次输入（可选）： 执行以下命令保存认证信息：
    ```bash
    git config --global credential.helper store
    ```

## 使用 SSH 密钥
1. 生成SSH秘钥：
    ```bash
    ssh-keygen -t rsa -b 4096 -C "你的邮箱@example.com"
    ```
   根据提示操作，建议直接回车，留空密钥密码。
2. 添加 SSH 密钥到 GitHub：
    + 查看生成的秘钥：
        ```bash
        cat ~/.ssh/id_rsa.pub
        ```
    + 复制输出的内容。
    + 打开 GitHub 的 SSH 密钥设置页面。
    + 点击 New SSH key，将公钥粘贴到文本框中并保存。
3. 修改远程仓库地址为 SSH： 使用以下命令更改远程仓库地址：
    ```bash
    git remote set-url origin git@github.com:hehuanxiang/ROBOTSOFTWARE.git
    ```
4. 测试连接： 确认 SSH 配置是否成功：
    ```bash
    ssh -T git@github.com
    ```
5. 推送代码
    ```bash
    git push
    ```

# tmux使用
1. 打开终端，启动一个新的tmux会话并命名：
    ```bash
    tmux new -s swine_lidar
    ```
    + swine_lidar 是会话名称，可以根据需要更改。
2. 在tmux会话中运行python脚本并保存日志
    ```bash
    python /home/pi/Desktop/ROBOTSOFTWARE/SwineLidarRobot_test_1031.py > /home/pi/Desktop/ROBOTSOFTWARE/swine_lidar.log 2>&1
    ```
    + \> /home/pi/Desktop/ROBOTSOFTWARE/swine_lidar.log 将标准输出保存到日志文件。
    + 2>&1 将标准错误重定向到标准输出，确保所有日志都保存到 swine_lidar.log 文件。
3. 断开 tmux 会话
    当脚本在 tmux 中运行时，你可以断开会话，让它继续在后台运行：

    + 按下 Ctrl + B，然后松开，再按 D 键。
    + 你会回到原来的终端，tmux 会话仍在后台运行。
4. 查看实时日志
    + 方法1 重新连接到 tmux 会话
      + 查看所有正在运行的 tmux 会话：
        ```bash
        tmux ls
        ```
      + 重新连接到 swine_lidar 会话：
        ```bash
        tmux attach-session -t swine_lidar
        ```
    + 方法2 使用 tail -f 命令查看日志文件
        ```bash
        tail -f /home/pi/Desktop/ROBOTSOFTWARE/swine_lidar.log
        ```
        + tail -f 实时追踪日志文件的更新。
        + 按 Ctrl + C 退出实时查看。