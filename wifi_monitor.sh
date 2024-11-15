#!/bin/bash

# 定义 Python 邮件发送脚本路径
PYTHON_SCRIPT="/home/pi/Desktop/ROBOTSOFTWARE/IP_email.py"

# 记录上一次的网络状态，初始值为空
PREVIOUSLY_CONNECTED=false


join_zerotier() {
    # 启动 ZeroTier 服务并加入网络（首次连接或重新连接后执行）
    sudo systemctl start zerotier-one
    sudo zerotier-cli join 60ee7c034abdb3c0
    sleep 10  # 等待几秒以确保 ZeroTier 网络已连接

}

# 定义检查网络连接并发送邮件的函数
check_and_connect_zerotier() {
    # 检查是否已连接到互联网
    if ping -c 1 google.com &> /dev/null; then
        echo "WiFi connected."

        # 如果上一次的状态是断开或是开机第一次运行，发送邮件
        if [ "$PREVIOUSLY_CONNECTED" = false ]; then
            echo "Connected to WiFi. Sending notification email..."
            python $PYTHON_SCRIPT  # 发送邮件
            PREVIOUSLY_CONNECTED=true  # 更新为连接状态

            # 启动 ZeroTier 
	    join_zerotier
        fi
        return 0  # 表示连接成功
    else
        echo "WiFi disconnected."

        # 如果之前是连接状态，更新状态为断开状态
        PREVIOUSLY_CONNECTED=false

        # 尝试重新连接 WiFi，最多尝试3次
        for i in {1..3}; do
            echo "Attempting to reconnect WiFi (attempt $i)..."
            sudo ip link set wlan0 down
            sudo ip link set wlan0 up
            sleep 10  # 等待几秒

            # 检查是否重新连接成功
            if ping -c 1 google.com &> /dev/null; then
                echo "Reconnected to WiFi. Sending notification email..."
                python $PYTHON_SCRIPT  # 发送邮件
                PREVIOUSLY_CONNECTED=true
		join_zerotier
                break
            fi
        done

        if [ "$PREVIOUSLY_CONNECTED" = false ]; then
            echo "Failed to reconnect WiFi after multiple attempts."
        fi

        return 1  # 表示连接断开
    fi
}

# 开机时尝试连接网络并发送邮件
sleep 30
echo "Checking initial network connection..."
check_and_connect_zerotier

# 每 10 分钟检查网络连接
while true; do
    echo "Waiting 10 minutes before next check..."
    sleep 600  # 等待 10 分钟 (600 秒)
    echo "Checking network connection..."
    check_and_connect_zerotier
done

