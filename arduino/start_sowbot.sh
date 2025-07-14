#!/bin/bash

SESSION_NAME="sowbot"

# 检查是否已存在同名 tmux 会话
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "⚠️ tmux 会话 '$SESSION_NAME' 已存在，附加连接中..."
    # tmux attach-session -t $SESSION_NAME
else
    echo "✅ 创建新的 tmux 会话并运行 sowbot_serial.py ..."
    tmux new-session -s $SESSION_NAME -d "python3 /home/pi/Desktop/ROBOTSOFTWARE/arduino/sowbot_serial.py"
    sleep 1
    # tmux attach-session -t $SESSION_NAME
fi
