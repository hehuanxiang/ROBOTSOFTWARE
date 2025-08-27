#!/bin/bash

SESSION_NAME="sowbot"
SCRIPT_PATH="/home/pi/Desktop/ROBOTSOFTWARE/arduino/sowbot_serial.py"

# 如果会话已存在，不重复创建
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    echo "⚠️ tmux 会话 '$SESSION_NAME' 已存在，跳过创建。"
else
    echo "✅ 创建 tmux 会话并运行 sowbot_serial.py ..."
    tmux new-session -s $SESSION_NAME -d "python $SCRIPT_PATH"
fi
