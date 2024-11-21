#!/bin/bash

LOG_FILE="/home/pi/Desktop/ROBOTSOFTWARE/test.log"  # 定义日志文件路径

# 主任务函数：运行 Python 脚本并记录启动时间
main_task() {
  echo "===== Python 脚本启动于 $(date '+%Y-%m-%d %H:%M:%S') =====" >> "$LOG_FILE"
  python /home/pi/Desktop/ROBOTSOFTWARE/test.py >> "$LOG_FILE" 2>&1 &
  MAIN_PID=$!  # 保存主任务的进程 ID
  echo "Python 脚本已启动 (PID: $MAIN_PID)"
}

# 停止主任务函数
stop_task() {
  if ps -p $MAIN_PID > /dev/null; then
    echo "Python 脚本停止于 $(date '+%Y-%m-%d %H:%M:%S')" >> "$LOG_FILE"
    kill $MAIN_PID
    echo "Python 脚本已停止 (PID: $MAIN_PID)"
  else
    echo "主任务未运行，无需停止"
  fi
}

# 键盘监听器
keyboard_listener() {
  echo "监听键盘输入：按 'q' 退出程序，上/下方向键控制 Python 脚本"
  while true; do
    read -rsn1 key  # 读取单个按键输入
    if [[ $key == $'\e' ]]; then
      read -rsn2 -t 0.1 key2  # 读取方向键后续字符
      case $key2 in
        '[A') 
          echo "检测到 ↑ 上方向键，停止 Python 脚本"
          stop_task
          ;;
        '[B') 
          echo "检测到 ↓ 下方向键，启动 Python 脚本"
          main_task
          ;;
      esac
    elif [[ $key == 'q' ]]; then
      echo "检测到 'q' 键，退出程序..."
      stop_task  # 确保退出前停止主任务
      exit 0
    fi
  done
}

# 启动主任务
main_task

# 启动键盘监听器
keyboard_listener
