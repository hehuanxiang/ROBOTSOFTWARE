#!/bin/bash

# 定义变量
SESSION_NAME="swine_lidar" # tmux 会话名称
DELAY=21600 # 延迟时间（秒），6 小时
PYTHON_SCRIPT="/home/pi/Desktop/ROBOTSOFTWARE/test/motor.py" # Python 脚本路径
PYTHON_ARGUMENT="back" # Python 脚本的参数

echo "Start to wait for $DELAY seconds to kill tmux session '$SESSION_NAME'."

# 定时计时器
# -gt bash中的运算符，表示greater than
while [ $DELAY -gt 0 ]; do
    # -n 表示输出取消换行，e用于解释转义字符\r
    # \r(回车符（将光标移动到行首）,覆盖同一行的输出内容)
    echo -ne "Time remaining: $DELAY seconds\r" # 实时显示剩余时间
    sleep 1 # 每秒钟更新一次
    ((DELAY--)) # 减少剩余时间
done
echo # 换行

# 检查 tmux 会话是否存在并关闭
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    tmux kill-session -t $SESSION_NAME
    echo "Tmux session '$SESSION_NAME' has been closed."
else
    echo "Tmux session '$SESSION_NAME' does not exist."
fi

# 启动新的 Python 脚本
echo "Starting Python script: $PYTHON_SCRIPT $PYTHON_ARGUMENT"
python $PYTHON_SCRIPT $PYTHON_ARGUMENT &

# 确认 Python 脚本是否启动
# $?是一个特殊变量，表示上一个命令的退出状态（Exit Code）
# -eq 比较运算符，表示equal to
if [ $? -eq 0 ]; then
    echo "Python script started successfully."
else
    echo "Failed to start Python script."
fi

# 输出脚本运行结束的时间，$(date)即可输出时间
# + 用于 指示格式化输出 的标志，告诉 date 需要按照后面提供的格式字符串（如 %Y-%m-%d %H:%M:%S）来显示时间。
echo "Script finished at: $(date '+%Y-%m-%d %H:%M:%S')"