#!/bin/bash

LOG_DIR="/home/pi/Desktop/ROBOTSOFTWARE/arduino/system_log"
mkdir -p "$LOG_DIR"

DATE=$(date "+%Y-%m-%d")
TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
LOG_FILE="$LOG_DIR/reboot_reason_${DATE}.log"

# # 检查当前 boot ID 和上一轮 boot ID 是否一致（避免误判）
# CURRENT_BOOT_ID=$(journalctl -b --no-pager --output=short | head -n 1 | cut -d' ' -f1)
# PREV_BOOT_ID=$(journalctl -b -1 --no-pager --output=short 2>/dev/null | head -n 1 | cut -d' ' -f1)

# if [ "$CURRENT_BOOT_ID" = "$PREV_BOOT_ID" ]; then
#     echo "[$TIMESTAMP] 🚫 当前并非刚刚重启，跳过本次分析。" >> "$LOG_FILE"
#     echo "" >> "$LOG_FILE"
#     exit 0
# fi

echo "[$TIMESTAMP] ---- Reboot Analysis Start ----" >> "$LOG_FILE"

# 检查是否启用持久化日志
if [ ! -d "/var/log/journal" ]; then
    echo "[$TIMESTAMP] ⚠️ 未启用持久化日志 (/var/log/journal 不存在)。建议配置 Storage=persistent。" >> "$LOG_FILE"
    exit 0
fi

# 检查上一轮日志是否存在
if ! journalctl -b -1 &>/dev/null; then
    echo "[$TIMESTAMP] ⚠️ 无法读取上一轮启动日志。" >> "$LOG_FILE"
    echo "[$TIMESTAMP] 结论：可能是首次启动、日志未保存或断电导致。" >> "$LOG_FILE"
    echo "[$TIMESTAMP] 📋 小结：首次启动或日志缺失" >> "$LOG_FILE"
    echo "" >> "$LOG_FILE"
    exit 0
fi

# 初始化标记
NORMAL_SHUTDOWN=0
HAS_CRASH=0
HAS_FSCK=0
LAST_REASON="未知异常重启"

# 获取时间
LAST_BOOT_TIME=$(journalctl -b -1 --no-pager --output=short-unix | head -n 1 | awk '{print $1}')
CURRENT_BOOT_TIME=$(journalctl -b 0 --no-pager --output=short-unix | head -n 1 | awk '{print $1}')
DIFF_MIN=$(awk -v a=$CURRENT_BOOT_TIME -v b=$LAST_BOOT_TIME 'BEGIN {print int((a - b) / 60)}')
SHUTDOWN_TIME_STR=$(date -d @$LAST_BOOT_TIME "+%Y-%m-%d %H:%M:%S")
STARTUP_TIME_STR=$(date -d @$CURRENT_BOOT_TIME "+%Y-%m-%d %H:%M:%S")

# 1. 正常关机
if journalctl -b -1 | grep -qE "systemd-shutdown"; then
    NORMAL_SHUTDOWN=1
    LAST_REASON="正常关机"
    echo "[$TIMESTAMP] ✅ 检测到正常的 systemd 关机/重启流程。" >> "$LOG_FILE"
    echo "[$TIMESTAMP] 结论：上次关机正常。" >> "$LOG_FILE"
    echo "[$TIMESTAMP] ⏱️ 关机时间：$SHUTDOWN_TIME_STR，开机时间：$STARTUP_TIME_STR，间隔 ${DIFF_MIN} 分钟。" >> "$LOG_FILE"
    echo "[$TIMESTAMP] 📋 小结：正常关机" >> "$LOG_FILE"

else
    echo "[$TIMESTAMP] ❗ 未检测到 systemd 正常关机流程。" >> "$LOG_FILE"

    # 2.1 系统崩溃
    CRASH_LOG=$(journalctl -b -1 -p 0..3 | grep -Ei "kernel panic|BUG|segfault|oom-killer|watchdog|fatal" | grep -vE "bcm2835|mmc")
    if [ -n "$CRASH_LOG" ]; then
        HAS_CRASH=1
        LAST_REASON="异常关机（系统崩溃）"
        echo "[$TIMESTAMP] ⚠️ 检测到严重等级的系统崩溃日志：" >> "$LOG_FILE"
        echo "$CRASH_LOG" | head -n 5 >> "$LOG_FILE"
        echo "[$TIMESTAMP] 结论：上次关机为系统崩溃导致。" >> "$LOG_FILE"
    fi
    # 2.2 断电信号
    FSCK_LOG=$(journalctl -b | grep -i "dirty bit is set")
    if [ -n "$FSCK_LOG" ]; then
        HAS_FSCK=1
        if [ "$HAS_CRASH" -eq 0 ]; then
            LAST_REASON="异常关机（断电）"
        fi
        echo "[$TIMESTAMP] ⚠️ 当前启动检测到文件系统未正常卸载：" >> "$LOG_FILE"
        echo "$FSCK_LOG" | head -n 2 >> "$LOG_FILE"
        echo "[$TIMESTAMP] 结论：上次关机未正常卸载，可能为断电。" >> "$LOG_FILE"
    fi

    echo "[$TIMESTAMP] ⏱️ 关机时间：$SHUTDOWN_TIME_STR，开机时间：$STARTUP_TIME_STR，间隔 ${DIFF_MIN} 分钟。" >> "$LOG_FILE"
fi

echo "[$TIMESTAMP] 📋 小结：$LAST_REASON" >> "$LOG_FILE"
echo "" >> "$LOG_FILE"

# ==== 写入 summary 文件 ====

REBOOT_REASON_EN="Unknown reboot"
REBOOT_TYPE="Unknown"

if [ "$LAST_REASON" = "正常关机" ]; then
    REBOOT_REASON_EN="Normal shutdown"
    if [ "$DIFF_MIN" -le 3 ]; then
        REBOOT_TYPE="Reboot"
    else
        REBOOT_TYPE="Shutdown"
    fi
elif [ "$LAST_REASON" = "异常关机（系统崩溃）" ]; then
    REBOOT_REASON_EN="System crash"
    REBOOT_TYPE="PowerLoss"
elif [ "$LAST_REASON" = "异常关机（断电）" ]; then
    REBOOT_REASON_EN="Power loss"
    REBOOT_TYPE="PowerLoss"
fi

SUMMARY_FILE="$LOG_DIR/reboot_summary_all.txt"
LOG_WRITE_TIME=$(date "+%Y-%m-%d %H:%M:%S")
RECORD_DATE=$(date "+%Y-%m-%d")

{
    echo "RECORD_DATE=$RECORD_DATE"
    echo "LOG_WRITE_TIME=$LOG_WRITE_TIME"
    echo "REBOOT_REASON=$REBOOT_REASON_EN"
    echo "REBOOT_TYPE=$REBOOT_TYPE"
    echo "SHUTDOWN_TIME=$SHUTDOWN_TIME_STR"
    echo "STARTUP_TIME=$STARTUP_TIME_STR"
    echo "SHUTDOWN_INTERVAL_MIN=$DIFF_MIN"
    echo ""
} >> "$SUMMARY_FILE"
