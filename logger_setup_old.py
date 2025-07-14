import logging
import os
from logging.handlers import TimedRotatingFileHandler

def setup_logger():
    logger = logging.getLogger("Sowbot")  # 所有模块共用同一个 logger
    logger.propagate = False  # 防止重复打印

    if not logger.handlers:
        log_dir = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log"
        os.makedirs(log_dir, exist_ok=True)

        base_log_file = os.path.join(log_dir, "sowbot.log")  # 基础文件名（系统自动添加日期）

        handler = TimedRotatingFileHandler(
            filename=base_log_file,
            when="midnight",
            interval=1,
            backupCount=0,           # 不删除旧文件
            encoding="utf-8",
            utc=False
        )

        # 设置文件名后缀格式，确保文件名为 sowbot_2025-07-10.log
        handler.suffix = "sowbot_%Y-%m-%d.log"

        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)

        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

    return logger
