# logger_setup.py
import logging
import os
import re
from logging.handlers import TimedRotatingFileHandler

class RenameSuffixTimedRotatingFileHandler(TimedRotatingFileHandler):
    def doRollover(self):
        super().doRollover()

        # 自动重命名为 sowbot_YYYY-MM-DD.log
        dir_name = os.path.dirname(self.baseFilename)
        for filename in os.listdir(dir_name):
            if filename.startswith("sowbot.log.") and re.match(r"sowbot\.log\.\d{4}-\d{2}-\d{2}.log", filename):
                src = os.path.join(dir_name, filename)
                dst = os.path.join(dir_name, filename.replace("sowbot.log.", "sowbot_"))
                if not os.path.exists(dst):
                    os.rename(src, dst)

# 全局 handler
_shared_handler = None

def setup_logger(submodule_name):
    global _shared_handler

    logger = logging.getLogger(f"Sowbot.{submodule_name}")
    logger.propagate = False

    if not logger.handlers:
        if _shared_handler is None:
            log_dir = "/home/pi/Desktop/ROBOTSOFTWARE/robot_log"
            os.makedirs(log_dir, exist_ok=True)

            log_path = os.path.join(log_dir, "sowbot.log")
            handler = RenameSuffixTimedRotatingFileHandler(
                filename=log_path,
                when="midnight",
                interval=1,
                backupCount=0,
                encoding="utf-8",
                utc=False
            )
            handler.suffix = "%Y-%m-%d.log"
            handler.extMatch = re.compile(r"^\d{4}-\d{2}-\d{2}.log$")  # 允许归档后缀
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)

            _shared_handler = handler

        logger.addHandler(_shared_handler)
        logger.setLevel(logging.DEBUG)

    return logger
