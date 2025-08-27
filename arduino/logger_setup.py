# logger_setup.py
import logging
import os
import re
from logging.handlers import TimedRotatingFileHandler

class RenameSuffixTimedRotatingFileHandler(TimedRotatingFileHandler):
    def doRollover(self):
        super().doRollover()
        dir_name = os.path.dirname(self.baseFilename)
        for filename in os.listdir(dir_name):
            if filename.startswith("sowbot.log.") and re.match(r"sowbot\.log\.\d{4}-\d{2}-\d{2}.log", filename):
                src = os.path.join(dir_name, filename)
                dst = os.path.join(dir_name, filename.replace("sowbot.log.", "sowbot_"))
                if not os.path.exists(dst):
                    os.rename(src, dst)

def get_file_logger():
    logger = logging.getLogger("Sowbot.MainLogger")
    logger.setLevel(logging.DEBUG)
    logger.propagate = False

    if not logger.handlers:
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
        handler.extMatch = re.compile(r"^\d{4}-\d{2}-\d{2}.log$")
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)

        logger.addHandler(handler)

    return logger
