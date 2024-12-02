import time
import os

def get_cpu_temp():
    # 获取CPU温度
    result = os.popen("vcgencmd measure_temp").readline()
    return result.replace("temp=", "").strip()

def log_temperature(interval, log_file):
    with open(log_file, "a") as file:
        while True:
            temp = get_cpu_temp()
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            file.write(f"{timestamp} - CPU Temperature: {temp}\n")
            print(f"{timestamp} - CPU Temperature: {temp}")
            time.sleep(interval)

if __name__ == "__main__":
    # 设置记录间隔（秒）和日志文件路径
    log_interval = 120  # 每隔60秒记录一次
    log_file_path = "/home/pi/Desktop/ROBOTSOFTWARE/test/cpu_temp_log.txt"
    log_temperature(log_interval, log_file_path)
