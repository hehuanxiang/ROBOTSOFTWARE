import os
from datetime import datetime, timedelta

DAILY_LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/arduino/analysis/report_log/daily_summary.log"
REBOOT_LOG_FILE = "/home/pi/Desktop/ROBOTSOFTWARE/arduino/system_log/reboot_summary_all.txt"

def load_log_stats(date_str):
    actual = []
    if os.path.exists(DAILY_LOG_FILE):
        with open(DAILY_LOG_FILE, 'r') as f:
            for line in f:
                if date_str in line:
                    try:
                        time_str = line.split(",")[0]
                        actual.append(datetime.strptime(time_str, "%Y-%m-%d %H:%M:%S"))
                    except:
                        continue

    expected = [datetime.strptime(date_str, "%Y-%m-%d") + timedelta(minutes=5 * i) for i in range(288)]
    actual_set = set(t.strftime("%H:%M") for t in actual)
    missing = [t.strftime("%H:%M") for t in expected if t.strftime("%H:%M") not in actual_set]

    actual_count = len(actual)
    expected_count = len(expected)
    percentage = (actual_count / expected_count) * 100 if expected_count else 0.0

    return actual, expected, missing, percentage

def load_reboot_summary(date_str):
    """返回该日期所有重启记录（列表），若无则返回 None"""
    if not os.path.exists(REBOOT_LOG_FILE):
        return None

    records = []
    with open(REBOOT_LOG_FILE, 'r') as f:
        lines = f.readlines()

    block = []
    for line in lines:
        line = line.strip()
        if line.startswith("RECORD_DATE="):
            if block:
                # 判断上一块是否为目标日期
                for bline in block:
                    if bline.startswith("RECORD_DATE=") and date_str in bline:
                        record = {}
                        for b in block:
                            if "=" in b:
                                key, value = b.split("=", 1)
                                record[key] = value
                        records.append(record)
                        break
            block = [line]
        elif line == "":
            if block:
                for bline in block:
                    if bline.startswith("RECORD_DATE=") and date_str in bline:
                        record = {}
                        for b in block:
                            if "=" in b:
                                key, value = b.split("=", 1)
                                record[key] = value
                        records.append(record)
                        break
            block = []
        else:
            block.append(line)

    # 最后一组也处理一下
    if block:
        for bline in block:
            if bline.startswith("RECORD_DATE=") and date_str in bline:
                record = {}
                for b in block:
                    if "=" in b:
                        key, value = b.split("=", 1)
                        record[key] = value
                records.append(record)
                break

    return records if records else None


