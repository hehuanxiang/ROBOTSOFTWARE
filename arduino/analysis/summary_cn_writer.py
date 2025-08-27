def write_summary_cn(date_str, actual, expected, missing, percentage):
    summary = f"📅 日期：{date_str}\n"
    summary += f"✅ 实际采集轮数：{len(actual)}\n"
    summary += f"📈 理论应采集轮数：{len(expected)}\n"
    summary += f"📊 完成率：{percentage:.2f}%\n"
    summary += f"❌ 缺失轮数：{len(missing)}\n"
    if missing:
        summary += "⏱️ 缺失时间段：\n" + ", ".join(missing) + "\n"

    print(summary)

    path = f"/home/pi/Desktop/ROBOTSOFTWARE/arduino/summary_cn_{date_str}.log"
    with open(path, "w", encoding="utf-8") as f:
        f.write(summary)

