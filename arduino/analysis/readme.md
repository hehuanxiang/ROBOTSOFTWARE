# 📦 Sowbot 数据采集日报系统

该目录下的脚本用于每天生成猪只采集轮次统计、中文摘要报告，以及英文邮件发送汇报。

## 📁 文件说明

| 文件名 | 功能 |
|--------|------|
| `daily_report.py` | 主控入口脚本，调用各模块 |
| `log_reader.py` | 读取 daily_summary.log 并提取采集时间戳 |
| `summary_cn_writer.py` | 写入每日中文汇报至 arduino 文件夹 |
| `email_sender.py` | 构建英文邮件并通过 Gmail SMTP 发送 |
| `README.md` | 当前说明文档 |

## 📌 每日运行流程

1. motor 模块在每轮采集完成后追加：
   ```
   YYYY-MM-DD HH:MM:SS,cycle=XX
   ```
   至 `/record/daily_summary.log`

2. 每天晚上 23:59，`daily_report.py` 通过 crontab 自动执行：

   ```bash
   59 23 * * * /usr/bin/python3 /home/pi/Desktop/ROBOTSOFTWARE/analysis/daily_report.py
   ```

3. 自动完成：
   - 中文汇报写入：`arduino/summary_cn_YYYY-MM-DD.log`
   - 英文邮件发送至：`receivers` 列表配置

## 🚧 未来功能预留模块（你可继续添加）

- `image_classifier.py` → 图像分类汇总并发送统计
- `estrus_predictor.py` → 发情预测结果输出及发送
- `attachments.py` → 邮件中嵌入示例图片或曲线图


