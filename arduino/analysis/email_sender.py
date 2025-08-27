import smtplib
from email.mime.text import MIMEText
from email.header import Header
import os
from datetime import datetime

mail_host = 'smtp.gmail.com'
mail_user = 'hxh0326@gmail.com'
mail_pass = 'pnkvseixcwzvlqlu'
sender = mail_user
receivers = ['xhh2c@umsystem.edu']  # ✅ 修改为你的实际收件人

def send_summary_email(date_str, actual, expected, missing, percentage, reboot_info):
    subject = f"[Sowbot Daily] {date_str} Data Summary"

    content = f"""📅 Date: {date_str}
✅ Completed cycles: {len(actual)}
📈 Expected cycles: {len(expected)}
📊 Completion rate: {percentage:.2f}%
❌ Missing cycles: {len(missing)}

"""

    if missing:
        content += "⏱️ Missing time slots:\n" + ", ".join(missing[:50])
        if len(missing) > 50:
            content += f"... (+{len(missing)-50} more)\n"
        content += "\n"

    if reboot_info:
        content += "🔁 Reboot Records:\n"
        for i, record in enumerate(reboot_info, 1):
            content += f"--- Reboot #{i} ---\n"
            content += f"📌 Reason: {record.get('REBOOT_REASON', 'Unknown')}\n"
            content += f"📂 Type: {record.get('REBOOT_TYPE', 'Unknown')}\n"
            content += f"🕒 Shutdown Time: {record.get('SHUTDOWN_TIME', '-')}\n"
            content += f"🔁 Startup Time: {record.get('STARTUP_TIME', '-')}\n"
            content += f"⏱️ Interval: {record.get('SHUTDOWN_INTERVAL_MIN', '-')} min\n\n"
    else:
        content += "📮 No reboot records found for this date.\n"

    # 备份邮件内容
    backup_dir = "/home/pi/Desktop/ROBOTSOFTWARE/arduino/analysis/mail_backup"
    os.makedirs(backup_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    backup_path = os.path.join(backup_dir, f"summary_email_{date_str}_{timestamp}.txt")
    with open(backup_path, "w", encoding="utf-8") as f:
        f.write(content)

    # 构造邮件
    message = MIMEText(content, 'plain', 'utf-8')
    message['From'] = Header("Sowbot Report", 'utf-8')
    message['To'] = Header("Team", 'utf-8')
    message['Subject'] = Header(subject, 'utf-8')

    try:
        smtp_obj = smtplib.SMTP(mail_host, 587)
        smtp_obj.starttls()
        smtp_obj.login(mail_user, mail_pass)
        smtp_obj.sendmail(sender, receivers, message.as_string())
        print(f"✅ 邮件发送成功，并已保存本地备份：{backup_path}")
    except Exception as e:
        print(f"❌ 邮件发送失败: {e}")
