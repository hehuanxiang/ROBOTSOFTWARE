#!/usr/bin/python3
# -*- coding: utf-8 -*-
import smtplib
from email.mime.text import MIMEText
from email.utils import formataddr
import socket
import subprocess

# 配置邮箱信息
mail_host = 'smtp.gmail.com.'   # QQ 企业邮箱 SMTP 地址
mail_user = 'hxh0326@gmail.com'     # 邮箱账号
mail_pass = 'pnkvseixcwzvlqlu'   # 授权码，不是密码
mail_postfix = 'gmail.com'             # 邮箱域名

def get_ip_addresses():
    """获取树莓派的本地和 ZeroTier IP 地址"""
    local_ip = subprocess.check_output("hostname -I | awk '{print $1}'", shell=True).decode().strip()
    
    # 这里假设 ZeroTier IP 是在特定的网络接口上，可以通过命令获取
    try:
        zt_ip = subprocess.check_output("hostname -I | awk '{print $2}'", shell=True).decode().strip()
    except:
        zt_ip = "未能获取 ZeroTier IP 地址"
    
    return local_ip, zt_ip

def send_mail(to_list, subject, content):
    me = f"{mail_user}@{mail_postfix}"
    msg = MIMEText(content, 'plain', 'utf-8')
    msg['From'] = formataddr(["树莓派通知", me])
    msg['To'] = to_list
    msg['Subject'] = subject

    try:
        s = smtplib.SMTP_SSL(mail_host, 465)
        s.login(mail_user, mail_pass)
        s.sendmail(me, [to_list], msg.as_string())
        s.quit()
        print("邮件发送成功")
        return True
    except Exception as e:
        print("邮件发送失败:", str(e))
        return False

if __name__ == "__main__":
    # 设置收件人和邮件主题
    recipient = "recipient_email@example.com"
    subject = "树莓派 IP 地址通知"

    # 获取 IP 地址信息
    local_ip, zt_ip = get_ip_addresses()
    content = f"树莓派已成功连接网络。\n本地 IP: {local_ip}\nZeroTier IP: {zt_ip}"

    # 发送邮件
    send_mail(recipient, subject, content)
