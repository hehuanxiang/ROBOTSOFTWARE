# sowbot.service
```bash
sudo nano /etc/systemd/system/sowbot.service
```

```ini
[Unit]
Description=Run sowbot_serial.py after 30s delay on boot
After=network-online.target time-sync.target    # 等待时间同步
Wants=network-online.target time-sync.target

[Service]
User=pi
Type=oneshot
ExecStart=/usr/bin/python /home/pi/Desktop/ROBOTSOFTWARE/arduino/sowbot_serial.py
WorkingDirectory=/home/pi/Desktop/ROBOTSOFTWARE/arduino
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target

```



# 重新加载 systemd 配置
sudo systemctl daemon-reload

# 设置开机自动启动
sudo systemctl enable sowbot.service

# 测试立即启动一次
sudo systemctl start sowbot.service

# 查看运行状态
systemctl status sowbot.service

# 查看日志
journalctl -u sowbot.service


# ✅ 启用 systemd 持久化日志功能（适用于 Raspberry Pi 或其他 Linux）

# 1. 创建持久化日志目录
```bash
sudo mkdir -p /var/log/journal
```

# 2. 设置正确的权限与结构
```bash
sudo systemd-tmpfiles --create --prefix /var/log/journal
```

# 3. 重启 systemd-journald 服务
```bash
sudo systemctl restart systemd-journald
```

# 4. 检查配置文件中 Storage 设置（默认 auto 即可）
```bash
cat /etc/systemd/journald.conf | grep Storage
```
应输出： 
```bash
#Storage=auto
```

如果你想手动设置为 persistent：
```bash
sudo nano /etc/systemd/journald.conf
```
修改为：
```ini
Storage=persistent
```
保存后重启服务：
```bash
sudo systemctl restart systemd-journald
```

# 5. 验证是否启用持久日志
```bash
journalctl --list-boots
```
# 应该看到多条启动记录，标记为 boot ID（如 -1、0）

# 6. 在下一次重启后查看上次启动前的日志
```bash
journalctl -b -1
```
如果启用成功，日志将保留在 /var/log/journal 中，即使断电也不会丢失。

