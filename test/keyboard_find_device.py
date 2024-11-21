import os
from evdev import InputDevice, categorize, ecodes, list_devices

# 列出所有输入设备
def find_devices():
    devices = [InputDevice(path) for path in list_devices()]
    print("可用的输入设备：")
    for device in devices:
        print(f"{device.path}: {device.name}")
    return devices

# 全局监听按键事件
def monitor_events(devices):
    print("\n监听全局键盘事件，按 Ctrl+C 退出")
    try:
        while True:
            # 遍历所有设备，监听按键事件
            for device in devices:
                try:
                    for event in device.read_loop():
                        if event.type == ecodes.EV_KEY:  # 按键事件
                            key_event = categorize(event)
                            if key_event.keystate == 1:  # 按键按下
                                if key_event.keycode == 'KEY_UP':
                                    print("检测到 ↑ 上方向键")
                                elif key_event.keycode == 'KEY_DOWN':
                                    print("检测到 ↓ 下方向键")
                                elif key_event.keycode == 'KEY_LEFT':
                                    print("检测到 ← 左方向键")
                                elif key_event.keycode == 'KEY_RIGHT':
                                    print("检测到 → 右方向键")
                                elif key_event.keycode == 'KEY_ESC':
                                    print("检测到 ESC 键，退出")
                                    return
                except BlockingIOError:
                    pass  # 跳过无事件的设备
    except KeyboardInterrupt:
        print("\n退出监听程序")

# 找到所有设备并开始监听
devices = find_devices()
monitor_events(devices)
