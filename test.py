from evdev import InputDevice, categorize, ecodes

# 获取键盘设备（可通过 `ls /dev/input` 找到设备路径）
device = InputDevice('/dev/input/by-id')

print("监听全局键盘事件，按 ESC 退出程序")

for event in device.read_loop():
    if event.type == ecodes.EV_KEY:
        key_event = categorize(event)
        if key_event.keystate == 1:  # 按键按下事件
            if key_event.keycode == 'KEY_UP':
                print("检测到 ↑ 上方向键")
            elif key_event.keycode == 'KEY_DOWN':
                print("检测到 ↓ 下方向键")
            elif key_event.keycode == 'KEY_ESC':
                print("检测到 ESC 键，退出程序")
                break
