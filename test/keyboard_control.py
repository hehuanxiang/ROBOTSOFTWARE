from evdev import InputDevice, categorize, ecodes

# 替换为你的设备路径
device = InputDevice('/dev/input/event1')

print("监听方向键：按 ESC 键退出程序")

# 读取事件
for event in device.read_loop():
    if event.type == ecodes.EV_KEY:  # 只处理按键事件
        key_event = categorize(event)
        if key_event.keystate == 1:  # 按键按下事件
            if key_event.keycode == 'KEY_UP':
                print("检测到 ↑ 上方向键")
            elif key_event.keycode == 'KEY_DOWN':
                print("检测到 ↓ 下方向键")
            elif key_event.keycode == 'KEY_LEFT':
                print("检测到 ← 左方向键")
            elif key_event.keycode == 'KEY_RIGHT':
                print("检测到 → 右方向键")
            elif key_event.keycode == 'KEY_ESC':
                print("检测到 ESC 键，退出程序")
                break
