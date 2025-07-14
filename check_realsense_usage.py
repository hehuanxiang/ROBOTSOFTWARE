import subprocess
import psutil

def list_video_devices():
    try:
        result = subprocess.run(['ls', '/dev/video*'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        return result.stdout.strip().split('\n')
    except Exception as e:
        return []

def find_process_using_video_device():
    video_devices = list_video_devices()
    matches = []

    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            for dev in video_devices:
                if dev in ' '.join(proc.info['cmdline']):
                    matches.append((proc.info['pid'], proc.info['name'], proc.info['cmdline']))
                    break
        except Exception:
            continue

    return matches

if __name__ == "__main__":
    print("🔍 Checking for processes using RealSense video devices...\n")
    processes = find_process_using_video_device()
    if processes:
        for pid, name, cmd in processes:
            print(f"❗ PID: {pid}, Name: {name}\n   CMD: {' '.join(cmd)}\n")
        print("✅ You may need to kill these processes before using RealSense.\nUse `kill -9 <PID>` to stop them.")
    else:
        print("✅ No processes found using RealSense video devices.")
