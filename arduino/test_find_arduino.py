import serial
import serial.tools.list_ports

def find_arduino_port():
    """
    在只有 Arduino + RealSense 的情况下，
    直接返回第一个 /dev/ttyACM* 串口即可。
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "ttyACM" in port.device:
            print(f"🔍 Found Arduino port: {port.device}")
            return port.device
    return None

try:
    arduino_port = find_arduino_port()
    if arduino_port:
        ser = serial.Serial(arduino_port, 9600, timeout=1)
        ser.write(b'FIND\n')
        ser.close()
        # motor_logger.info(f"🔍 FIND command sent to Arduino on {arduino_port}.")
    else:
        print("⚠️ No /dev/ttyACM* device found.")
except Exception as e:
    print(f"⚠️ Failed to send FIND command: {e}")
