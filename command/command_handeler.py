"""Serial communication bridge between Python and ESP32 arm firmware."""

import time
import serial


SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
SERIAL_TIMEOUT = 1
SERIAL_INIT_DELAY = 2
RESPONSE_DELAY = 0.1

_ser = None


def _get_serial():
    """Lazy-initialize and return the serial connection."""
    global _ser
    if _ser is None:
        try:
            _ser = serial.Serial(SERIAL_PORT, BAUD_RATE,
                                 timeout=SERIAL_TIMEOUT)
            time.sleep(SERIAL_INIT_DELAY)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            _ser = None
    return _ser


def send_joint_data(t1, t2, t3, t4, t5, t6):
    """Send formatted joint angles to ESP32."""
    ser = _get_serial()
    if ser is None:
        return
    data = f"b{t1}r{t2}l{t3}t{t4}w{t5}f{t6}\n"
    print(f"[TX] {data.strip()}")
    ser.write(data.encode("utf-8"))
    read_response()


def send_command(cmd):
    """Send a text command string to the ESP32."""
    ser = _get_serial()
    if ser is None:
        return
    ser.write((cmd + "\n").encode("utf-8"))
    print(f"[TX] {cmd}")
    read_response()


def read_response():
    """Read and print all pending ESP32 response lines."""
    ser = _get_serial()
    if ser is None:
        return
    time.sleep(RESPONSE_DELAY)
    while ser.in_waiting:
        response = ser.readline().decode("utf-8", errors="ignore").strip()
        if response:
            print(f"[RX] {response}")
