import serial
import time

# Change this to your actual port (e.g., "COM10" on Windows or "/dev/ttyUSB0" on Linux)
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Wait for the Arduino to reset


def send_joint_data(t1, t2, t3, t4, t5, t6, t7):
    """
    Send formatted joint data to the Arduino
    t1 to t6: Joint positions
    t7: Motion duration (tf)
    """
    # Format: s<t1>a<t2>b<t3>c<t4>d<t5>e<t6>f<t7>\n
    data = f"s{t1}a{t2}b{t3}c{t4}d{t5}e{t6}f{t7}\n"
    print(f"Sending: {data.strip()}")
    ser.write(data.encode('utf-8'))


# Example usage:
send_joint_data(0, 10, 5, 0, 30, 20, 150)  # Replace values as needed

# Keep script alive (optional)
while True:
    # Could implement live input or timed sequences here
    time.sleep(0.1)
