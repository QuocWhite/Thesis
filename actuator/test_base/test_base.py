import serial
import time

# Change to your ESP32 COM port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # Linux: '/dev/ttyUSB0'
print("Connected to ESP32")

for i in range(160):
    ser.write(f"{i}\n".encode())

while True:
    for i in range(160):
        ser.write(f"{i}\n".encode())
        time.sleep(1)
