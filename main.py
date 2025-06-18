import serial
import time

# Adjust for your system
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Wait for Arduino to reset
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit(1)


def send_joint_data(t1, t2, t3, t4, t5, t6, t7=50):
    """
    Send formatted joint data to the Arduino and read response.
    """
    data = f"s{t1}a{t2}b{t3}c{t4}d{t5}e{t6}f{t7}\n"
    print(f"[TX] {data.strip()}")
    ser.write(data.encode('utf-8'))
    read_response()


def send_command(cmd):
    """
    Send command string to Arduino and read response.
    """
    ser.write((cmd + '\n').encode('utf-8'))
    print(f"[TX] {cmd}")
    read_response()


def read_response():
    """
    Read and print the Arduino response (if any).
    """
    time.sleep(0.1)  # Give Arduino a little time to respond
    while ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"[RX] {response}")


print("Ready. Type 6 joint angles, or a command (e.g. 'init', 'status'). Type 'q' to quit.")

while True:
    try:
        user_input = input(">>> ").strip()

        if user_input.lower() in ('q', 'quit', 'exit'):
            print("Exiting...")
            break

        parts = user_input.split()

        if len(parts) == 6:
            try:
                angles = list(map(int, parts))
                send_joint_data(*angles)
            except ValueError:
                print("Error: Please enter 6 valid integers.")
        else:
            if user_input:
                send_command(user_input)
            else:
                print("Empty input. Please enter a command or 6 angles.")

    except KeyboardInterrupt:
        print("\nInterrupted. Exiting...")
        break
