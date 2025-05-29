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


# Keep script alive (optional)
while True:
    try:
        user_input = input(
            "Enter 7 values separated by spaces (t1 t2 t3 t4 t5 t6 t7), or 'q' to quit: ")
        if user_input.lower() == 'q':
            print("Exiting...")
            break

        parts = user_input.strip().split()
        if len(parts) != 7:
            print("Error: Please enter exactly 7 values.")
            continue

        # Convert all parts to integers
        t1, t2, t3, t4, t5, t6, t7 = map(int, parts)
        send_joint_data(t1, t2, t3, t4, t5, t6, t7)

    except ValueError:
        print("Error: Make sure all inputs are integers.")
    except KeyboardInterrupt:
        print("\nInterrupted by user. Exiting...")
        break

# Close serial port when done
ser.close()
