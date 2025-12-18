import serial
import time
import speech_recognition as sr
import noisereduce as nr
import numpy as np

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


def listen_and_recognize():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Record .")
        audio = recognizer.listen(source)
        # Convert audio to numpy array for processing
        audio_data = np.frombuffer(audio.get_raw_data(), np.int16)
        # Reduce noise
        reduced_noise = nr.reduce_noise(y=audio_data, sr=16000)
        # Convert back to AudioData for recognition
        audio = sr.AudioData(reduced_noise.tobytes(),
                             audio.sample_rate, audio.sample_width)
    try:
        text = recognizer.recognize_google(audio)
        return text
    except sr.UnknownValueError:
        return None
    except sr.RequestError:
        return None


def extract_command(text):
    text = text.lower()
    actions = ['take', 'pick', 'grab', 'get']
    action = None
    for act in actions:
        if act in text:
            action = act
            break
    if action:
        # Remove polite words and the action
        for prefix in ['please', 'could you', 'would you']:
            text = text.replace(prefix, '')
        # Remove action
        text = text.replace(action, '', 1)
        object_ = text.strip()
        print(f'action: {action}')
        print(f'object: {object_}')
        return action
    else:
        print("No action found in the command.")
        return None


def send_joint_data(t1, t2, t3, t4, t5, t6):
    """
    Send formatted joint data to the Arduino and read response.
    base, right, left, twist, wrist, finger, gripper
    """
    data = f"b{t1}r{t2}l{t3}t{t4}w{t5}f{t6}\n"
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


def take_obj1():
    # rotate :
    send_joint_data(180, 100, 20, 95, 150, 50)
    time.sleep(2)
    # ground
    send_joint_data(180, 36, 90, 95, 80, 50)
    time.sleep(2)
    # grab
    send_joint_data(180, 36, 90, 95, 80, 145)
    time.sleep(2)
    # up
    send_joint_data(180, 100, 20, 95, 150, 145)
    time.sleep(2)
    # to box
    send_joint_data(0, 100, 20, 95, 150, 145)
    time.sleep(2)
    # place
    send_joint_data(0, 100, 20, 95, 150, 50)
    time.sleep(2)
    send_joint_data(90, 100, 20, 95, 150, 150)


def take_obj2():
    # rotate :
    send_joint_data(120, 100, 20, 95, 150, 50)
    time.sleep(2)
    # ground
    send_joint_data(120, 36, 90, 95, 80, 50)
    time.sleep(2)
    # grab
    send_joint_data(120, 36, 90, 95, 80, 145)
    time.sleep(2)
    # up
    send_joint_data(120, 100, 20, 95, 150, 145)
    time.sleep(2)
    # to box
    send_joint_data(0, 100, 20, 95, 150, 145)
    time.sleep(2)
    # place
    send_joint_data(0, 100, 20, 95, 150, 50)
    time.sleep(2)
    send_joint_data(90, 100, 20, 95, 150, 150)


print("Please give me the command: ")

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
            if user_input == 's':
                send_joint_data(90, 100, 20, 95, 150, 150)
            elif user_input == 'o':
                take_obj1()
            elif user_input == 'i':
                take_obj2()

            else:
                print("Empty input. Please enter a command or 6 angles.")
                send_command(user_input)

    except KeyboardInterrupt:
        print("\nInterrupted. Exiting...")
        break
