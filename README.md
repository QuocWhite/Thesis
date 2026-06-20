# Quoc — Voice-Guided 6-DOF Robotic Arm Control System

Python-based control system for a **6-DOF pick-and-place robot arm** with voice commands, computer vision (YOLOv3), inverse kinematics, and Arduino Mega firmware.

## System Overview

```
User Voice → Speech Recognition → Command Parsing → Object Detection (YOLO)
                                                       ↓
                                              Inverse Kinematics
                                                       ↓
                                           Serial → Arduino Mega
                                                       ↓
                                           PCA9685 → 6 Servo Motors
```

## 6-DOF Joint Configuration (Denavit-Hartenberg)

| Joint | Name       | Type     | DH a (mm) | DH d (mm) | α       | Serial Label |
|-------|------------|----------|-----------|-----------|---------|--------------|
| J1    | Base       | Waist    | 42        | 75        | +90°    | `b`          |
| J2    | Right      | Shoulder | 95        | 0         | 0°      | `r`          |
| J3    | Left       | Elbow    | 18        | 0         | +90°    | `l`          |
| J4    | Twist      | Wrist roll | 0       | 166       | -90°    | `t`          |
| J5    | Wrist      | Wrist pitch | 0      | 0         | +90°    | `w`          |
| J6    | Finger     | Gripper  | 0         | 27        | +180°   | `f`          |

## Project Structure

```
Quoc/
├── voice/                      Speech recognition (Google STT + noise reduction)
│   └── VoiceRecognization.py
├── vision/                     YOLOv3 object detection
│   ├── yolo.py                 2D detection → pixel coordinates
│   └── yolo3/                  YOLOv3 model architecture
├── kinematics/                 Forward & inverse kinematics
│   └── kinematics.py           DH parameters, Jacobian-based IK solver
├── actuator/                   Arduino Mega firmware & 3D print files
│   ├── ArmControl/             Firmware source (C++)
│   │   ├── ArmControl.ino      Entry point
│   │   ├── ArmControl.h        Configuration, pin assignments, class API
│   │   ├── ArmControl.cpp      6-axis motion control, cubic trajectory, serial parser
│   │   └── Test/               Host-side integration & unit tests
│   └── Finger/                 3D-printable gripper STL files
├── command/                    Serial communication bridge
│   └── command_handeler.py     Sends joint data & commands to Arduino
├── main.py                     Pipeline entry point
├── run_sys.sh                  Startup script (activates venv, runs main.py)
└── requirements.txt
```

## Firmware (Arduino Mega)

### Communication Protocol

Serial at **115200 baud** on `/dev/ttyUSB0` (configurable).

**Joint angle format:** `b<base>r<right>l<left>t<twist>w<wrist>f<finger>\n`

Example: `b90r45l60t0w30f50\n` sets all 6 joints.

**Text commands:**
| Command    | Action                           |
|------------|----------------------------------|
| `init`     | Reset all joints to factory home |
| `report`   | Print current joint positions     |
| `test`     | Run servo sweep test              |
| `mcalib`   | Read manual calibration data      |
| `acalib`   | Run auto calibration              |

### Motion Control

- **Servo driver:** PCA9685 16-channel PWM controller (I²C)
- **Cubic trajectory interpolation:** Smooth point-to-point motion with configurable duration (150 ms per segment)
- **Trajectory queue:** Circular buffer (16 waypoints) for multi-step sequences
- **Joint limits:** Configurable min/max pulse widths per joint via hardware trigger pins
- **EEPROM calibration:** Stores calibrated pulse ranges persistently

### Hardware Pin Mapping

Each joint has dedicated trigger pins for limit/calibration:

| Joint  | Min Pin | Max Pin | PCA9685 Channel |
|--------|---------|---------|-----------------|
| Base   | 12      | 5       | 0               |
| Right  | 14      | 13      | 1               |
| Left   | 19      | 18      | 2               |
| Twist  | 26      | 25      | 3               |
| Wrist  | 32      | 27      | 4               |
| Finger | 4       | 33      | 5               |

## Python Components

### Voice (`voice/VoiceRecognization.py`)
- Google Speech Recognition API
- Real-time noise reduction via `noisereduce`
- Keyword extraction for actions: *take, pick, grab, get*

### Vision (`vision/yolo.py`)
- YOLOv3 object detection (TensorFlow 2 / Keras)
- 2D pixel coordinates from webcam feed
- COCO dataset classes (80 object categories)

### Kinematics (`kinematics/kinematics.py`)
- Forward kinematics via DH parameter transformation matrices
- Inverse kinematics via Jacobian pseudoinverse (damped least-squares)
- Euler angle extraction from rotation matrices
- Configurable iteration count, tolerance, and learning rate

### Command Bridge (`command/command_handeler.py`)
- Serial communication with Arduino Mega
- `send_joint_data(t1..t6)` — sends formatted joint angles
- `send_command(cmd)` — sends text commands
- Response reading and logging

## Setup

```bash
pip install -r requirements.txt
./run_sys.sh
```

Requires **Python 3.10+** and an **Arduino Mega** running the firmware at `/dev/ttyUSB0` (configurable in `command/command_handeler.py`).

### Arduino Firmware Upload

1. Open `actuator/ArmControl/ArmControl.ino` in Arduino IDE
2. Install dependencies: `Adafruit PWMServoDriver` library
3. Select **Arduino Mega** board
4. Upload to `/dev/ttyUSB0`

## 3D-Printable Parts

Gripper STL files in `actuator/Finger/`:
- Palm (top & bottom)
- Finger pads & hinge arms
- Left/right gear arms
- Servo gear components

## Requirements

- Python 3.10+
- Arduino Mega
- PCA9685 servo driver
- 6× servo motors (with limit switches)
- USB webcam
- Microphone
- TB6600 stepper drivers (optional)
