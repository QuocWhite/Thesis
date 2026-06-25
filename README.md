# Quoc — Voice-Guided 6-DOF Robotic Arm Control System

Python-based control system for a **6-DOF pick-and-place robot arm** with voice commands, computer vision (YOLOv3), inverse kinematics, and ESP32 firmware (Arduino framework).

## System Overview

```
User Voice → Speech Recognition → Command Parsing → Object Detection (YOLO)
                                                       ↓
                                              Inverse Kinematics
                                                       ↓
                                            Serial → ESP32
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
├── actuator/                   ESP32 firmware (Arduino framework) & 3D print files
│   ├── ArmControl/             Firmware source (C++)
│   │   ├── ArmControl.ino      Entry point
│   │   ├── ArmControl.h        Configuration, pin assignments, class API
│   │   └── ArmControl.cpp      6-axis motion control, cubic trajectory, serial parser
│   └── Finger/                 3D-printable gripper STL files
├── command/                    Serial communication bridge
│   └── command_handeler.py     Sends joint data & commands to ESP32
├── Test/                       Host-side integration & unit tests
│   ├── test_system.py          Kinematics, serial bridge, voice parsing, pipeline, 3D viz
│   ├── test_armcontrol.py      Python port of firmware logic (18 unit/stress tests)
│   ├── armcontrol.py           Python port of firmware TrajectoryQueue
│   ├── Arm_IT_test.cpp         C++ host endurance test (10k steps)
│   ├── Arm                     Compiled C++ test binary
│   ├── arm_fk_plot.png         Generated 3D FK visualization
│   └── arm_ik_plot.png         Generated IK convergence plot
├── main.py                     Pipeline entry point
├── run_sys.sh                  Startup script (activates venv, runs main.py)
└── requirements.txt
```

## Firmware (ESP32)

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
- **Trajectory queue:** 2D circular buffer (`queueBuffer_[MAX_JOINT][TRAJECTORY_QUEUE_SIZE]`) for multi-step sequences
- **Joint limits:** Configurable min/max pulse widths per joint via hardware trigger pins
- **EEPROM calibration:** Stores calibrated pulse ranges persistently

### Firmware Fixes Applied

The C++ firmware (`ArmControl.h`/`.cpp`) received the following fixes:
- **Dead macro removal:** 6 unused macros removed from `ArmControl.h`
- **Typo correction:** `MED_DEVIDE` renamed to `MIDPOINT_DIV`
- **Semantic rename:** 3 misleading names (`QUEUE_ERROR`→`QUEUE_ERR`, `UNAVAIL_SER`→`SER_ERR`, `INCREASE`→`INC_STEP`) — private methods in `ArmControl.cpp`
- **Private method naming:** 15 methods unified to camelCase convention
- **Integer overflow fix:** 5 overflow bugs in `computeCubicTrajectory_` fixed with `int64_t` casts to prevent wrap-around on large intermediate products
- **Output clamping:** Cubic interpolation results clamped to `[COMMON_MIN_ANGLE, COMMON_MAX_ANGLE]` to protect servos from overshoot

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
- Numerical Jacobian via finite-difference perturbation (fixed: input arrays auto-cast to float to prevent integer truncation of perturbations)

### Tests (`Test/`)
- **`test_system.py`** — 16 integration tests:
  - 4 kinematics tests: FK at zero (checking x≈155, y≈0, z≈−118 per DH convention), FK at known angles, IK roundtrip (solve → FK → error < 1.0), Jacobian shape (6×6) and rank validation
  - 3 serial bridge tests (mocked): protocol format verification, text commands, concatenated sends
  - 6 voice parsing tests (conditionally skipped without `noisereduce`): action/object extraction for *take/pick/grab/get*, case insensitivity, no-action
  - 1 full pipeline test: mocked voice → YOLO → IK → serial → roundtrip verification
  - 2 3D visualization tests: `arm_fk_plot.png` (3-configuration arm layout), `arm_ik_plot.png` (convergence curve + solved arm)
- **`test_armcontrol.py`** — 18 unit/stress tests for the Python firmware port (edge-case clamping, trajectory queue overflow, cubic interpolation)
- **`armcontrol.py`** — Python port of `TrajectoryQueue` with 2D circular buffer
- **`Arm_IT_test.cpp`** — C++ host endurance test (10k iterations, all 7 tests passing)

### Command Bridge (`command/command_handeler.py`)
- Serial communication with ESP32
- `send_joint_data(t1..t6)` — sends formatted joint angles
- `send_command(cmd)` — sends text commands
- Response reading and logging

## Setup

```bash
pip install -r requirements.txt
./run_sys.sh
```

Requires **Python 3.10+** and an **ESP32** running the firmware at `/dev/ttyUSB0` (configurable in `command/command_handeler.py`).

### Firmware Upload

1. Open `actuator/ArmControl/ArmControl.ino` in Arduino IDE
2. Install dependencies: `Adafruit PWMServoDriver` library
3. Select **ESP32 Dev Module** board (install ESP32 board support if needed)
4. Upload to the appropriate serial port (e.g. `/dev/ttyUSB0`)

## 3D-Printable Parts

Gripper STL files in `actuator/Finger/`:
- Palm (top & bottom)
- Finger pads & hinge arms
- Left/right gear arms
- Servo gear components

## Requirements

- Python 3.10+
- ESP32 (Arduino framework)
- PCA9685 servo driver
- 6× servo motors (with limit switches)
- USB webcam
- Microphone
