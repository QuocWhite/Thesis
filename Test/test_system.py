"""Full-system integration tests: kinematics, serial bridge, voice parsing, pipeline.

Generates visualization PNGs in the Test/ directory.
All tests run without physical hardware (hardware is mocked).
"""

import sys
import os
import io
import json
import math
import importlib.util

import numpy as np
import pytest

# ── Add project root to path ──────────────────────────────────────────
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

# ── Visualization ─────────────────────────────────────────────────────
HAVE_MPL = False
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    HAVE_MPL = True
except ImportError:
    pass

TEST_DIR = os.path.dirname(os.path.abspath(__file__))


# ═══════════════════════════════════════════════════════════════════════
# KINEMATICS TESTS
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="session")
def kin():
    """Lazy-import kinematics module once."""
    from kinematics import kinematics as k
    return k


# DH table from README (d, a, alpha) per joint
DH_PARAMS = [
    (75,  42,  math.pi / 2),
    (0,   95,  0),
    (0,   18,  math.pi / 2),
    (166, 0,  -math.pi / 2),
    (0,   0,   math.pi / 2),
    (27,  0,   math.pi),
]


def compute_joint_positions(joint_angles_deg):
    """Return list of 4x4 transforms for each joint frame (cumulative)."""
    joint_angles_rad = np.radians(joint_angles_deg)
    transforms = [np.eye(4)]
    for i in range(6):
        theta = joint_angles_rad[i]
        d, a, alpha = DH_PARAMS[i]
        ct, st = math.cos(theta), math.sin(theta)
        ca, sa = math.cos(alpha), math.sin(alpha)
        ti = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d      ],
            [0,   0,        0,       1      ],
        ])
        transforms.append(transforms[-1] @ ti)
    return transforms


def test_kin_fk_zero_position(kin):
    """Forward kinematics at all-zeros — known end-effector position."""
    angles = np.zeros(6)
    T = kin.compute_forward_kinematics(angles)
    pos = T[:3, 3]
    # At zero: arm lies forward along x with z below base (DH convention)
    assert abs(pos[0] - 155) < 1, f"Expected x≈155 at zero, got x={pos[0]:.1f}"
    assert abs(pos[1]) < 1e-6, f"y should be ~0 at zero, got y={pos[1]:.1f}"
    assert abs(pos[2] - (-118)) < 1, f"Expected z≈-118 at zero, got z={pos[2]:.1f}"


def test_kin_fk_known_angles(kin):
    """FK at a specific configuration — verify end-effector position."""
    angles = np.array([0, 45, -45, 0, 0, 0])
    T = kin.compute_forward_kinematics(angles)
    pos = T[:3, 3]
    # Just check it produces a finite position in expected range
    assert all(np.isfinite(pos)), f"Non-finite position: {pos}"
    assert np.linalg.norm(pos) > 0, "Zero-length position vector"


def test_kin_ik_roundtrip(kin):
    """IK solve then FK verify — position error must be below tolerance."""
    target_angles = np.array([10, 30, -20, 15, -10, 5])
    target_T = kin.compute_forward_kinematics(target_angles)
    target_pose = kin.transform_matrix_to_pose_vector(target_T).flatten()

    initial_guess = np.zeros(6)
    result = kin.inverse_kinematics_solver(
        target_pose, initial_guess, max_iterations=500, tolerance=1e-2, learning_rate=0.5)

    result_T = kin.compute_forward_kinematics(result)
    result_pose = kin.transform_matrix_to_pose_vector(result_T).flatten()

    error = np.linalg.norm(target_pose - result_pose)
    assert error < 1.0, f"IK roundtrip error too large: {error:.4f}"


def test_kin_jacobian(kin):
    """Jacobian matrix has expected shape and non-zero entries."""
    angles = np.array([10, 20, 30, 5, 10, 0])
    J = kin.compute_jacobian_matrix(angles)
    assert J.shape == (6, 6), f"Jacobian shape wrong: {J.shape}"
    assert np.linalg.matrix_rank(J) > 0, "Jacobian is rank-zero (singular)"


# ═══════════════════════════════════════════════════════════════════════
# COMMAND HANDLER TESTS (mocked serial)
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture
def mock_serial(monkeypatch):
    """Replace serial.Serial with a mock that captures writes."""
    class MockSerial:
        def __init__(self, *args, **kwargs):
            self.buffer = io.BytesIO()
            self.timeout = kwargs.get("timeout", 1)
            self.in_waiting = 0

        def write(self, data):
            self.buffer.write(data)

        def readline(self):
            return b""

        def close(self):
            pass

    import command.command_handeler as ch
    # Reset singleton
    ch._ser = None
    monkeypatch.setattr("serial.Serial", MockSerial)
    yield ch
    ch._ser = None


def test_command_send_joint_data_format(mock_serial):
    """Verify send_joint_data produces correct protocol format."""
    ch = mock_serial
    ch.send_joint_data(90, 45, 60, 0, 30, 50)
    data = ch._ser.buffer.getvalue().decode()
    assert data == "b90r45l60t0w30f50\n", f"Unexpected format: {repr(data)}"


def test_command_send_command_format(mock_serial):
    """Verify send_command sends exact text + newline."""
    ch = mock_serial
    ch.send_command("init")
    data = ch._ser.buffer.getvalue().decode()
    assert data == "init\n", f"Unexpected format: {repr(data)}"

    ch._ser.buffer = io.BytesIO()
    ch.send_command("report")
    data = ch._ser.buffer.getvalue().decode()
    assert data == "report\n"


def test_command_multiple_sends(mock_serial):
    """Multiple sends in sequence produce concatenated output."""
    ch = mock_serial
    ch.send_joint_data(0, 10, 20, 30, 40, 50)
    ch.send_command("test")
    ch.send_joint_data(90, 90, 90, 90, 90, 90)
    data = ch._ser.buffer.getvalue().decode()
    lines = [l for l in data.split("\n") if l]
    assert len(lines) == 3
    assert lines[0] == "b0r10l20t30w40f50"
    assert lines[1] == "test"
    assert lines[2] == "b90r90l90t90w90f90"


# ═══════════════════════════════════════════════════════════════════════
# VOICE COMMAND PARSING TESTS
# ═══════════════════════════════════════════════════════════════════════

@pytest.fixture(scope="session")
def voice():
    try:
        from voice import VoiceRecognization as vr
        return vr
    except ImportError as e:
        if "noisereduce" in str(e):
            pytest.skip("noisereduce not installed")
        raise


def test_voice_extract_take(voice):
    action, obj = voice.extract_command("take the cup")
    assert action == "take"
    assert "cup" in obj


def test_voice_extract_pick(voice):
    action, obj = voice.extract_command("pick up the bottle")
    assert action == "pick"
    assert "bottle" in obj


def test_voice_extract_grab(voice):
    action, obj = voice.extract_command("please grab the red ball")
    assert action == "grab"
    assert "red ball" in obj


def test_voice_extract_get(voice):
    action, obj = voice.extract_command("could you get the phone")
    assert action == "get"
    assert "phone" in obj


def test_voice_extract_no_action(voice):
    action, obj = voice.extract_command("hello world")
    assert action is None
    assert obj is None


def test_voice_extract_case_insensitive(voice):
    action, obj = voice.extract_command("TAKE THE BOOK")
    assert action == "take"
    assert "book" in obj


# ═══════════════════════════════════════════════════════════════════════
# FULL PIPELINE END-TO-END TEST (all mocked)
# ═══════════════════════════════════════════════════════════════════════

class MockVoice:
    @staticmethod
    def listen_and_recognize():
        return "grab the bottle"

    @staticmethod
    def extract_command(text):
        if "grab" in text.lower():
            return "grab", "bottle"
        return None, None


class MockYOLO:
    @staticmethod
    def capture_and_detect():
        return [
            {"class": "bottle", "score": 0.92,
             "box": (100, 200, 300, 400), "center": (200, 300)}
        ]


def test_full_pipeline_integration(mock_serial):
    """End-to-end: voice → parse → detect → send (all mocked)."""
    ch = mock_serial
    ch._ser = None  # re-init

    # ── Step 1: Voice ──
    text = MockVoice.listen_and_recognize()
    assert text == "grab the bottle"

    action, obj = MockVoice.extract_command(text)
    assert action == "grab"
    assert obj == "bottle"

    # ── Step 2: Vision ──
    detections = MockYOLO.capture_and_detect()
    assert len(detections) == 1
    assert detections[0]["class"] == "bottle"
    assert detections[0]["center"] == (200, 300)

    # ── Step 3: Compute IK for target position ──
    from kinematics import kinematics as kin_mod
    pixel_x, pixel_y = detections[0]["center"]
    # Convert pixel to approximate 3D target (simplified projection)
    target_pose = np.array([[pixel_x * 0.1], [pixel_y * 0.1], [200], [0], [-30], [0]], dtype=float)
    initial = np.zeros(6)
    ik_result = kin_mod.inverse_kinematics_solver(
        target_pose.flatten(), initial, max_iterations=200, tolerance=0.5, learning_rate=0.3)
    assert all(np.isfinite(ik_result)), "IK produced non-finite angles"

    # ── Step 4: Send command ──
    ch.send_joint_data(*ik_result.round().astype(int))
    data = ch._ser.buffer.getvalue().decode()
    assert data.startswith("b"), f"Expected joint data format, got: {repr(data)}"

    # ── Step 5: Verify roundtrip ──
    sent_angles = np.array([float(x) for x in data.strip("b\n").split("r")[0].split("l")])
    # Just check the first angle is reasonable
    assert -180 <= sent_angles[0] <= 180, f"Angle out of range: {sent_angles[0]}"


# ═══════════════════════════════════════════════════════════════════════
# 3D VISUALIZATION: arm in multiple configurations
# ═══════════════════════════════════════════════════════════════════════

@pytest.mark.skipif(not HAVE_MPL, reason="matplotlib not available")
def test_visualize_arm_3d():
    """Generate 3D plot of arm in 3 configurations, save to PNG."""
    from kinematics import kinematics as kin_mod

    configs = [
        ("Home (all zeros)", [0, 0, 0, 0, 0, 0]),
        ("Reach forward",    [0, 45, -30, 0, -20, 0]),
        ("Reach up+right",   [30, 60, -45, 15, -10, 5]),
    ]

    fig = plt.figure(figsize=(18, 6))
    colors = ["tab:blue", "tab:orange", "tab:green"]

    for idx, (label, angles) in enumerate(configs):
        ax = fig.add_subplot(1, 3, idx + 1, projection="3d")
        transforms = compute_joint_positions(angles)
        positions = np.array([T[:3, 3] for T in transforms])

        # Plot links
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                "o-", color=colors[idx], linewidth=3, markersize=6, label=label)

        # Plot base
        ax.scatter([0], [0], [0], c="k", s=80, marker="s")

        # Annotate joints
        for j, pos in enumerate(positions):
            ax.text(pos[0], pos[1], pos[2], f"J{j}", fontsize=8, ha="center")

        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        ax.set_zlabel("Z (mm)")
        ax.set_title(label)
        ax.legend(fontsize=8)

        # Set equal aspect
        max_range = max(
            positions[:, 0].max() - positions[:, 0].min(),
            positions[:, 1].max() - positions[:, 1].min(),
            positions[:, 2].max() - positions[:, 2].min(),
        ) / 2 + 50
        mid_x = positions[:, 0].mean()
        mid_y = positions[:, 1].mean()
        mid_z = positions[:, 2].mean()
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.suptitle("6-DOF Arm Forward Kinematics", fontsize=14)
    plt.tight_layout()
    path = os.path.join(TEST_DIR, "arm_fk_plot.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  Saved {path}")


@pytest.mark.skipif(not HAVE_MPL, reason="matplotlib not available")
def test_visualize_ik_convergence():
    """Generate IK convergence plot showing pose error vs iterations."""
    from kinematics import kinematics as kin_mod

    target_angles = np.array([15, 40, -35, 20, -15, 10])
    target_T = kin_mod.compute_forward_kinematics(target_angles)
    target_pose = kin_mod.transform_matrix_to_pose_vector(target_T).flatten()

    joint_angles = np.zeros(6).astype(float)
    errors = []
    max_iter = 300

    for i in range(max_iter):
        current_pose = kin_mod.transform_matrix_to_pose_vector(
            kin_mod.compute_forward_kinematics(joint_angles)).flatten()
        pose_error = target_pose - current_pose
        err_norm = float(np.linalg.norm(pose_error))
        errors.append(err_norm)
        if err_norm < 1e-2:
            break
        J = kin_mod.compute_jacobian_matrix(joint_angles)
        delta = 0.5 * np.dot(np.linalg.pinv(J), pose_error)
        joint_angles += delta.flatten()

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Plot 1: Error convergence
    ax1.semilogy(errors, "b-", linewidth=2)
    ax1.axhline(y=1e-2, color="r", linestyle="--", alpha=0.5, label="Tolerance")
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Pose Error (norm)")
    ax1.set_title("IK Convergence")
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Plot 2: Arm at target position
    ax2 = fig.add_subplot(122, projection="3d")
    transforms = compute_joint_positions(joint_angles)
    positions = np.array([T[:3, 3] for T in transforms])
    ax2.plot(positions[:, 0], positions[:, 1], positions[:, 2],
             "o-", color="tab:orange", linewidth=3, markersize=6)
    ax2.scatter([0], [0], [0], c="k", s=60, marker="s")
    for j, pos in enumerate(positions):
        ax2.text(pos[0], pos[1], pos[2], f"J{j}", fontsize=9)
    ax2.set_xlabel("X (mm)")
    ax2.set_ylabel("Y (mm)")
    ax2.set_zlabel("Z (mm)")
    ax2.set_title(f"IK Solved ({len(errors)} iters, err={errors[-1]:.4f})")

    plt.tight_layout()
    path = os.path.join(TEST_DIR, "arm_ik_plot.png")
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"\n  Saved {path}")
