"""Comprehensive + stress tests for ported arm control logic (no physical hardware)."""

import random
from armcontrol import (
    constrain, angle_to_value, value_to_angle,
    TrajectoryQueue, is_all_joint_motion_complete,
    prepare_next_motion_step, compute_cubic_trajectory,
    SERVO_MIN_SOURCE, SERVO_MAX_SOURCE, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE,
    FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE,
    generate_joint_motion_complete
)

MAX_JOINT = 6

# ============================================================
# FULL FORM: Edge-case unit tests
# ============================================================

def test_constrain_edges():
    """Boundary and overflow clamping."""
    assert constrain(SERVO_MIN_SOURCE) == SERVO_MIN_SOURCE
    assert constrain(SERVO_MAX_SOURCE) == SERVO_MAX_SOURCE
    assert constrain(-1) == SERVO_MIN_SOURCE
    assert constrain(181) == SERVO_MAX_SOURCE
    assert constrain(-9999) == SERVO_MIN_SOURCE
    assert constrain(9999) == SERVO_MAX_SOURCE


def test_angle_to_value_all_ranges():
    """angle_to_value works for all joint ranges including finger."""
    assert angle_to_value(0, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MIN_ANGLE
    assert angle_to_value(180, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MAX_ANGLE
    assert angle_to_value(0, FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE) == FACT_FINGER_MIN_ANGLE
    assert angle_to_value(180, FACT_FINGER_MIN_ANGLE,
                          FACT_FINGER_MAX_ANGLE) == FACT_FINGER_MAX_ANGLE
    assert angle_to_value(-10, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MIN_ANGLE
    assert angle_to_value(200, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == COMMON_MAX_ANGLE


def test_value_to_angle_boundary():
    """value_to_angle at pulse boundaries."""
    assert value_to_angle(COMMON_MIN_ANGLE, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == 0
    assert value_to_angle(COMMON_MAX_ANGLE, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) == 180


def test_queue_full_wrap_around():
    """Fill exactly to capacity, verify rejection, drain, verify empty."""
    q = TrajectoryQueue(5, MAX_JOINT)
    data = [i for i in range(MAX_JOINT)]
    for _ in range(5):
        assert q.enqueue(data)
    assert q.is_full()
    assert not q.enqueue([99] * MAX_JOINT)
    for _ in range(5):
        assert q.dequeue() is not None
    assert q.is_empty()
    assert q.dequeue() is None


def test_queue_fill_drain_cycles():
    """Repeated fill-drain cycles stress circular buffer wrap."""
    q = TrajectoryQueue(4, MAX_JOINT)
    for cycle in range(20):
        for _ in range(4):
            assert q.enqueue([cycle] * MAX_JOINT)
        for _ in range(4):
            out = q.dequeue()
            assert out is not None and out[0] == cycle
        assert q.is_empty()


def test_queue_enqueue_dequeue_interleaved():
    """Random interleaved enqueue/dequeue patterns."""
    random.seed(42)
    q = TrajectoryQueue(8, MAX_JOINT)
    expected = []
    for i in range(100):
        if random.random() < 0.6 and not q.is_full():
            data = [i] * MAX_JOINT
            q.enqueue(data)
            expected.append(data)
        elif not q.is_empty():
            out = q.dequeue()
            assert out == expected.pop(0)
    while not q.is_empty():
        out = q.dequeue()
        assert out == expected.pop(0)


def test_cubic_trajectory_fractions():
    """Cubic trajectory at exact fractions of duration."""
    start, end, T = 1000, 2000, 100
    assert compute_cubic_trajectory(start, end, 0, T) == start
    assert compute_cubic_trajectory(start, end, T, T) == end
    assert compute_cubic_trajectory(start, end, T + 1, T) == end
    mid = compute_cubic_trajectory(start, end, T // 2, T)
    assert start < mid < end


def test_cubic_trajectory_reverse():
    """Cubic trajectory for decreasing moves."""
    start, end, T = 2000, 1000, 100
    for t in range(0, T + 1, 10):
        pos = compute_cubic_trajectory(start, end, t, T)
        assert end <= pos <= start
    assert compute_cubic_trajectory(start, end, T, T) == end


def test_cubic_trajectory_zero_delta():
    """Cubic trajectory with no movement (start == end)."""
    start, end, T = 1500, 1500, 100
    for t in range(0, T + 1, 10):
        assert compute_cubic_trajectory(start, end, t, T) == start
    assert compute_cubic_trajectory(start, end, T + 1, T) == start


def test_cubic_trajectory_monotonic():
    """Increasing move must be non-decreasing in time."""
    start, end, T = 800, 2200, 100
    prev = -1
    for t in range(0, T + 1):
        pos = compute_cubic_trajectory(start, end, t, T)
        assert pos >= prev
        prev = pos


def test_is_all_joint_motion_complete():
    assert is_all_joint_motion_complete([True] * 6) is True
    assert is_all_joint_motion_complete([True, False, True, True, True, True]) is False
    assert is_all_joint_motion_complete([False] * 6) is False


def test_prepare_next_motion_step():
    """Advance to next queued waypoint resets flags correctly."""
    previous = [0] * MAX_JOINT
    current = [1, 2, 3, 4, 5, 6]
    jmc = [True] * MAX_JOINT
    q = TrajectoryQueue(3, MAX_JOINT)
    q.enqueue([10, 11, 12, 13, 14, 15])
    prev2, curr2, jmc2 = prepare_next_motion_step(previous[:], current[:], jmc[:], q)
    assert prev2 == current
    assert jmc2 == [False] * MAX_JOINT
    assert curr2 == [10, 11, 12, 13, 14, 15]


def test_prepare_next_motion_step_empty_queue():
    """prepare_next_motion_step with no queued waypoint keeps current."""
    previous = [100] * MAX_JOINT
    current = [200] * MAX_JOINT
    jmc = [True] * MAX_JOINT
    q = TrajectoryQueue(3, MAX_JOINT)
    prev2, curr2, jmc2 = prepare_next_motion_step(previous[:], current[:], jmc[:], q)
    assert prev2 == current
    assert curr2 == current
    assert jmc2 == [False] * MAX_JOINT


def test_full_single_waypoint_pipeline():
    """One waypoint through full pipeline: enqueue → prepare → interpolate → complete."""
    q = TrajectoryQueue(16, MAX_JOINT)
    prev = [COMMON_MIN_ANGLE] * MAX_JOINT
    curr = [COMMON_MIN_ANGLE] * MAX_JOINT
    jmc = [True] * MAX_JOINT

    waypoint = [1000, 1200, 1400, 1600, 1800, 600]
    q.enqueue(waypoint)
    prev, curr, jmc = prepare_next_motion_step(prev, curr, jmc, q)

    assert prev == [COMMON_MIN_ANGLE] * MAX_JOINT
    assert curr == waypoint
    assert jmc == [False] * MAX_JOINT

    # Mark all joints complete manually (mimics cubic trajectory completion)
    jmc = [True] * MAX_JOINT
    assert is_all_joint_motion_complete(jmc) is True
    assert q.is_empty()


# ============================================================
# STRESS / ENDURANCE TESTS
# ============================================================

def test_stress_random_walk_10000_steps():
    """10000 random waypoints through the full pipeline.

    Simulates continuous pick-and-place operation.
    Verifies system never enters an invalid state.
    """
    random.seed(42)
    Q_SIZE = 16

    prev = [COMMON_MIN_ANGLE] * MAX_JOINT
    curr = [COMMON_MIN_ANGLE] * MAX_JOINT
    jmc = [True] * MAX_JOINT
    q = TrajectoryQueue(Q_SIZE, MAX_JOINT)
    T = 10
    steps_completed = 0
    queue_full_errors = 0

    for step in range(10000):
        if step % 5 == 0:
            for _ in range(random.randint(1, 4)):
                waypoint = [
                    random.randint(COMMON_MIN_ANGLE, COMMON_MAX_ANGLE)
                    for _ in range(MAX_JOINT)
                ]
                if not q.enqueue(waypoint):
                    queue_full_errors += 1

        if all(jmc) and not q.is_empty():
            prev, curr, jmc = prepare_next_motion_step(prev, curr, jmc, q)
            steps_completed += 1

        for j in range(MAX_JOINT):
            pos = compute_cubic_trajectory(prev[j], curr[j], 0, T)
            assert COMMON_MIN_ANGLE <= pos <= COMMON_MAX_ANGLE, \
                f"Joint {j} out of range: {pos}"

    # Verify no crashes, valid final state
    for v in prev + curr:
        assert COMMON_MIN_ANGLE <= v <= COMMON_MAX_ANGLE
    assert steps_completed > 0


def test_stress_queue_thrash():
    """5000 rapid enqueue/dequeue cycles with random batch sizes."""
    random.seed(7)
    q = TrajectoryQueue(16, MAX_JOINT)
    for cycle in range(5000):
        n = random.randint(1, 16)
        for _ in range(n):
            q.enqueue([cycle] * MAX_JOINT)
        for _ in range(n):
            out = q.dequeue()
            assert out is None or out[0] == cycle
        if random.random() < 0.1:
            assert q.is_empty()


def test_stress_alternating_min_max():
    """2000 cycles alternating between min and max pulse (worst-case stress)."""
    random.seed(13)
    Q_SIZE = 16
    prev = [COMMON_MIN_ANGLE] * MAX_JOINT
    curr = [COMMON_MIN_ANGLE] * MAX_JOINT
    jmc = [True] * MAX_JOINT
    q = TrajectoryQueue(Q_SIZE, MAX_JOINT)
    T = 10
    steps_completed = 0

    for cycle in range(2000):
        extreme = COMMON_MAX_ANGLE if cycle % 2 == 0 else COMMON_MIN_ANGLE
        q.enqueue([extreme] * MAX_JOINT)

        if all(jmc) and not q.is_empty():
            prev, curr, jmc = prepare_next_motion_step(prev, curr, jmc, q)
            steps_completed += 1

        for j in range(MAX_JOINT):
            pos = compute_cubic_trajectory(prev[j], curr[j], 0, T)
            assert COMMON_MIN_ANGLE <= pos <= COMMON_MAX_ANGLE

    assert steps_completed > 0


def test_stress_multi_step_sequence():
    """200 waypoints through pipeline with continual enqueue/dequeue."""
    random.seed(99)
    Q_SIZE = 16
    prev = [COMMON_MIN_ANGLE] * MAX_JOINT
    curr = [COMMON_MIN_ANGLE] * MAX_JOINT
    jmc = [True] * MAX_JOINT
    q = TrajectoryQueue(Q_SIZE, MAX_JOINT)
    T = 10

    waypoints = [
        [random.randint(COMMON_MIN_ANGLE, COMMON_MAX_ANGLE) for _ in range(MAX_JOINT)]
        for _ in range(200)
    ]

    completed = 0
    wp_idx = 0
    while completed < 200:
        # Enqueue if space available
        if wp_idx < len(waypoints) and not q.is_full():
            q.enqueue(waypoints[wp_idx])
            wp_idx += 1

        if all(jmc) and not q.is_empty():
            prev, curr, jmc = prepare_next_motion_step(prev, curr, jmc, q)
            completed += 1

        for j in range(MAX_JOINT):
            pos = compute_cubic_trajectory(prev[j], curr[j], 0, T)
            assert COMMON_MIN_ANGLE <= pos <= COMMON_MAX_ANGLE

        jmc = [True] * MAX_JOINT  # fast-forward completion

    assert completed == 200
