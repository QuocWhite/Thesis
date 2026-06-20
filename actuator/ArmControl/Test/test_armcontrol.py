"""Unit tests for ported arm control logic (no Arduino hardware)."""

from armcontrol import (
    constrain, angle_to_value, value_to_angle,
    TrajectoryQueue, is_all_joint_motion_complete,
    prepare_next_motion_step, compute_cubic_trajectory,
    SERVO_MIN_SOURCE, SERVO_MAX_SOURCE, COMMON_MIN_ANGLE, COMMON_MAX_ANGLE,
    generate_joint_motion_complete
)


def test_constrain():
    """Test value clamping."""
    assert constrain(-50) == SERVO_MIN_SOURCE
    assert constrain(500) == SERVO_MAX_SOURCE
    assert constrain(90) == 90


def test_angle_to_value_and_value_to_angle():
    """Test angle-to-pulse-to-angle roundtrip."""
    min_target = COMMON_MIN_ANGLE
    max_target = COMMON_MAX_ANGLE
    assert angle_to_value(0, min_target, max_target) == min_target
    assert angle_to_value(180, min_target, max_target) == max_target
    mid_angle = 90
    mid_value = angle_to_value(mid_angle, min_target, max_target)
    assert value_to_angle(mid_value, min_target, max_target) == mid_angle


def test_queue_operations():
    """Test circular queue enqueue/dequeue/full/empty."""
    max_joint = 6
    size = 3
    q = TrajectoryQueue(size, max_joint)
    assert q.is_empty()
    data1 = [1, 2, 3, 4, 5, 6]
    data2 = [7, 8, 9, 10, 11, 12]
    data3 = [13, 14, 15, 16, 17, 18]
    assert q.enqueue(data1)
    assert not q.is_empty()
    assert q.enqueue(data2)
    assert q.enqueue(data3)
    assert not q.enqueue([19, 20, 21, 22, 23, 24])
    assert q.is_full()
    assert q.dequeue() == data1
    assert q.dequeue() == data2
    assert q.dequeue() == data3
    assert q.dequeue() is None
    assert q.is_empty()


def test_is_all_joint_motion_complete():
    """Test all-joints-complete detection."""
    assert is_all_joint_motion_complete([True]*6) == True
    assert is_all_joint_motion_complete(
        [True, False, True, True, True, True]) == False
    assert is_all_joint_motion_complete([False]*6) == False


def test_prepare_next_motion_step():
    """Test advancing to next queued waypoint."""
    previous = [0, 0, 0, 0, 0, 0]
    current = [1, 2, 3, 4, 5, 6]
    joint_motion_complete = [True]*6
    q = TrajectoryQueue(3, 6)
    q.enqueue([10, 11, 12, 13, 14, 15])
    prev2, curr2, jmc2 = prepare_next_motion_step(
        previous[:], current[:], joint_motion_complete[:], q)
    assert prev2 == current
    assert jmc2 == [False]*6
    assert curr2 == [10, 11, 12, 13, 14, 15]


def test_compute_cubic_trajectory():
    """Test cubic spline interpolation."""
    start = 100
    end = 200
    T = 10
    for t in range(0, T+1):
        pos = compute_cubic_trajectory(start, end, t, T)
        assert isinstance(pos, int)
    assert compute_cubic_trajectory(start, end, T+1, T) == end