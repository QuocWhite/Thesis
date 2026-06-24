"""Ported arm control logic for host-side testing (no Arduino hardware)."""

SERVO_MIN_SOURCE = 0
SERVO_MAX_SOURCE = 180
COMMON_MIN_ANGLE = 400
COMMON_MAX_ANGLE = 2450

FACT_FINGER_MIN_ANGLE = 500
FACT_FINGER_MAX_ANGLE = 1560


def constrain(value, min_val=SERVO_MIN_SOURCE, max_val=SERVO_MAX_SOURCE):
    """Constrain value within [min_val, max_val]."""
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    return value


def angle_to_value(angle, min_target, max_target):
    """Convert angle (degrees) to servo pulse value."""
    converted_angle = constrain(angle)
    value = ((converted_angle - SERVO_MIN_SOURCE) * (max_target -
             min_target)) // (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + min_target
    return value


def value_to_angle(value, min_target, max_target):
    """Convert servo pulse value to angle (degrees)."""
    angle = ((value - min_target) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE)
             ) // (max_target - min_target) + SERVO_MIN_SOURCE
    return angle


class TrajectoryQueue:
    """Circular buffer for multi-joint trajectory waypoints (matches firmware)."""

    def __init__(self, size, max_joint):
        self.buffer = [[0] * size for _ in range(max_joint)]
        self.front = -1
        self.rear = -1
        self.max_size = size
        self.max_joint = max_joint

    def is_full(self):
        """Return True if the queue is full."""
        return self.front == (self.rear + 1) % self.max_size

    def is_empty(self):
        """Return True if the queue is empty."""
        return self.front == -1

    def enqueue(self, data):
        """Push a multi-joint waypoint onto the queue."""
        if self.is_full():
            return False
        if self.is_empty():
            self.front = 0
            self.rear = 0
        else:
            self.rear = (self.rear + 1) % self.max_size
        for i in range(self.max_joint):
            self.buffer[i][self.rear] = data[i]
        return True

    def dequeue(self):
        """Pop the front multi-joint waypoint from the queue."""
        if self.is_empty():
            return None
        data = [self.buffer[i][self.front] for i in range(self.max_joint)]
        if self.front == self.rear:
            self.front = -1
            self.rear = -1
        else:
            self.front = (self.front + 1) % self.max_size
        return data


def is_all_joint_motion_complete(joint_motion_complete):
    """Return True if all joints have finished their motion."""
    return all(joint_motion_complete)


def prepare_next_motion_step(previous_joint_pulse, current_joint_pulse,
                             joint_motion_complete, trajectory_queue):
    """Advance to the next queued waypoint for all joints."""
    for i in range(len(previous_joint_pulse)):
        previous_joint_pulse[i] = current_joint_pulse[i]
        joint_motion_complete[i] = False
    if not trajectory_queue.is_empty():
        next_pulse = trajectory_queue.dequeue()
        for i in range(len(current_joint_pulse)):
            current_joint_pulse[i] = next_pulse[i]
    return previous_joint_pulse, current_joint_pulse, joint_motion_complete


def compute_cubic_trajectory(start_pos, end_pos, t, T):
    """Compute cubic spline position at time t over duration T."""
    delta = end_pos - start_pos
    a0 = start_pos
    a1 = 0
    a2 = (3 * delta) / (T ** 2)
    a3 = (-2 * delta) / (T ** 3)
    if t <= T:
        pos = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
        return int(round(pos))
    else:
        return end_pos


def generate_joint_motion_complete(all_true=True):
    """Generate a sample motion-complete array for 6 joints."""
    return [True] * 6 if all_true else [True, False, True, True, True, True]
