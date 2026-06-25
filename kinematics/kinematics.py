"""Forward and inverse kinematics for a 6-DOF robotic arm via DH parameters."""

import numpy as np

JOINT_COUNT = 6
EULER_SINGULARITY_THRESH = 1e-6
JACOBIAN_DELTA = 1e-3

# Denavit-Hartenberg parameters: [d, theta_index, a, alpha]
# Columns: d (mm), theta (rad, from input), a (mm), alpha (rad)
DH_TABLE = [
    [75, 0, 42, np.pi / 2],
    [0, 1, 95, 0],
    [0, 2, 18, np.pi / 2],
    [166, 3, 0, -np.pi / 2],
    [0, 4, 0, np.pi / 2],
    [27, 5, 0, np.pi],
]


def rotation_matrix_to_euler_angles(rotation_matrix):
    sy = np.sqrt(rotation_matrix[0, 0] ** 2 + rotation_matrix[1, 0] ** 2)
    if sy < EULER_SINGULARITY_THRESH:
        roll = np.arctan2(rotation_matrix[1, 2], rotation_matrix[1, 1])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = 0.0
    else:
        roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        pitch = np.arctan2(-rotation_matrix[2, 0], sy)
        yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    return np.array([yaw, pitch, roll]) * 180 / np.pi


def transform_matrix_to_pose_vector(transform_matrix):
    euler_angles = rotation_matrix_to_euler_angles(transform_matrix[:3, :3])
    pose_vector = np.zeros((JOINT_COUNT, 1))
    pose_vector[0][0] = transform_matrix[0, 3]
    pose_vector[1][0] = transform_matrix[1, 3]
    pose_vector[2][0] = transform_matrix[2, 3]
    pose_vector[3][0] = euler_angles[0]
    pose_vector[4][0] = euler_angles[1]
    pose_vector[5][0] = euler_angles[2]
    return pose_vector


def compute_forward_kinematics(joint_angles_deg):
    joint_angles_deg = np.asarray(joint_angles_deg, dtype=float)
    joint_angles_rad = joint_angles_deg * np.pi / 180
    dh_parameters = np.array([
        [row[0], joint_angles_rad[row[1]], row[2], row[3]]
        for row in DH_TABLE
    ])
    transform = np.eye(4)
    for i in range(JOINT_COUNT):
        theta, d, a, alpha = (
            dh_parameters[i, 1], dh_parameters[i, 0],
            dh_parameters[i, 2], dh_parameters[i, 3],
        )
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        current_transform = np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])
        transform = np.dot(transform, current_transform)
    return transform


def compute_jacobian_matrix(joint_angles_deg):
    joint_angles_deg = np.asarray(joint_angles_deg, dtype=float)
    jacobian = np.zeros((JOINT_COUNT, JOINT_COUNT))
    base_pose = transform_matrix_to_pose_vector(
        compute_forward_kinematics(joint_angles_deg))
    for joint_index in range(JOINT_COUNT):
        perturbed_angles = joint_angles_deg.copy()
        perturbed_angles[joint_index] += JACOBIAN_DELTA
        perturbed_pose = transform_matrix_to_pose_vector(
            compute_forward_kinematics(perturbed_angles))
        jacobian[:, joint_index] = (
            (perturbed_pose - base_pose) / JACOBIAN_DELTA
        ).flatten()
    return jacobian


def inverse_kinematics_solver(target_pose_vector, initial_joint_angles_deg,
                              max_iterations=500, tolerance=1e-2,
                              learning_rate=0.5):
    target_pose_vector = (
        np.asarray(target_pose_vector).reshape(JOINT_COUNT, 1).astype(float)
    )
    joint_angles = initial_joint_angles_deg.copy().astype(float)
    for _ in range(max_iterations):
        current_pose_vector = transform_matrix_to_pose_vector(
            compute_forward_kinematics(joint_angles))
        pose_error = target_pose_vector - current_pose_vector
        if np.linalg.norm(pose_error) < tolerance:
            break
        jacobian = compute_jacobian_matrix(joint_angles)
        delta_theta = learning_rate * np.dot(
            np.linalg.pinv(jacobian), pose_error)
        joint_angles += delta_theta.flatten()
    return joint_angles
