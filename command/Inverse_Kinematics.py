import numpy as np


def rotm2eul(rotm):
    sy = np.sqrt(rotm[0, 0] ** 2 + rotm[1, 0] ** 2)
    if sy < 1e-6:
        phi = np.arctan2(rotm[1, 2], rotm[1, 1])
        theta = np.arctan2(-rotm[2, 0], sy)
        psi = 0.0
    else:
        phi = np.arctan2(rotm[2, 1], rotm[2, 2])
        theta = np.arctan2(-rotm[2, 0], sy)
        psi = np.arctan2(rotm[1, 0], rotm[0, 0])
    return np.array([psi, theta, phi]) * 180 / np.pi


def convert_matrix(A):
    euler = rotm2eul(A[:3, :3])
    B = np.zeros((6, 1))
    B[0][0] = A[0, 3]
    B[1][0] = A[1, 3]
    B[2][0] = A[2, 3]
    B[3][0] = euler[0]
    B[4][0] = euler[1]
    B[5][0] = euler[2]
    return B


def forward_kinematics(theta):
    theta = theta * np.pi / 180
    DH = np.array([
        [75, theta[0], 42, np.pi / 2],
        [0, theta[1], 95, 0],
        [0, theta[2], 18, np.pi / 2],
        [166, theta[3], 0, -np.pi / 2],
        [0, theta[4], 0, np.pi / 2],
        [27, theta[5], 0, np.pi]
    ])

    T = np.eye(4)
    for i in range(6):
        ct, st = np.cos(DH[i, 1]), np.sin(DH[i, 1])
        ca, sa = np.cos(DH[i, 3]), np.sin(DH[i, 3])
        T = np.dot(T, np.array([
            [ct, -st * ca, st * sa, DH[i, 2] * ct],
            [st, ct * ca, -ct * sa, DH[i, 2] * st],
            [0, sa, ca, DH[i, 0]],
            [0, 0, 0, 1]
        ]))
    return T


def calculate_jacobian(theta):
    J = np.zeros((6, 6))
    delta = 1e-3
    pose = convert_matrix(forward_kinematics(theta))
    for k in range(6):
        theta_d = theta.copy()
        theta_d[k] += delta
        pose_d = convert_matrix(forward_kinematics(theta_d))
        J[:, k] = ((pose_d - pose) / delta).flatten()
    return J


def inverse_kinematics_jacobian(target_pose, initial_theta, max_iter=500, tol=1e-2, alpha=0.5):
    theta = initial_theta.copy().astype(float)
    for _ in range(max_iter):
        current_pose = convert_matrix(forward_kinematics(theta))
        error = target_pose - current_pose
        if np.linalg.norm(error) < tol:
            break
        J = calculate_jacobian(theta)
        d_theta = alpha * np.dot(np.linalg.pinv(J), error)
        theta += d_theta.flatten()
    return theta
