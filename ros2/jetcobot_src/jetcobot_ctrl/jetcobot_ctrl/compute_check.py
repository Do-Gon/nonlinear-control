import numpy as np
import math

# Denavit–Hartenberg parameters for myCobot 280 (Arduino edition)
# Each tuple: (a_{i-1} [mm], alpha_{i-1} [rad], d_i [mm])
dh_params = [
    (0.0,    np.deg2rad(90),  76.2),
    (25.0,   0.0,             0.0),
    (135.0,  0.0,             0.0),
    (0.0,    np.deg2rad(90), 147.0),
    (0.0,    np.deg2rad(-90), 0.0),
    (0.0,    0.0,            34.3)
]

def dh_transform(a, alpha, d, theta):
    """Compute individual DH transformation."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ ct, -st*ca,  st*sa, a*ct],
        [ st,  ct*ca, -ct*sa, a*st],
        [  0,     sa,     ca,    d],
        [  0,      0,      0,    1]
    ])

def forward_kinematics(thetas):
    """Compute forward kinematics for a list of 6 joint angles (radians)."""
    T = np.eye(4)
    for (a, alpha, d), theta in zip(dh_params, thetas):
        T = T @ dh_transform(a, alpha, d, theta)
    return T

def euler_to_rot(rx, ry, rz):
    """Convert roll, pitch, yaw (in radians) to rotation matrix R = Rz * Ry * Rx."""
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                   [          0, 1,          0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [         0,           0, 1]])
    return Rz @ Ry @ Rx

def inverse_kinematics(coords):
    """
    Analytical IK for myCobot 280:
      coords: [x, y, z, rx, ry, rz] in mm and degrees.
    Returns joint angles [theta1…theta6] in radians.
    """
    x, y, z, rx, ry, rz = coords
    # Convert Euler angles to radians
    rx, ry, rz = np.deg2rad([rx, ry, rz])
    R06 = euler_to_rot(rx, ry, rz)

    # Wrist center
    d6 = dh_params[5][2]
    pw = np.array([x, y, z]) - d6 * R06[:, 2]

    # theta1
    theta1 = np.arctan2(pw[1], pw[0])

    # Planar distances for shoulder/elbow
    d1 = dh_params[0][2]
    r = np.hypot(pw[0], pw[1])
    s = pw[2] - d1

    # Link lengths
    a2, a3 = dh_params[1][0], dh_params[2][0]
    # Law of cosines for theta3
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    D = np.clip(D, -1.0, 1.0)
    theta3 = np.arctan2(+np.sqrt(1 - D**2), D)  # elbow-down

    # theta2
    phi = np.arctan2(s, r)
    psi = np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
    theta2 = phi - psi

    # Compute rotation R0_3
    T1 = dh_transform(*dh_params[0], theta1)
    T2 = dh_transform(*dh_params[1], theta2)
    T3 = dh_transform(*dh_params[2], theta3)
    R0_3 = (T1 @ T2 @ T3)[:3, :3]

    # Compute wrist orientation
    R3_6 = R0_3.T @ R06

    # Decompose R3_6 using Z-Y-X
    sy = math.sqrt(R3_6[0,2]**2 + R3_6[1,2]**2)
    singular = sy < 1e-6

    if not singular:
        theta4 = np.arctan2(R3_6[1,2], R3_6[0,2])
        theta5 = np.arctan2(sy,        R3_6[2,2])
        theta6 = np.arctan2(R3_6[2,1], -R3_6[2,0])
    else:
        theta4 = 0.0
        theta5 = np.arctan2(sy, R3_6[2,2])
        theta6 = np.arctan2(-R3_6[1,0], R3_6[1,1])

    return [theta1, theta2, theta3, theta4, theta5, theta6]

# Example target:
coords = [276.80, -86.20, 120.30, -178.60, -3.45, -45.51]

# 1) Compute IK
thetas = inverse_kinematics(coords)

# 2) FK of that solution
T_sol = forward_kinematics(thetas)
pos_sol = T_sol[:3, 3]
R_sol   = T_sol[:3, :3]

# 3) Print errors
print("== IK → FK verification ==")
print("Desired position:", coords[:3])
print("IK position     :", pos_sol)
print("Position error  :", np.linalg.norm(pos_sol - coords[:3]), "mm")

# Extract RPY from R_sol in the SAME convention:
rx_sol = math.atan2(R_sol[2,1], R_sol[2,2])
ry_sol = math.atan2(-R_sol[2,0], math.sqrt(R_sol[2,1]**2 + R_sol[2,2]**2))
rz_sol = math.atan2(R_sol[1,0], R_sol[0,0])
print("Desired RPY     :", coords[3:])
print("IK   RPY        :", np.rad2deg([rx_sol, ry_sol, rz_sol]))
print("RPY error       :", np.rad2deg([coords[3]-rz_sol,
                                      coords[4]-ry_sol,
                                      coords[5]-rx_sol]))

