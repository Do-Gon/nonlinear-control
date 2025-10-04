# #!/usr/bin/env python3
# import time
# import numpy as np
# from pymycobot.mycobot import MyCobot

# # Denavit–Hartenberg parameters for myCobot 280 (Arduino edition)
# # Each tuple: (a_{i-1} [mm], alpha_{i-1} [rad], d_i [mm])
# dh_params = [
#     (0.0,    np.deg2rad(90),  76.2),
#     (25.0,   0.0,             0.0),
#     (135.0,  0.0,             0.0),
#     (0.0,    np.deg2rad(90), 147.0),
#     (0.0,    np.deg2rad(-90), 0.0),
#     (0.0,    0.0,            34.3)
# ]

# def dh_transform(a, alpha, d, theta):
#     """Compute individual DH transformation."""
#     ca, sa = np.cos(alpha), np.sin(alpha)
#     ct, st = np.cos(theta), np.sin(theta)
#     return np.array([
#         [ ct, -st*ca,  st*sa, a*ct],
#         [ st,  ct*ca, -ct*sa, a*st],
#         [  0,     sa,     ca,    d],
#         [  0,      0,      0,    1]
#     ])

# def euler_to_rot(rx, ry, rz):
#     """Convert roll, pitch, yaw (in radians) to rotation matrix R = Rz * Ry * Rx."""
#     Rx = np.array([[1, 0, 0],
#                    [0, np.cos(rx), -np.sin(rx)],
#                    [0, np.sin(rx),  np.cos(rx)]])
#     Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
#                    [          0, 1,          0],
#                    [-np.sin(ry), 0, np.cos(ry)]])
#     Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
#                    [np.sin(rz),  np.cos(rz), 0],
#                    [         0,           0, 1]])
#     return Rz @ Ry @ Rx

# def inverse_kinematics(coords):
#     """
#     Analytical IK for myCobot 280:
#       coords: [x, y, z, rx, ry, rz] in mm and degrees.
#     Returns joint angles [theta1…theta6] in radians.
#     """
#     x, y, z, rx, ry, rz = coords
#     # Convert Euler angles to radians
#     rx, ry, rz = np.deg2rad([rx, ry, rz])
#     R06 = euler_to_rot(rx, ry, rz)

#     # Wrist center
#     d6 = dh_params[5][2]
#     pw = np.array([x, y, z]) - d6 * R06[:, 2]

#     # theta1
#     theta1 = np.arctan2(pw[1], pw[0])

#     # Planar distances for shoulder/elbow
#     d1 = dh_params[0][2]
#     r = np.hypot(pw[0], pw[1])
#     s = pw[2] - d1

#     # Link lengths
#     a2, a3 = dh_params[1][0], dh_params[2][0]
#     # Law of cosines for theta3
#     D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
#     D = np.clip(D, -1.0, 1.0)
#     theta3 = np.arctan2(+np.sqrt(1 - D**2), D)  # elbow-down

#     # theta2
#     phi = np.arctan2(s, r)
#     psi = np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
#     theta2 = phi - psi

#     # Compute rotation R0_3
#     T1 = dh_transform(*dh_params[0], theta1)
#     T2 = dh_transform(*dh_params[1], theta2)
#     T3 = dh_transform(*dh_params[2], theta3)
#     R0_3 = (T1 @ T2 @ T3)[:3, :3]

#     # Compute wrist orientation
#     R3_6 = R0_3.T @ R06

#     # Decompose R3_6 using Z-Y-X (theta4 about Z, theta5 about Y, theta6 about X)
#     # theta5:
#     sy = np.sqrt(R3_6[0,2]**2 + R3_6[1,2]**2)
#     singular = sy < 1e-6

#     if not singular:
#         theta4 = np.arctan2(R3_6[1,2], R3_6[0,2])
#         theta5 = np.arctan2(sy,      R3_6[2,2])
#         theta6 = np.arctan2(R3_6[2,1], -R3_6[2,0])
#     else:
#         # Gimbal lock: arbitrary set theta4 = 0
#         theta4 = 0.0
#         theta5 = np.arctan2(sy, R3_6[2,2])
#         theta6 = np.arctan2(-R3_6[1,0], R3_6[1,1])

#     return [theta1, theta2, theta3, theta4, theta5, theta6]

# def plan_joint_trajectory(theta_start, theta_goal, steps=50):
#     """Linear interpolation in joint space."""
#     theta_start = np.array(theta_start)
#     theta_goal = np.array(theta_goal)
#     return [
#         theta_start + (theta_goal - theta_start) * t/(steps-1)
#         for t in range(steps)
#     ]

# def main():
#     mc = MyCobot('/dev/ttyUSB0', 1000000)
#     speed = 50

#     def move_to(coords):
#         thetas = inverse_kinematics(coords)
#         current = np.deg2rad(mc.get_angles())
#         traj = plan_joint_trajectory(current, thetas, steps=30)
#         for waypoint in traj:
#             mc.send_angles(list(np.rad2deg(waypoint)), speed)
#             time.sleep(0.05)

#     # Reset pose
#     reset = [0, 0, 0, 0, 0, -45]
#     mc.send_angles(reset, speed)
#     mc.set_gripper_value(100, speed)
#     time.sleep(3)

#     # Grasp
#     move_to([276.80, -86.20, 120.30, -178.60, -3.45, -45.51])
#     mc.set_gripper_value(20, 100)
#     time.sleep(1)

#     # # Intermediate
#     # move_to([227.60,  48.60, 242.30, -173.16, -5.54, -24.07])
#     # time.sleep(1)

#     # # Release
#     # move_to([210.10, 195.70, 120.40, -179.68, -0.17, -18.98])
#     # mc.set_gripper_value(100, 100)
#     # time.sleep(1)

#     # # Return to reset and release servos
#     # mc.send_angles(reset, speed)
#     mc.release_all_servos()

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import time
from pymycobot.mycobot import MyCobot

def move_cartesian(mc, target, speed=50, tol_mm=2.0, timeout_s=5.0):
    """
    Moves the arm to `target` = [x,y,z, rx,ry,rz] using the firmware IK solver.
    Returns True if within tol_mm in timeout_s, False otherwise.
    """
    # 1) Read current joints (degrees)
    current_joints = mc.get_angles()

    # 2) Ask firmware to solve IK: returns a list of 6 angles (deg)
    ik_solution = mc.solve_inv_kinematics(target, current_joints)
    if not ik_solution or len(ik_solution) != 6:
        raise RuntimeError("solve_inv_kinematics failed or returned invalid data")

    # 3) Send those joint angles
    mc.send_angles(ik_solution, speed)

    # 4) (Optional) wait until in position
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        pos = mc.get_coords()[:3]
        err = ((pos[0]-target[0])**2 + (pos[1]-target[1])**2 + (pos[2]-target[2])**2)**0.5
        if err <= tol_mm:
            return True
        time.sleep(0.05)
    return False

if __name__ == "__main__":
    mc = MyCobot('/dev/ttyUSB0', 1000000)
    speed = 50

    # Define your Cartesian target [x, y, z, rx, ry, rz]
    target_pose = [276.8, -86.2, 120.3, -178.6, -3.45, -45.51]

    # Move there:
    success = move_cartesian(mc, target_pose, speed=speed)
    if not success:
        print("Warning: did not reach within tolerance")

    # Then do your grasp:
    mc.set_gripper_value(20, 100)
    time.sleep(1)
    mc.release_all_servos()

