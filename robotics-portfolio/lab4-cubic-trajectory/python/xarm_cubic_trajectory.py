#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Joint-space cubic trajectory for a full circular path in task space (Lite6).
Uses xArm SDK inverse kinematics to get joint waypoints on the circle,
plans cubic joint-space motion, logs executed TCP, and saves XY plot.
"""

import time
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless backend, OK for Docker
import matplotlib.pyplot as plt

from xarm.wrapper import XArmAPI
from xarm import version


# ===================== GLOBAL PARAMETERS =====================

ROBOT_IP = "192.168.1.182"

DT = 0.02  # control period (≈50 Hz)

QDOT_LIMIT_DEG = 60.0
QDOT_LIMIT_RAD = math.radians(QDOT_LIMIT_DEG)
DAMPING = 1e-3

MOVE_SPEED = 50.0
MOVE_ACC = 1000.0


# ===================== DH MODEL OF LITE6 =====================

# DH link parameters (mm, degrees)
LINK_A = np.array([0.0, 200.0, 87.0, 0.0, 0.0, 0.0])
LINK_D = np.array([243.3, 0.0, 0.0, 227.6, 0.0, 61.5])
LINK_ALPHA_DEG = np.array([-90.0, 180.0, 90.0, 90.0, -90.0, 0.0])
THETA_OFFSETS_DEG = np.array([0.0, -90.0, -90.0, 0.0, 0.0, 0.0])


# ===================== KINEMATIC FUNCTIONS =====================

def dh_transform(theta, alpha, a_i, d_i):
    """Standard DH homogeneous transform (radians, mm)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a_i * ct],
        [st,  ct * ca, -ct * sa, a_i * st],
        [0.0,     sa,      ca,      d_i],
        [0.0,   0.0,     0.0,      1.0],
    ])


def forward_kinematics(q_rad):
    """
    Full FK:
      q_rad: array(6) of joint angles in radians
      returns T06 and list of intermediate transforms.
    """
    alphas = np.deg2rad(LINK_ALPHA_DEG)
    th_off = np.deg2rad(THETA_OFFSETS_DEG)

    T = np.eye(4)
    frames = [T.copy()]

    for i in range(6):
        Ti = dh_transform(q_rad[i] + th_off[i], alphas[i], LINK_A[i], LINK_D[i])
        T = T @ Ti
        frames.append(T.copy())

    return T, frames


def compute_geometric_jacobian(q_rad):
    """Compute geometric Jacobian (6×6) from FK frames."""
    T06, frames = forward_kinematics(q_rad)
    o_n = T06[:3, 3]

    Jv = np.zeros((3, 6))
    Jw = np.zeros((3, 6))

    for i in range(6):
        z_axis = frames[i][:3, 2]
        origin = frames[i][:3, 3]
        Jv[:, i] = np.cross(z_axis, (o_n - origin))
        Jw[:, i] = z_axis

    return np.vstack((Jv, Jw))


def damped_inverse(J, lam=DAMPING):
    """Damped least-squares pseudo-inverse."""
    JJt = J @ J.T
    return J.T @ np.linalg.inv(JJt + (lam ** 2) * np.eye(JJt.shape[0]))


def orientation_error(R_des, R_current):
    """Axis-angle orientation error (3×1 vector)."""
    R_err = R_des @ R_current.T
    cos_ang = np.clip((np.trace(R_err) - 1) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_ang)

    if angle < 1e-9:
        return np.zeros(3)

    rx = R_err[2, 1] - R_err[1, 2]
    ry = R_err[0, 2] - R_err[2, 0]
    rz = R_err[1, 0] - R_err[0, 1]

    axis = (1 / (2 * math.sin(angle))) * np.array([rx, ry, rz])
    return axis * angle


# ===================== XARM SUPPORT UTILITIES =====================

def status_report(robot, label=""):
    """Print robot status snapshot."""
    if label:
        print(f"==== {label} ====")
    try:
        print("state      :", robot.state)
        print("mode       :", robot.mode)
        print("warn_code  :", robot.warn_code)
        print("error_code :", robot.error_code)
        print("angles(deg):", robot.angles)
    except Exception as e:
        print("[status_report] error reading state:", e)


def go_home(robot):
    """Move robot to factory home (mode 0)."""
    print("[home] Moving to factory home...")
    ret = robot.move_gohome(wait=True)
    status_report(robot, "After move_gohome")
    return ret == 0


def move_to_pose_cartesian(robot, x, y, z, roll_deg, pitch_deg, yaw_deg):
    """Move in position mode (mode 0) to a Cartesian pose (blocking)."""
    print(f"[move_to_pose] x={x:.1f}, y={y:.1f}, z={z:.1f}, rpy=({roll_deg},{pitch_deg},{yaw_deg})")
    ret = robot.set_position(
        x=x, y=y, z=z,
        roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
        speed=MOVE_SPEED, mvacc=MOVE_ACC,
        wait=True
    )
    status_report(robot, "After move_to_pose")
    return ret == 0


def stop_robot(robot):
    """Stop velocity control."""
    try:
        robot.vc_set_joint_velocity([0]*6, is_radian=True)
    except Exception:
        pass


# ===================== IK VIA SDK =====================

def inverse_kinematics_sdk(robot, x, y, z, roll, pitch, yaw, use_radian=False):
    """
    IK via xArm SDK:
      pose = [x(mm), y(mm), z(mm), roll, pitch, yaw]
    If use_radian=False: roll/pitch/yaw and output are in degrees.
    If use_radian=True:  roll/pitch/yaw and output are in radians.
    """
    pose = [x, y, z, roll, pitch, yaw]
    code, angles = robot.get_inverse_kinematics(
        pose,
        input_is_radian=use_radian,
        return_is_radian=use_radian
    )
    if code != 0 or not angles:
        print(f"[IK] get_inverse_kinematics failed, code={code}, pose={pose}")
        return None
    return np.array(angles[:6], dtype=float)


def poses_to_joint_list(robot, poses_xyzrpy):
    """
    Convert a list of poses [X,Y,Z,R,P,Y] (mm, degrees)
    to a list of joint vectors (radians), using SDK IK.
    """
    q_list = []
    for pose in poses_xyzrpy:
        x, y, z, roll_deg, pitch_deg, yaw_deg = pose

        q_deg = inverse_kinematics_sdk(
            robot,
            x, y, z,
            roll_deg, pitch_deg, yaw_deg,
            use_radian=False
        )
        if q_deg is None:
            raise RuntimeError(f"IK failed for pose {pose}")

        q_rad = np.deg2rad(q_deg)
        q_list.append(q_rad)
    return q_list


# ===================== CUBIC TRAJECTORY UTILITIES =====================

def cubic_coefficients(q0, qf, v0, vf, T):
    """
    q(t) = a0 + a1 t + a2 t^2 + a3 t^3
    with q(0)=q0, q(T)=qf, q̇(0)=v0, q̇(T)=vf.
    """
    a0 = q0
    a1 = v0
    a2 = (3.0*(qf - q0) / (T**2)) - (2.0*v0 + vf) / T
    a3 = (2.0*(q0 - qf) / (T**3)) + (v0 + vf) / (T**2)
    return a0, a1, a2, a3


def sample_cubic_segment(q_start, q_end, v_start, v_end, T_seg, dt):
    """
    Generate samples of a cubic trajectory between two joint vectors.
    q_start, q_end, v_start, v_end are 6×1 (radians).
    """
    q_start = np.asarray(q_start, dtype=float)
    q_end   = np.asarray(q_end, dtype=float)
    v_start = np.asarray(v_start, dtype=float)
    v_end   = np.asarray(v_end, dtype=float)

    steps = int(T_seg / dt) + 1
    times = np.linspace(0.0, T_seg, steps)

    coeffs = []
    for j in range(6):
        a0, a1, a2, a3 = cubic_coefficients(
            q_start[j], q_end[j], v_start[j], v_end[j], T_seg
        )
        coeffs.append((a0, a1, a2, a3))

    q_traj = []
    qdot_traj = []
    for t in times:
        q = np.zeros(6)
        qdot = np.zeros(6)
        for j in range(6):
            a0, a1, a2, a3 = coeffs[j]
            q[j] = a0 + a1*t + a2*(t**2) + a3*(t**3)
            qdot[j] = a1 + 2*a2*t + 3*a3*(t**2)
        q_traj.append(q)
        qdot_traj.append(qdot)
    return q_traj, qdot_traj


# ===================== CIRCLE GENERATION =====================

def generate_circle_poses_xy(
    center_x, center_y, center_z,
    radius,
    roll_deg, pitch_deg, yaw_deg,
    num_points
):
    """
    Generate Cartesian poses on a full circle in the XY plane.
    Returns a list of [X,Y,Z,R,P,Y] in mm, degrees.
    """
    poses = []
    for k in range(num_points):
        theta = 2.0 * math.pi * k / num_points  # 0 -> 2pi
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        z = center_z
        poses.append([x, y, z, roll_deg, pitch_deg, yaw_deg])
    return poses


# ===================== PLOTTING =====================

def plot_and_save_tcp_path(tcp_log, filename="trajectory_plot.png"):
    """
    Plot executed TCP path in the XY plane and save to PNG.
    """
    if not tcp_log:
        print("[PLOT] No TCP data to plot.")
        return

    tcp_arr = np.array(tcp_log)  # shape (N, 3)
    x = tcp_arr[:, 0]
    y = tcp_arr[:, 1]

    plt.figure(figsize=(6, 6))
    plt.plot(x, y, 'b-')
    plt.xlabel('X (mm)')
    plt.ylabel('Y (mm)')
    plt.title('Executed TCP path in XY')
    plt.axis('equal')
    plt.grid(True)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    plt.close()
    print(f"[PLOT] Saved executed TCP XY trajectory plot to {filename}")


# ===================== JOINT-SPACE CUBIC PLANNER =====================

def plan_and_execute_cubic_joint_trajectory(
    robot,
    pose_A,
    pose_B,
    via_poses=None,
    total_time=6.0,
    dt=DT
):
    """
    Plan and execute a joint-space cubic trajectory from pose A to B with via poses.
    pose_A, pose_B, via_poses in [X,Y,Z,R,P,Y] (mm, degrees).
    """
    if via_poses is None:
        via_poses = []

    pose_list = [pose_A] + list(via_poses) + [pose_B]
    num_waypoints = len(pose_list)
    if num_waypoints < 2:
        raise ValueError("Need at least pose_A and pose_B")

    # 1) IK: poses -> joints (radians)
    q_list = poses_to_joint_list(robot, pose_list)

    num_segments = num_waypoints - 1
    T_seg = total_time / num_segments

    # zero velocity at all waypoints (independent cubic per segment)
    v_list = [np.zeros(6) for _ in range(num_waypoints)]

    # 2) Switch to joint velocity mode (4) for the whole trajectory
    print("\n[CUBIC] Switching to joint velocity mode (4)...")
    robot.set_mode(4)
    robot.set_state(0)
    time.sleep(0.2)
    status_report(robot, "Before cubic trajectory execution")

    if robot.error_code != 0:
        print("[CUBIC] Robot has error before execution.")
        return

    tcp_log = []  # executed TCP path
    start_time = time.time()
    step_index = 0

    for i in range(num_segments):
        q_start = q_list[i]
        q_end   = q_list[i+1]
        v_s     = v_list[i]
        v_f     = v_list[i+1]

        q_traj, qdot_traj = sample_cubic_segment(q_start, q_end, v_s, v_f, T_seg, dt)
        steps_seg = len(q_traj)

        for k in range(steps_seg):
            if robot.error_code != 0:
                print("[CUBIC] error_code != 0 during execution.")
                status_report(robot, "State at abort")
                stop_robot(robot)
                plot_and_save_tcp_path(tcp_log)
                return

            qdot_cmd = qdot_traj[k]
            qdot_cmd = np.clip(qdot_cmd, -QDOT_LIMIT_RAD, QDOT_LIMIT_RAD)
            qdot_deg = np.rad2deg(qdot_cmd)

            ret = robot.vc_set_joint_velocity(qdot_deg.tolist(), is_sync=True)
            if ret != 0:
                print(f"[CUBIC] Velocity command failed at seg {i}, step {k}, ret={ret}")
                status_report(robot, "Velocity command failure")
                stop_robot(robot)
                plot_and_save_tcp_path(tcp_log)
                return

            # log executed TCP from current joint angles
            q_deg_now = robot.angles
            if q_deg_now is not None and len(q_deg_now) >= 6:
                q_rad_now = np.deg2rad(np.array(q_deg_now[:6]))
                T_now, _ = forward_kinematics(q_rad_now)
                p_now = T_now[:3, 3]
                tcp_log.append(p_now.copy())

            # timing
            t_target = start_time + step_index * dt
            remain = t_target - time.time()
            if remain > 0:
                time.sleep(remain)
            step_index += 1

    stop_robot(robot)
    print("[CUBIC] Trajectory execution finished.")
    plot_and_save_tcp_path(tcp_log)


# =========================== MAIN ===========================

def main():
    print("SDK_VERSION:", version.__version__)
    robot = XArmAPI(ROBOT_IP)
    status_report(robot, "After XArmAPI init")

    if robot.error_code != 0:
        print("[FATAL] Robot has errors on startup.")
        robot.disconnect()
        return

    # mode 0 for position moves
    print("[init] motion_enable, mode=0, state=0")
    robot.motion_enable(True)
    robot.set_mode(0)
    robot.set_state(0)
    time.sleep(0.5)
    status_report(robot, "After init")

    if robot.error_code != 0:
        print("[FATAL] error_code != 0 after initialization.")
        robot.disconnect()
        return

    # Go to home in mode 0
    if not go_home(robot):
        print("[FATAL] Could not reach home.")
        robot.disconnect()
        return

    # Circle parameters similar to labwork 3
    CENTER_X, CENTER_Y, CENTER_Z = 250.0, 0.0, 300.0
    RADIUS = 30.0
    ROLL, PITCH, YAW = 180.0, 0.0, 0.0

    # Number of waypoints on the circle
    NUM_POINTS = 40

    circle_poses = generate_circle_poses_xy(
        CENTER_X, CENTER_Y, CENTER_Z,
        RADIUS,
        ROLL, PITCH, YAW,
        num_points=NUM_POINTS
    )

    pose_A = circle_poses[0]
    pose_B = circle_poses[-1]
    via_poses = circle_poses[1:-1]

    # Move to starting pose A in mode 0
    if not move_to_pose_cartesian(robot, *pose_A):
        print("[FATAL] Could not move to pose A.")
        robot.disconnect()
        return

    # Plan and execute cubic joint-space trajectory around the full circle
    try:
        plan_and_execute_cubic_joint_trajectory(
            robot,
            pose_A,
            pose_B,
            via_poses=via_poses,
            total_time=12.0,   # increase if needed for smoother motion
            dt=DT
        )
    except Exception as exc:
        print("[EXCEPTION] Error during cubic trajectory:", exc)
        status_report(robot, "After exception")

    # Back to home in mode 0
    print("\n[home] Returning to factory home...")
    try:
        robot.set_mode(0)
        robot.set_state(0)
        time.sleep(0.2)
        go_home(robot)
        status_report(robot, "After home move")
    except Exception as exc:
        print("[WARNING] Error returning home:", exc)

    print("[cleanup] Disconnecting robot...")
    try:
        robot.set_state(4)
    except Exception:
        pass
    robot.disconnect()


if __name__ == "__main__":
    main()
