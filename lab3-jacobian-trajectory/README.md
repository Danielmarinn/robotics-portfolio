# Lab 3 — Jacobian Control & Circular Trajectory

Velocity-level control using the geometric Jacobian, applied to circular path following. Implemented in both **MATLAB simulation** (RRPR robot) and **Python on the real xArm Lite6** hardware.

---

## MATLAB — RRPR Robot (`matlab/`)

| File | Description |
|------|-------------|
| `IK_RRPR.m` | Closed-form inverse kinematics for the RRPR robot |
| `Jacobian_RRPR.m` | Geometric Jacobian of the RRPR robot |
| `circular_trajectory.m` | Circular trajectory using IK at via-points (Exercises 3 & 4) |
| `jacobian_control.m` | Jacobian-based velocity control for imposed `α̇` (Exercise 5) |

### Approach
The RRPR robot is tasked with tracing a circle of radius `r` centred at `(Cx, Cy)`. Two methods are compared:

1. **Via-point IK** — sample circle at discrete angles, solve IK at each, interpolate
2. **Jacobian control** — continuously compute `q̇ = J⁻¹ · ẋ` from the desired Cartesian velocity

The XY trajectory of the end-effector is compared between both methods, with error statistics printed.

---

## Python — xArm Lite6 (`python/`)

| File | Description |
|------|-------------|
| `xarm_circle_position.py` | Circle execution using built-in `set_position` (Cartesian position mode) |
| `xarm_circle_jacobian.py` | Circle execution using custom Jacobian + damped least-squares velocity control |

### `xarm_circle_position.py`
Simplest approach: send Cartesian poses directly. The robot's internal IK handles joint motion.

### `xarm_circle_jacobian.py`
Custom velocity controller running at ~50 Hz:
- Computes the geometric Jacobian from the DH model of the Lite6
- Uses **damped least-squares** pseudo-inverse: `J† = Jᵀ(JJᵀ + λ²I)⁻¹` to avoid singularities
- Sends joint velocities via `vc_set_joint_velocity` (mode 4)
- Logs executed TCP path and saves an XY plot

```
Circle parameters (defaults):
  centre: (250, 0, 300) mm
  radius: 30 mm
  orientation: roll=180°, pitch=0°, yaw=0°
```

---

## Key Concepts

| Concept | Description |
|--------|-------------|
| Geometric Jacobian | Maps joint velocities to end-effector linear + angular velocity |
| Damped least-squares | Numerically stable pseudo-inverse near singularities |
| Velocity control (mode 4) | Joint velocity commands at fixed rate (~50 Hz) |
| Axis-angle error | Orientation error representation for feedback control |

---

## Hardware Setup

- Robot: **UFactory xArm Lite6**
- Connection: Ethernet (set `ROBOT_IP` in each script)
- SDK: `xarm-python-sdk`

```bash
pip install xarm-python-sdk
python xarm_circle_jacobian.py
```

---

## Dependencies
- MATLAB + Robotics Toolbox (simulation)
- Python 3, NumPy, Matplotlib
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
