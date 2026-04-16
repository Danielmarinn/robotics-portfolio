# Lab 4 — Joint-Space Trajectory Planning

Trajectory planning in joint space using cubic polynomials and parabolic blends, applied to both simulated (MATLAB) and real hardware (xArm Lite6 via Python).

---

## MATLAB — RPR Robot (`matlab/`)

| File | Description |
|------|-------------|
| `IK_RPR.m` | Closed-form inverse kinematics for the RPR robot |
| `J_RPR.m` | Geometric Jacobian of the RPR robot |
| `cubic_trajectory.m` | Cubic polynomial trajectory planner between joint waypoints |
| `parabolic_segment.m` | Parabolic blend segment helper for smooth velocity profiles |
| `trajectory_comparison.m` | Comparison of cubic vs parabolic blend trajectories |

### Approach

Joint-space trajectory planning between a sequence of waypoints:

1. **Cubic polynomials** — fit a degree-3 polynomial to each joint segment, matching position and velocity boundary conditions
2. **Parabolic blends** — linear segments with parabolic transitions at via-points for smoother acceleration profiles

---

## Python — xArm Lite6 (`python/`)

| File | Description |
|------|-------------|
| `xarm_cubic_trajectory.py` | Cubic joint-space trajectory for circular path on real xArm Lite6 |

### Approach

1. Sample a circle in task space at discrete angles
2. Use SDK IK to compute joint configurations at each waypoint
3. Plan cubic interpolation between consecutive joint configurations
4. Execute at ~50 Hz via joint velocity commands

```
Circle parameters (defaults):
  centre: configurable (mm)
  radius: 30 mm
  waypoints: evenly spaced around full circle
  control rate: 50 Hz
```

---

## Key Concepts

| Concept | Description |
|---------|-------------|
| Cubic trajectory | 3rd-order polynomial matching pos + vel at boundaries |
| Parabolic blend | Constant-acc transition regions for smooth via-points |
| Joint-space planning | Guarantees smooth joint motion; Cartesian path is approximate |
| Task-space planning | Direct Cartesian control; requires Jacobian or IK at each step |

---

## Hardware Setup

- Robot: **UFactory xArm Lite6**
- Connection: Ethernet (set `ROBOT_IP` in script)
- SDK: `xarm-python-sdk`

```bash
pip install xarm-python-sdk numpy matplotlib
python python/xarm_cubic_trajectory.py
```

---

## Dependencies
- MATLAB + Robotics Toolbox (simulation)
- Python 3, NumPy, Matplotlib
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
