# Lab 4 — Cubic Spline Trajectory Planning

Multi-segment cubic polynomial trajectory planning in joint space, with velocity blending at via-points. Applied to a full circular path on the **RPR robot** (MATLAB simulation) and the **xArm Lite6** (Python, real hardware).

---

## MATLAB — RPR Robot (`matlab/`)

| File | Description |
|------|-------------|
| `IK_RPR.m` | Analytical inverse kinematics for the RPR robot |
| `Jacobian_RPR.m` | Geometric Jacobian of the RPR robot |
| `cubic_trajectory_planning.m` | Full cubic spline trajectory + Jacobian comparison |
| `jacobian_velocity_control.m` | Jacobian-based velocity control with imposed `α̇` |
| `parabolic_blend.m` | Parabolic blending segment helper |

### Cubic Trajectory Algorithm

Given via-points at times `t_k` with joint positions `q_k`:

1. **Boundary conditions:** zero velocity at start and end
2. **Interior velocities (heuristic):** average of adjacent finite differences; set to zero if sign changes (prevents overshoot)
3. **Cubic coefficients** per segment `[t_k, t_{k+1}]`:

```
q(t) = a₀ + a₁τ + a₂τ² + a₃τ³

a₀ = q₀
a₁ = v₀
a₂ = 3(q₁−q₀)/T² − (2v₀+v₁)/T
a₃ = −2(q₁−q₀)/T³ + (v₀+v₁)/T²
```

4. **Comparison:** the cubic trajectory end-effector path is plotted against the Jacobian velocity control path; mean and max XY deviation are printed.

---

## Python — xArm Lite6 (`python/`)

### `xarm_cubic_trajectory.py`

Full joint-space cubic trajectory executed on the real robot:

- **40 waypoints** sampled on a circle (radius 30 mm, centre at 250, 0, 300 mm)
- Waypoints converted to joint space via the **xArm SDK's internal IK**
- Cubic polynomial planned per segment with zero boundary velocities
- Executed in **joint velocity mode (mode 4)** at 50 Hz
- Executed TCP path logged via FK and saved as a PNG plot

```
Flow:
  home → move to circle start (mode 0) → cubic trajectory (mode 4) → home
```

**Key parameters (configurable at top of file):**
```python
ROBOT_IP       = "192.168.1.182"
DT             = 0.02          # control period (50 Hz)
QDOT_LIMIT_DEG = 60.0          # joint velocity limit
DAMPING        = 1e-3          # damped LS regularization
NUM_POINTS     = 40            # waypoints on circle
```

---

## Key Concepts

| Concept | Description |
|--------|-------------|
| Cubic polynomial | Smooth interpolation with position + velocity BCs |
| Via-point planning | Chained segments through intermediate configurations |
| Velocity heuristics | Automatic interior velocities, zero at sign changes |
| Parabolic blending | Alternative to cubic for trapezoidal velocity profiles |
| Mode 4 (xArm) | Joint velocity control mode for real-time streaming |

---

## Hardware Setup

- Robot: **UFactory xArm Lite6**
- Connection: Ethernet
- SDK: `xarm-python-sdk`

```bash
pip install xarm-python-sdk
python xarm_cubic_trajectory.py
```

Output: `trajectory_plot.png` — XY plot of the executed TCP path.

---

## Dependencies
- MATLAB + Robotics Toolbox (simulation)
- Python 3, NumPy, Matplotlib
- [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
