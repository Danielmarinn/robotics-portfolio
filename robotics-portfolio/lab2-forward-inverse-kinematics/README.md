# Lab 2 — Forward & Inverse Kinematics

DH-based forward kinematics and inverse kinematics (geometric and numerical) implemented for three different robot architectures. Highlights the challenge of IK and how solution strategies differ depending on robot topology.

---

## Robot Configurations

### `rrr-robot/` — 3-DOF Revolute Robot (RRR)
Full FK/IK for a 3R planar/spatial robot.

| File | Description |
|------|-------------|
| `MGD_DH.m` | DH matrix builder (single link transform) |
| `forward_kinematics.m` | Numeric FK — computes T_0^n and plots joint frames |
| `forward_kinematics_symbolic.m` | Symbolic FK using MATLAB `syms` |
| `dh_matrix.m` | Generic DH transform helper |
| `make_seriallink.m` | Builds the robot as a Robotics Toolbox `SerialLink` object |
| `inverse_kinematics.m` | Numerical IK using iterative Jacobian method |
| `fk_helper.m` | FK utility for use inside the IK loop |
| `pose_error.m` | Computes pose error (position + orientation) for IK convergence |

---

### `rr-robot/` — 2-DOF Planar Robot (RR)
Simpler 2-link planar robot, useful for validating the IK approach analytically.

| File | Description |
|------|-------------|
| `MGD_DH.m` | DH matrix builder |
| `forward_kinematics.m` | Numeric FK |
| `forward_kinematics_symbolic.m` | Symbolic FK |
| `inverse_kinematics.m` | Geometric IK (closed-form, elbow-up/down solutions) |

---

### `rpr-robot/` — Mixed Revolute-Prismatic Robot (RPR)
A robot with one prismatic joint — introduces non-uniform joint types.

| File | Description |
|------|-------------|
| `forward_kinematics.m` | DH-based FK |
| `inverse_kinematics.m` | Geometric IK adapted for mixed joint types |

---

## Key Concepts

| Concept | Description |
|--------|-------------|
| DH convention | Systematic parameterization of any serial chain |
| Geometric IK | Closed-form solutions exploiting robot geometry |
| Numerical IK | Iterative Jacobian pseudo-inverse approach |
| Pose error | 6D error vector (position + axis-angle orientation) |
| Elbow solutions | Multiple IK solutions for the same end-effector pose |

---

## Dependencies
- MATLAB
- [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/)
