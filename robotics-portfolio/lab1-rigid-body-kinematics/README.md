# Lab 1 — Rigid Body Kinematics

Foundation of spatial reasoning in robotics: representing and animating rigid body motion using homogeneous transformation matrices, and building the forward kinematics of a real 6-DOF robot from scratch.

---

## Scripts

### `rigid_body_transforms.m`
Animated 3D visualization of a rigid body (box) undergoing a sequence of transformations:

1. **Rotation 45°** about the World OY axis
2. **Rotation 60°** about the object's local `[1, 1, -1]` axis (Rodriguez formula)
3. **Translation +8 units** along the object's local OX axis
4. **World rotation −60°** about OZ

Key implementations:
- Rodriguez rotation formula (`rotation_axis`)
- Distinction between **world-frame** vs **body-frame** transformations
- Animated rendering with `drawnow` and frame updates

### `xarm_forward_kinematics.m`
Symbolic and animated forward kinematics for the **xArm Lite6** using the Denavit-Hartenberg convention.

- Full DH parameter table for the Lite6 (6 joints)
- Symbolic matrix chain: `T_0^i = A_1 · A_2 · ... · A_i`
- Interactive animation: move joint sliders, see the robot update in real-time

---

## Concepts

| Concept | Description |
|--------|-------------|
| Homogeneous matrix | 4×4 matrix encoding rotation + translation |
| Rodriguez formula | Rotation by angle `θ` about arbitrary axis `n̂` |
| DH parameters | `(θ, d, a, α)` — standard robot link parameterization |
| Forward kinematics | Mapping from joint angles → end-effector pose |

---

## Dependencies
- MATLAB
- [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/) (for `trplot`)
