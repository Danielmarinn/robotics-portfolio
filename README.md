# Robotics Engineering Portfolio

**Daniel Marin** — MSc Engineering Physics, University of Coimbra

This repository documents a complete robotics course covering rigid body kinematics, forward/inverse kinematics, Jacobian control, and trajectory planning — progressing from MATLAB simulation to real hardware control of a **UFactory xArm Lite6** 6-DOF robot arm.

---

## Repository Structure

| Lab | Topic | Tools |
|-----|-------|-------|
| [Lab 1](lab1-rigid-body-kinematics/) | Rigid Body Kinematics & DH Modelling | MATLAB |
| [Lab 2](lab2-forward-inverse-kinematics/) | Forward & Inverse Kinematics (RR, RRR, RPR) | MATLAB |
| [Lab 3](lab3-jacobian-trajectory/) | Jacobian Control & Circular Path Following | MATLAB + Python (real robot) |
| [Lab 4](lab4-trajectory-planning/) | Cubic Trajectory Planning in Joint Space | MATLAB + Python (real robot) |

---

## Skills Demonstrated

- Denavit-Hartenberg kinematic modelling from scratch
- Geometric and numerical inverse kinematics
- Real-time Jacobian-based velocity control on physical hardware at 50 Hz
- Trajectory planning (cubic polynomials, parabolic blends)
- Damped least-squares pseudo-inverse for singularity-robust control
- Python + NumPy for robot control; MATLAB for simulation and symbolic computation

---

## Hardware

- **Robot:** UFactory xArm Lite6 (6-DOF, 600 mm reach)
- **Connection:** Ethernet TCP/IP
- **SDK:** [xArm Python SDK](https://github.com/xArm-Developer/xArm-Python-SDK)
- **Control rate:** ~50 Hz (joint velocity mode 4)

---

## Other Projects

🤖 **MSc Thesis (in progress):** Applying G2ANet multi-agent reinforcement learning (MARL) to control a wastewater treatment plant simulated in BSM2/Simulink — 4 cooperative RL agents controlling aeration, recirculation, carbon dosing, and sludge wasting.

---

## Setup

```bash
# Python labs (real robot)
pip install xarm-python-sdk numpy matplotlib
python lab3-jacobian-trajectory/python/xarm_circle_jacobian.py

# MATLAB labs
# Requires MATLAB + Robotics Toolbox (Peter Corke)
# https://petercorke.com/toolboxes/robotics-toolbox/
```
