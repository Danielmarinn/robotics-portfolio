# robotics-portfolio AGENT GUIDE

## Purpose
- This repository is a public showcase of robotics coursework and real-robot control work.
- Priorities are correctness, clarity, and presentability.
- Treat this repo as a polished portfolio, not an experimental dump.

## Working Style
- Implement requested improvements directly.
- Prefer changes that make the repo cleaner, easier to understand, and safer to run.
- Keep explanations and README updates concise and professional.

## Project Facts
- MATLAB work is organized by lab folders.
- Python hardware-control scripts exist in:
  - `lab3-jacobian-trajectory\\python\\`
  - `lab4-cubic-trajectory\\python\\`
  - `lab4-trajectory-planning\\python\\`
- Real hardware target: UFactory xArm Lite6.

## Guardrails
- Never run real-robot scripts against hardware from a cloud environment.
- Treat xArm Python scripts as hardware-adjacent and potentially unsafe to execute remotely.
- For cloud work, prefer static analysis, code cleanup, documentation, and non-hardware-safe verification.
- Do not remove material that documents course progression unless explicitly asked.

## Validation
- After Python changes, run:
  - `python -m py_compile lab3-jacobian-trajectory\\python\\xarm_circle_jacobian.py lab3-jacobian-trajectory\\python\\xarm_circle_position.py lab4-cubic-trajectory\\python\\xarm_cubic_trajectory.py lab4-trajectory-planning\\python\\xarm_cubic_trajectory.py`
- If MATLAB is available locally, MATLAB validation is useful, but do not claim it ran unless it actually ran.
- For README or portfolio presentation edits, verify that the repo structure and links still match the actual files.

## Output Expectations
- Highlight:
  - what changed
  - whether verification was static only or included execution
  - any hardware-specific step Daniel must run locally before trusting the change

