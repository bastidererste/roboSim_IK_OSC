# roboSim

TCP-first robot simulation workflow:

- A Tool Center Point (TCP) target (`position + rotation`) is sent to the simulator via OSC.
- The simulator solves IK and moves the robot to that TCP pose.
- External tools (Ventuz, Unity, or similar) can feed/render this movement pipeline for an R3 robot workflow.
- For local testing, this repo includes a dedicated TCP sender tool.

## Demo

![roboSim demo](roboSim.gif)

## What This Setup Is

This repo contains two Python apps that work together:

- `osc_pybullet_arm_sim.py`: receives TCP goals over OSC (`pos/rot`), solves IK, and moves the robot in PyBullet.
- `osc_position_test_simulator.py`: TCP sender/testing tool for feeding goals (`/tcp/goal`, `/tcp/xyz`, `/tcp/rpy`, etc.) from CLI, REPL, or GUI backends (`tk`, `pygame`, `pybullet`).

Use case:

- Feed TCP targets from Ventuz/Unity/other OSC-capable systems.
- Validate TCP position/rotation behavior and IK response before final integration.
- Use the built-in sender to test without external tooling.

## How It Works

1. External source (Ventuz, Unity, or the included sender) sends TCP pose over OSC.
2. Simulator listens on `--in-port` (default `9000`) and receives `position + rotation`.
3. Simulator computes IK and drives the robot toward that TCP target.
4. Simulator streams state back on `--out-port` (default `9001`).

Motion details:

- Position: linear interpolation in XYZ at configured speed.
- Rotation: quaternion slerp interpolation.

## Coordinate System / Handedness

If your upstream tool (for example Ventuz) uses a different handedness, use axis-flip flags in the simulator:

- `--coord-flip-x`
- `--coord-flip-y`
- `--coord-flip-z`

For a typical Y-axis handedness mismatch, start the simulator with:

```bash
python osc_pybullet_arm_sim.py --in-port 9000 --out-port 9001 --coord-flip-y
```

These flips are applied consistently to:

- Incoming TCP position/orientation from OSC
- Outgoing EE and joint world XYZ OSC streams

## Requirements

- Python 3.10+ recommended
- `pybullet`
- `python-osc`
- `pygame` (optional, for `--backend pygame`)
- `tkinter` (optional, for `--backend tk`, depends on Python build)

## Install

Example with conda env `robosim`:

```bash
conda activate robosim
python -m pip install pybullet python-osc pygame
```

Quick verify:

```bash
python -c "import pybullet, pythonosc; print('ok')"
python -c "import pygame; print(pygame.__version__)"
```

## Quick Start

Run simulator (current recommended startup):

```bash
python osc_pybullet_arm_sim.py --in-port 9000 --out-port 9001 --osc-out-hz 30 --fast-gui --send-joint-world-pos --joint-output-deg
```

If you need Ventuz Y-axis handedness conversion, add:

```bash
python osc_pybullet_arm_sim.py --in-port 9000 --out-port 9001 --osc-out-hz 30 --fast-gui --send-joint-world-pos --joint-output-deg --coord-flip-y
```

In another terminal, send one goal:

```bash
python osc_position_test_simulator.py --sim-ip 127.0.0.1 --sim-port 9000 goal --x 0.45 --y 0.0 --z 0.35 --rpy 0 1.57 0 --speed 0.2
```

Open interactive GUI sender (recommended):

```bash
python osc_position_test_simulator.py --sim-ip 127.0.0.1 --sim-port 9000 gui --backend pygame --ui-hz 60 --status-hz 0 --quiet
```

## Sender Modes

- `goal`: send a single target from CLI args.
- `repl`: type targets in terminal.
- `gui`: interactive controls.

Global args (`--sim-ip`, `--sim-port`, `--listen`, etc.) must come before mode name.

Correct:

```bash
python osc_position_test_simulator.py --sim-ip 127.0.0.1 --sim-port 9000 gui
```

Not correct:

```bash
python osc_position_test_simulator.py gui --sim-ip 127.0.0.1 --sim-port 9000
```

## GUI Backends

- `--backend auto`: tries `tk`, then `pygame`, then `pybullet`.
- `--backend pygame`: fastest and most responsive in this project.
- `--backend tk`: compact slider panel if `_tkinter` is available.
- `--backend pybullet`: fallback slider UI inside PyBullet window.

Useful GUI flags:

- `--send-hz`: OSC send rate.
- `--ui-hz`: UI update rate.
- `--status-hz 0 --quiet`: reduce console spam.
- `--fast-gui`: reduce PyBullet visual overhead (only relevant to `pybullet` backend and simulator).

## OSC Input API (Simulator Receives)

All values are floats.

- `/tcp/xyz` -> `x y z`
- `/tcp/rpy` -> `roll pitch yaw` (radians)
- `/tcp/quat` -> `qx qy qz qw`
- `/tcp/speed` -> `speed_mps`
- `/tcp/goal` -> `x y z qx qy qz qw speed_mps`

`/tcp/goal` is the most complete single-message command.

## OSC Output API (Simulator Sends)

- `/joint/<name>` -> joint position (radians by default, degrees with `--joint-output-deg` for revolute joints)
- `/joint_vel/<name>` -> joint velocity (rad/s by default, deg/s with `--joint-output-deg` for revolute joints)
- `/joint_world/<name>/x|y|z` -> joint world position (when `--send-joint-world-pos`)
- `/ee/x`, `/ee/y`, `/ee/z` -> end-effector position (unless `--no-ee`)
- `/sim/t` -> unix timestamp

## Simulator Flags

Common options:

- `--robot-urdf kuka_iiwa/model.urdf`
- `--ee-link-index <index>` to override auto-detected EE link.
- `--default-speed <m/s>`
- `--hz <rate>`
- `--headless` for no GUI.
- `--fast-gui` for lighter rendering.
- `--joint-output-deg` to output revolute `/joint/*` and `/joint_vel/*` in degrees.

## Troubleshooting

- `ModuleNotFoundError: pybullet` or `pythonosc`:
  - Install packages into the same interpreter used to run scripts.
- `Tkinter is not available` / `_tkinter` missing:
  - Use `--backend pygame` or `--backend pybullet`.
- GUI closes immediately in `pybullet` backend:
  - Prefer `--backend pygame` for interactive control.
- Active conda env but wrong packages still used:
  - Check interpreter path with `python -c "import sys; print(sys.executable)"`.
  - Ensure it points inside your conda env.
- Sender runs but robot does not move:
  - Confirm simulator is running and listening on matching `--in-port`.
  - Confirm sender uses same `--sim-port`.

## Typical Workflow

1. Start simulator.
2. Start sender GUI (`pygame` backend).
3. Move `x/y/z/roll/pitch/yaw/speed` live.
4. Consume simulator OSC output in your external app on port `9001`.
