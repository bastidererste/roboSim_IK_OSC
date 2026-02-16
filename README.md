# roboSim

Minimal PyBullet robot-arm simulator with two-way OSC, plus tools to send TCP goals from CLI, REPL, or GUI.

## What This Setup Is

This repo contains two Python apps that work together:

- `osc_pybullet_arm_sim.py`: runs a PyBullet robot arm, receives TCP goals over OSC, solves IK, and streams sim state back over OSC.
- `osc_position_test_simulator.py`: sends TCP goals to the simulator (`/tcp/goal`, `/tcp/xyz`, etc.) from command line, interactive REPL, or GUI backends (`tk`, `pygame`, or `pybullet`).

Use case:

- Control robot TCP position + orientation dynamically.
- Integrate with Unity/TouchDesigner/Max/MSP/other OSC-capable tools.
- Rapidly test pose targets and motion speed.

## How It Works

- Simulator listens for OSC input on `--in-port` (default `9000`).
- Sender tool sends goals to that port (`--sim-port`, default `9000`).
- Simulator streams state back on `--out-port` (default `9001`).
- Motion is linear in XYZ at configured speed, with quaternion slerp for orientation interpolation.

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

Run simulator:

```bash
python osc_pybullet_arm_sim.py --in-port 9000 --out-port 9001 --osc-out-hz 30 --fast-gui --send-joint-world-pos
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

- `/joint/<name>` -> joint position
- `/joint_vel/<name>` -> joint velocity (when `--send-joint-vel`)
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
