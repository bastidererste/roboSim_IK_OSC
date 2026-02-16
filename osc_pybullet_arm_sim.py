#!/usr/bin/env python3
"""Single-file PyBullet + OSC robotic arm simulator (Python 3.11)."""

from __future__ import annotations

import argparse
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import pybullet as p
import pybullet_data
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer
from pythonosc.udp_client import SimpleUDPClient

EPS = 1e-9


def decode_name(raw: object) -> str:
    if isinstance(raw, (bytes, bytearray)):
        return raw.decode("utf-8", errors="replace")
    return str(raw)


def sanitize_osc_segment(name: str) -> str:
    cleaned = []
    for ch in name:
        if ch.isalnum() or ch in ("_", "-"):
            cleaned.append(ch)
        else:
            cleaned.append("_")
    out = "".join(cleaned).strip("_")
    return out or "joint"


def vec_add(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_mul(a: tuple[float, float, float], s: float) -> tuple[float, float, float]:
    return (a[0] * s, a[1] * s, a[2] * s)


def vec_len(a: tuple[float, float, float]) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def normalize_quat(q: tuple[float, float, float, float]) -> Optional[tuple[float, float, float, float]]:
    if len(q) != 4:
        return None
    if not all(math.isfinite(v) for v in q):
        return None
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < EPS:
        return None
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_slerp(
    q0: tuple[float, float, float, float], q1: tuple[float, float, float, float], t: float
) -> tuple[float, float, float, float]:
    q0n = normalize_quat(q0) or (0.0, 0.0, 0.0, 1.0)
    q1n = normalize_quat(q1) or q0n

    dot = q0n[0] * q1n[0] + q0n[1] * q1n[1] + q0n[2] * q1n[2] + q0n[3] * q1n[3]
    if dot < 0.0:
        q1n = (-q1n[0], -q1n[1], -q1n[2], -q1n[3])
        dot = -dot

    dot = max(-1.0, min(1.0, dot))
    t = max(0.0, min(1.0, t))

    if dot > 0.9995:
        out = (
            q0n[0] + t * (q1n[0] - q0n[0]),
            q0n[1] + t * (q1n[1] - q0n[1]),
            q0n[2] + t * (q1n[2] - q0n[2]),
            q0n[3] + t * (q1n[3] - q0n[3]),
        )
        return normalize_quat(out) or q0n

    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * t
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0

    return (
        s0 * q0n[0] + s1 * q1n[0],
        s0 * q0n[1] + s1 * q1n[1],
        s0 * q0n[2] + s1 * q1n[2],
        s0 * q0n[3] + s1 * q1n[3],
    )


@dataclass
class MotionPlan:
    start_pos: tuple[float, float, float]
    start_orn: tuple[float, float, float, float]
    goal_pos: tuple[float, float, float]
    goal_orn: tuple[float, float, float, float]
    direction: tuple[float, float, float]
    dist_total: float
    traveled: float = 0.0


class SharedTargetState:
    def __init__(
        self,
        target_pos: tuple[float, float, float],
        target_orn: tuple[float, float, float, float],
        speed: float,
    ) -> None:
        self.lock = threading.Lock()
        self.target_pos = target_pos
        self.target_orn = target_orn
        self.speed = max(0.0, float(speed))
        self.new_goal = False


class OSCBridge:
    def __init__(self, shared: SharedTargetState, out_ip: str, out_port: int) -> None:
        self.shared = shared
        self.client = SimpleUDPClient(out_ip, out_port)
        self.dispatcher = Dispatcher()
        self.server: Optional[ThreadingOSCUDPServer] = None
        self.server_thread: Optional[threading.Thread] = None
        self._setup_dispatcher()

    def _setup_dispatcher(self) -> None:
        self.dispatcher.map("/tcp/xyz", self._on_tcp_xyz)
        self.dispatcher.map("/tcp/rpy", self._on_tcp_rpy)
        self.dispatcher.map("/tcp/quat", self._on_tcp_quat)
        self.dispatcher.map("/tcp/speed", self._on_tcp_speed)
        self.dispatcher.map("/tcp/goal", self._on_tcp_goal)

    @staticmethod
    def _parse_floats(values: tuple[object, ...], expected: int) -> Optional[list[float]]:
        if len(values) != expected:
            return None
        out: list[float] = []
        for v in values:
            try:
                fv = float(v)
            except (TypeError, ValueError):
                return None
            if not math.isfinite(fv):
                return None
            out.append(fv)
        return out

    def _on_tcp_xyz(self, _addr: str, *args: object) -> None:
        vals = self._parse_floats(args, 3)
        if vals is None:
            print("Invalid /tcp/xyz payload; expected 3 floats", flush=True)
            return
        with self.shared.lock:
            self.shared.target_pos = (vals[0], vals[1], vals[2])
            self.shared.new_goal = True

    def _on_tcp_rpy(self, _addr: str, *args: object) -> None:
        vals = self._parse_floats(args, 3)
        if vals is None:
            print("Invalid /tcp/rpy payload; expected 3 floats", flush=True)
            return
        quat = p.getQuaternionFromEuler((vals[0], vals[1], vals[2]))
        qn = normalize_quat((float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))
        if qn is None:
            print("Invalid quaternion generated from /tcp/rpy", flush=True)
            return
        with self.shared.lock:
            self.shared.target_orn = qn
            self.shared.new_goal = True

    def _on_tcp_quat(self, _addr: str, *args: object) -> None:
        vals = self._parse_floats(args, 4)
        if vals is None:
            print("Invalid /tcp/quat payload; expected 4 floats", flush=True)
            return
        qn = normalize_quat((vals[0], vals[1], vals[2], vals[3]))
        if qn is None:
            print("Rejected /tcp/quat due to zero or non-finite quaternion", flush=True)
            return
        with self.shared.lock:
            self.shared.target_orn = qn
            self.shared.new_goal = True

    def _on_tcp_speed(self, _addr: str, *args: object) -> None:
        vals = self._parse_floats(args, 1)
        if vals is None:
            print("Invalid /tcp/speed payload; expected 1 float", flush=True)
            return
        with self.shared.lock:
            self.shared.speed = max(0.0, vals[0])

    def _on_tcp_goal(self, _addr: str, *args: object) -> None:
        vals = self._parse_floats(args, 8)
        if vals is None:
            print("Invalid /tcp/goal payload; expected 8 floats", flush=True)
            return
        qn = normalize_quat((vals[3], vals[4], vals[5], vals[6]))
        if qn is None:
            print("Rejected /tcp/goal due to invalid quaternion", flush=True)
            return
        with self.shared.lock:
            self.shared.target_pos = (vals[0], vals[1], vals[2])
            self.shared.target_orn = qn
            self.shared.speed = max(0.0, vals[7])
            self.shared.new_goal = True

    def start_server(self, in_ip: str, in_port: int) -> None:
        self.server = ThreadingOSCUDPServer((in_ip, in_port), self.dispatcher)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

    def shutdown(self) -> None:
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        if self.server_thread is not None:
            self.server_thread.join(timeout=1.0)

    def consume_goal_snapshot(
        self,
    ) -> tuple[float, bool, tuple[float, float, float], tuple[float, float, float, float]]:
        with self.shared.lock:
            speed = self.shared.speed
            if self.shared.new_goal:
                self.shared.new_goal = False
                return speed, True, self.shared.target_pos, self.shared.target_orn
            return speed, False, self.shared.target_pos, self.shared.target_orn

    def send_outputs(
        self,
        robot_id: int,
        controllable_joints: list[int],
        joint_names: list[str],
        ee_link_index: int,
        emit_joint_vel: bool,
        emit_ee: bool,
        emit_joint_world_pos: bool,
    ) -> None:
        for idx, joint_index in enumerate(controllable_joints):
            js = p.getJointState(robot_id, joint_index)
            name = joint_names[idx]
            self.client.send_message(f"/joint/{name}", float(js[0]))
            if emit_joint_vel:
                self.client.send_message(f"/joint_vel/{name}", float(js[1]))
            if emit_joint_world_pos:
                ls = p.getLinkState(robot_id, joint_index, computeForwardKinematics=True)
                if ls is not None:
                    pos_raw = ls[4] if len(ls) > 4 and ls[4] is not None else ls[0]
                    self.client.send_message(f"/joint_world/{name}/x", float(pos_raw[0]))
                    self.client.send_message(f"/joint_world/{name}/y", float(pos_raw[1]))
                    self.client.send_message(f"/joint_world/{name}/z", float(pos_raw[2]))

        if emit_ee:
            ee_pos, _ = get_ee_pose(robot_id, ee_link_index)
            self.client.send_message("/ee/x", float(ee_pos[0]))
            self.client.send_message("/ee/y", float(ee_pos[1]))
            self.client.send_message("/ee/z", float(ee_pos[2]))

        self.client.send_message("/sim/t", float(time.time()))


def build_motion_plan(
    start_pos: tuple[float, float, float],
    start_orn: tuple[float, float, float, float],
    goal_pos: tuple[float, float, float],
    goal_orn: tuple[float, float, float, float],
) -> MotionPlan:
    delta = vec_sub(goal_pos, start_pos)
    dist = vec_len(delta)
    if dist <= EPS:
        direction = (0.0, 0.0, 0.0)
        dist = 0.0
    else:
        inv = 1.0 / dist
        direction = (delta[0] * inv, delta[1] * inv, delta[2] * inv)
    return MotionPlan(
        start_pos=start_pos,
        start_orn=normalize_quat(start_orn) or (0.0, 0.0, 0.0, 1.0),
        goal_pos=goal_pos,
        goal_orn=normalize_quat(goal_orn) or (0.0, 0.0, 0.0, 1.0),
        direction=direction,
        dist_total=dist,
        traveled=0.0,
    )


def advance_motion(plan: MotionPlan, speed_mps: float, dt: float) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    speed = max(0.0, speed_mps)

    # If you want acceleration limits later, this is where speed ramping can be inserted.
    if plan.dist_total > EPS and speed > 0.0:
        plan.traveled = min(plan.dist_total, plan.traveled + speed * dt)

    if plan.dist_total <= EPS:
        pos = plan.goal_pos
        u = 1.0
    else:
        pos = vec_add(plan.start_pos, vec_mul(plan.direction, plan.traveled))
        u = plan.traveled / plan.dist_total

    orn = quat_slerp(plan.start_orn, plan.goal_orn, u)
    return pos, orn


def get_ee_pose(robot_id: int, ee_link_index: int) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    ls = p.getLinkState(robot_id, ee_link_index, computeForwardKinematics=True)
    if ls is None:
        raise RuntimeError(f"getLinkState failed for link {ee_link_index}")

    pos_raw = ls[4] if len(ls) > 4 and ls[4] is not None else ls[0]
    orn_raw = ls[5] if len(ls) > 5 and ls[5] is not None else ls[1]

    pos = (float(pos_raw[0]), float(pos_raw[1]), float(pos_raw[2]))
    orn = normalize_quat((float(orn_raw[0]), float(orn_raw[1]), float(orn_raw[2]), float(orn_raw[3])))
    return pos, (orn or (0.0, 0.0, 0.0, 1.0))


def discover_controllable_joints(
    robot_id: int,
) -> tuple[list[int], list[str], list[float], list[float], list[float]]:
    joints: list[int] = []
    names: list[str] = []
    lowers: list[float] = []
    uppers: list[float] = []
    ranges: list[float] = []

    num_joints = p.getNumJoints(robot_id)
    for j in range(num_joints):
        info = p.getJointInfo(robot_id, j)
        joint_type = info[2]
        if joint_type not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            continue

        joints.append(j)
        names.append(sanitize_osc_segment(decode_name(info[1])))

        lo = float(info[8])
        hi = float(info[9])

        if lo > hi:
            if joint_type == p.JOINT_REVOLUTE:
                lo, hi = -2.0 * math.pi, 2.0 * math.pi
            else:
                lo, hi = -1.0, 1.0

        span = hi - lo
        if span <= EPS:
            span = 2.0 * math.pi if joint_type == p.JOINT_REVOLUTE else 1.0

        lowers.append(lo)
        uppers.append(hi)
        ranges.append(span)

    return joints, names, lowers, uppers, ranges


def resolve_ee_link_index(robot_id: int, controllable_joints: list[int], configured_index: Optional[int]) -> int:
    num_joints = p.getNumJoints(robot_id)

    if configured_index is not None:
        if configured_index < 0 or configured_index >= num_joints:
            raise ValueError(f"--ee-link-index {configured_index} out of range [0, {num_joints - 1}]")
        return configured_index

    if not controllable_joints:
        raise RuntimeError("No controllable joints found in robot URDF")

    name_hints = ("ee", "eef", "tool", "tcp", "gripper")
    for j in reversed(controllable_joints):
        info = p.getJointInfo(robot_id, j)
        joint_name = decode_name(info[1]).lower()
        link_name = decode_name(info[12]).lower()
        if any(h in joint_name or h in link_name for h in name_hints):
            return j

    return controllable_joints[-1]


def solve_ik(
    robot_id: int,
    ee_link_index: int,
    target_pos: tuple[float, float, float],
    target_orn: tuple[float, float, float, float],
    lower_limits: list[float],
    upper_limits: list[float],
    joint_ranges: list[float],
    rest_poses: list[float],
) -> tuple[float, ...]:
    try:
        return p.calculateInverseKinematics(
            robot_id,
            ee_link_index,
            target_pos,
            target_orn,
            lowerLimits=lower_limits,
            upperLimits=upper_limits,
            jointRanges=joint_ranges,
            restPoses=rest_poses,
            maxNumIterations=80,
            residualThreshold=1e-4,
        )
    except TypeError:
        # Fallback for older PyBullet APIs that do not accept null-space kwargs.
        return p.calculateInverseKinematics(
            robot_id,
            ee_link_index,
            target_pos,
            target_orn,
            maxNumIterations=80,
            residualThreshold=1e-4,
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="6/7-DOF robotic arm sim with two-way OSC")
    parser.add_argument("--in-ip", default="0.0.0.0", help="OSC listen IP (Unity -> Python)")
    parser.add_argument("--in-port", type=int, default=9000, help="OSC listen port (Unity -> Python)")
    parser.add_argument("--out-ip", default="127.0.0.1", help="OSC destination IP (Python -> Unity)")
    parser.add_argument("--out-port", type=int, default=9001, help="OSC destination port (Python -> Unity)")
    parser.add_argument("--hz", type=float, default=120.0, help="Simulation/control rate")
    parser.add_argument("--max-force", type=float, default=200.0, help="Joint motor max force")
    parser.add_argument("--default-speed", type=float, default=0.2, help="Default TCP linear speed (m/s)")
    parser.add_argument("--robot-urdf", default="kuka_iiwa/model.urdf", help="Robot URDF path in pybullet_data")
    parser.add_argument("--ee-link-index", type=int, default=None, help="Override end-effector link index")
    parser.add_argument("--headless", action="store_true", help="Run without PyBullet GUI")
    parser.add_argument(
        "--osc-out-hz",
        type=float,
        default=30.0,
        help="OSC output stream rate (Hz). Use <= 0 to stream every sim tick.",
    )
    parser.add_argument(
        "--fast-gui",
        action="store_true",
        help="Disable some GUI visual extras (shadows/widgets) for better rendering performance.",
    )
    parser.add_argument("--send-joint-vel", action="store_true", help="Also stream /joint_vel/<name>")
    parser.add_argument(
        "--send-joint-world-pos",
        action="store_true",
        help="Also stream world XYZ per controllable joint as /joint_world/<name>/(x|y|z).",
    )
    parser.add_argument("--no-ee", action="store_true", help="Disable /ee/x /ee/y /ee/z outputs")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.in_port == args.out_port:
        raise ValueError("Use different ports for OSC input and output")

    if args.hz <= 0:
        raise ValueError("--hz must be > 0")

    dt = 1.0 / args.hz

    connection_mode = p.DIRECT if args.headless else p.GUI
    client_id = p.connect(connection_mode)
    if client_id < 0:
        raise RuntimeError("Failed to connect to PyBullet")

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0.0, 0.0, -9.81)
    p.setRealTimeSimulation(0)
    p.setTimeStep(dt)
    if not args.headless and args.fast_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(args.robot_urdf, useFixedBase=True)

    controllable_joints, joint_names, lower_limits, upper_limits, joint_ranges = discover_controllable_joints(robot_id)
    if not controllable_joints:
        raise RuntimeError("No controllable (revolute/prismatic) joints found")

    ee_link_index = resolve_ee_link_index(robot_id, controllable_joints, args.ee_link_index)
    init_pos, init_orn = get_ee_pose(robot_id, ee_link_index)

    shared = SharedTargetState(init_pos, init_orn, args.default_speed)
    osc = OSCBridge(shared, args.out_ip, args.out_port)
    osc.start_server(args.in_ip, args.in_port)

    print(f"OSC IN : {args.in_ip}:{args.in_port}", flush=True)
    print(f"OSC OUT: {args.out_ip}:{args.out_port}", flush=True)
    print(f"Robot URDF: {args.robot_urdf}", flush=True)
    print(f"Controllable joints ({len(controllable_joints)}): {', '.join(joint_names)}", flush=True)
    print(f"EE link index: {ee_link_index}", flush=True)
    if args.osc_out_hz > 0.0:
        print(f"OSC output rate: {args.osc_out_hz:.2f} Hz", flush=True)
    else:
        print("OSC output rate: every sim tick", flush=True)

    motion_plan = build_motion_plan(init_pos, init_orn, init_pos, init_orn)
    emit_ee = not args.no_ee
    stream_every_tick = args.osc_out_hz <= 0.0
    osc_out_period = 0.0 if stream_every_tick else (1.0 / args.osc_out_hz)

    next_tick = time.perf_counter()
    next_osc_out = next_tick

    try:
        while True:
            speed, new_goal, goal_pos, goal_orn = osc.consume_goal_snapshot()

            if new_goal:
                current_ee_pos, current_ee_orn = get_ee_pose(robot_id, ee_link_index)
                motion_plan = build_motion_plan(current_ee_pos, current_ee_orn, goal_pos, goal_orn)

            target_pos, target_orn = advance_motion(motion_plan, speed, dt)
            current_joint_positions = [float(p.getJointState(robot_id, j)[0]) for j in controllable_joints]

            ik_solution = solve_ik(
                robot_id,
                ee_link_index,
                target_pos,
                target_orn,
                lower_limits,
                upper_limits,
                joint_ranges,
                current_joint_positions,
            )

            for i, joint_index in enumerate(controllable_joints):
                target = float(ik_solution[i]) if i < len(ik_solution) else current_joint_positions[i]
                p.setJointMotorControl2(
                    bodyUniqueId=robot_id,
                    jointIndex=joint_index,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target,
                    force=float(args.max_force),
                )

            p.stepSimulation()
            now = time.perf_counter()
            if stream_every_tick or now >= next_osc_out:
                osc.send_outputs(
                    robot_id=robot_id,
                    controllable_joints=controllable_joints,
                    joint_names=joint_names,
                    ee_link_index=ee_link_index,
                    emit_joint_vel=args.send_joint_vel,
                    emit_ee=emit_ee,
                    emit_joint_world_pos=args.send_joint_world_pos,
                )
                if not stream_every_tick:
                    while next_osc_out <= now:
                        next_osc_out += osc_out_period

            next_tick += dt
            now = time.perf_counter()
            sleep_s = next_tick - now
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = now

    except KeyboardInterrupt:
        print("Stopping simulation...", flush=True)
    finally:
        osc.shutdown()
        p.disconnect()


if __name__ == "__main__":
    main()
