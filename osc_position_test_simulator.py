#!/usr/bin/env python3
"""Command-line OSC tool to test TCP position goals for osc_pybullet_arm_sim.py."""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time
from typing import Optional

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import ThreadingOSCUDPServer
from pythonosc.udp_client import SimpleUDPClient

EPS = 1e-9


def normalize_quat(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < EPS:
        raise ValueError("Quaternion norm is zero")
    return (qx / n, qy / n, qz / n, qw / n)


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return normalize_quat(qx, qy, qz, qw)


def start_listener(ip: str, port: int) -> tuple[ThreadingOSCUDPServer, threading.Thread]:
    dispatcher = Dispatcher()

    def _default_handler(address: str, *args: object) -> None:
        print(f"[OSC OUT] {address}: {args}")

    dispatcher.set_default_handler(_default_handler)
    server = ThreadingOSCUDPServer((ip, port), dispatcher)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server, thread


def send_goal(
    client: SimpleUDPClient,
    x: float,
    y: float,
    z: float,
    quat: tuple[float, float, float, float],
    speed: float,
    repeat: int,
    interval: float,
) -> None:
    payload = [x, y, z, quat[0], quat[1], quat[2], quat[3], max(0.0, speed)]
    for _ in range(max(1, repeat)):
        client.send_message("/tcp/goal", payload)
        if interval > 0.0:
            time.sleep(interval)


def run_goal_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    if args.quat is not None:
        quat = normalize_quat(args.quat[0], args.quat[1], args.quat[2], args.quat[3])
    else:
        quat = quat_from_rpy(args.roll, args.pitch, args.yaw)

    send_goal(
        client=client,
        x=args.x,
        y=args.y,
        z=args.z,
        quat=quat,
        speed=args.speed,
        repeat=args.repeat,
        interval=args.interval,
    )
    print(
        "Sent /tcp/goal:",
        f"x={args.x:.4f} y={args.y:.4f} z={args.z:.4f}",
        f"q=({quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f})",
        f"speed={max(0.0, args.speed):.4f} m/s",
    )


def run_repl_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    print("Interactive mode.")
    print("Enter: x y z speed")
    print("Or   : x y z roll pitch yaw speed")
    print("Type 'q' or 'quit' to exit.")

    while True:
        try:
            line = input("goal> ").strip()
        except EOFError:
            print()
            return
        except KeyboardInterrupt:
            print()
            return

        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            return

        parts = line.split()
        try:
            vals = [float(v) for v in parts]
        except ValueError:
            print("Invalid input: expected numeric values.")
            continue

        if len(vals) == 4:
            x, y, z, speed = vals
            quat = quat_from_rpy(0.0, 0.0, 0.0)
        elif len(vals) == 7:
            x, y, z, roll, pitch, yaw, speed = vals
            quat = quat_from_rpy(roll, pitch, yaw)
        else:
            print("Use 4 values (x y z speed) or 7 values (x y z roll pitch yaw speed).")
            continue

        send_goal(
            client=client,
            x=x,
            y=y,
            z=z,
            quat=quat,
            speed=speed,
            repeat=args.repeat,
            interval=args.interval,
        )
        print(f"Sent goal x={x:.4f} y={y:.4f} z={z:.4f} speed={max(0.0, speed):.4f}")


def run_tk_gui_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    try:
        import tkinter as tk
        from tkinter import ttk
    except ImportError as exc:
        raise ValueError("Tkinter is not available in this Python environment") from exc

    send_hz = max(1.0, float(args.send_hz))
    send_interval_ms = max(10, int(round(1000.0 / send_hz)))
    status_interval_ms = max(100, int(round(1000.0 / 3.0)))

    root = tk.Tk()
    root.title("OSC TCP Goal Controller")
    root.geometry("460x420")

    main = ttk.Frame(root, padding=12)
    main.pack(fill=tk.BOTH, expand=True)
    main.columnconfigure(1, weight=1)
    main.columnconfigure(2, minsize=72)

    vars_by_name = {
        "x": tk.DoubleVar(value=args.x),
        "y": tk.DoubleVar(value=args.y),
        "z": tk.DoubleVar(value=args.z),
        "roll": tk.DoubleVar(value=args.roll),
        "pitch": tk.DoubleVar(value=args.pitch),
        "yaw": tk.DoubleVar(value=args.yaw),
        "speed": tk.DoubleVar(value=args.speed),
    }
    value_labels: dict[str, ttk.Label] = {}

    slider_specs = [
        ("x", "X (m)", args.x_min, args.x_max, 0.001),
        ("y", "Y (m)", args.y_min, args.y_max, 0.001),
        ("z", "Z (m)", args.z_min, args.z_max, 0.001),
        ("roll", "Roll (rad)", -math.pi, math.pi, 0.001),
        ("pitch", "Pitch (rad)", -math.pi, math.pi, 0.001),
        ("yaw", "Yaw (rad)", -math.pi, math.pi, 0.001),
        ("speed", "Speed (m/s)", 0.0, args.speed_max, 0.001),
    ]

    def _format_value(v: float) -> str:
        return f"{v:+.3f}"

    def _refresh_value_labels() -> None:
        for key, lbl in value_labels.items():
            lbl.configure(text=_format_value(vars_by_name[key].get()))

    for row, (key, label, min_v, max_v, step) in enumerate(slider_specs):
        ttk.Label(main, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=4)
        scale = tk.Scale(
            main,
            from_=min_v,
            to=max_v,
            orient=tk.HORIZONTAL,
            showvalue=False,
            resolution=step,
            variable=vars_by_name[key],
            length=250,
            command=lambda _v, k=key: value_labels[k].configure(text=_format_value(vars_by_name[k].get())),
        )
        scale.grid(row=row, column=1, sticky="ew", pady=4)
        val_label = ttk.Label(main, text=_format_value(vars_by_name[key].get()))
        val_label.grid(row=row, column=2, sticky="e", pady=4)
        value_labels[key] = val_label

    status_var = tk.StringVar(value=f"Streaming to {args.sim_ip}:{args.sim_port} at {send_hz:.1f} Hz")
    ttk.Separator(main, orient=tk.HORIZONTAL).grid(row=len(slider_specs), column=0, columnspan=3, sticky="ew", pady=(8, 8))
    ttk.Label(main, textvariable=status_var).grid(
        row=len(slider_specs) + 1,
        column=0,
        columnspan=3,
        sticky="w",
    )

    button_row = ttk.Frame(main)
    button_row.grid(row=len(slider_specs) + 2, column=0, columnspan=3, sticky="w", pady=(10, 0))

    send_count = {"n": 0}
    last_status = {"t": 0.0}

    def _build_payload() -> list[float]:
        roll = vars_by_name["roll"].get()
        pitch = vars_by_name["pitch"].get()
        yaw = vars_by_name["yaw"].get()
        quat = quat_from_rpy(roll, pitch, yaw)
        return [
            vars_by_name["x"].get(),
            vars_by_name["y"].get(),
            vars_by_name["z"].get(),
            quat[0],
            quat[1],
            quat[2],
            quat[3],
            max(0.0, vars_by_name["speed"].get()),
        ]

    def _send_once() -> None:
        payload = _build_payload()
        client.send_message("/tcp/goal", payload)
        send_count["n"] += 1
        now = time.time()
        if (now - last_status["t"]) * 1000.0 >= status_interval_ms:
            status_var.set(
                f"Sent #{send_count['n']}  xyz=({payload[0]:+.3f}, {payload[1]:+.3f}, {payload[2]:+.3f})  "
                f"speed={payload[7]:.3f} m/s"
            )
            last_status["t"] = now

    def _tick() -> None:
        _send_once()
        root.after(send_interval_ms, _tick)

    def _zero_rpy() -> None:
        vars_by_name["roll"].set(0.0)
        vars_by_name["pitch"].set(0.0)
        vars_by_name["yaw"].set(0.0)
        _refresh_value_labels()
        _send_once()

    ttk.Button(button_row, text="Send now", command=_send_once).pack(side=tk.LEFT)
    ttk.Button(button_row, text="Zero RPY", command=_zero_rpy).pack(side=tk.LEFT, padx=8)
    ttk.Button(button_row, text="Quit", command=root.destroy).pack(side=tk.LEFT)

    _refresh_value_labels()
    _send_once()
    root.after(send_interval_ms, _tick)
    root.mainloop()


def run_pygame_gui_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    try:
        import pygame
    except ImportError as exc:
        raise ValueError("Pygame backend is not available in this Python environment (install pygame)") from exc

    send_hz = max(1.0, float(args.send_hz))
    ui_hz = max(10.0, float(args.ui_hz))
    send_interval = 1.0 / send_hz
    status_interval = 0.0 if args.status_hz <= 0.0 else (1.0 / float(args.status_hz))

    class Slider:
        def __init__(self, name: str, label: str, min_value: float, max_value: float, value: float, y: int) -> None:
            self.name = name
            self.label = label
            self.min_value = min_value
            self.max_value = max_value
            self.value = min(max(value, min_value), max_value)
            self.track_rect = pygame.Rect(210, y + 16, 420, 8)
            self.dragging = False

        def _norm(self) -> float:
            span = self.max_value - self.min_value
            if abs(span) < EPS:
                return 0.0
            return (self.value - self.min_value) / span

        def _x_to_value(self, x: float) -> float:
            clamped_x = min(max(x, float(self.track_rect.left)), float(self.track_rect.right))
            t = (clamped_x - float(self.track_rect.left)) / float(self.track_rect.width)
            return self.min_value + (self.max_value - self.min_value) * t

        def knob_pos(self) -> tuple[int, int]:
            x = int(round(self.track_rect.left + self._norm() * self.track_rect.width))
            y = self.track_rect.centery
            return x, y

        def set_value(self, value: float) -> None:
            self.value = min(max(value, self.min_value), self.max_value)

        def handle_event(self, event: pygame.event.Event) -> bool:
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                knob_x, knob_y = self.knob_pos()
                dx = float(event.pos[0] - knob_x)
                dy = float(event.pos[1] - knob_y)
                if self.track_rect.inflate(0, 18).collidepoint(event.pos) or (dx * dx + dy * dy) <= 144.0:
                    self.dragging = True
                    self.set_value(self._x_to_value(float(event.pos[0])))
                    return True
            if event.type == pygame.MOUSEMOTION and self.dragging:
                self.set_value(self._x_to_value(float(event.pos[0])))
                return True
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1 and self.dragging:
                self.dragging = False
                return True
            return False

        def draw(self, surface: pygame.Surface, font: pygame.font.Font, selected: bool) -> None:
            label_color = (236, 239, 244) if selected else (192, 199, 214)
            track_color = (96, 105, 126)
            fill_color = (99, 179, 237) if selected else (76, 139, 190)
            knob_color = (161, 214, 255) if selected else (122, 178, 222)

            pygame.draw.rect(surface, track_color, self.track_rect, border_radius=4)
            fill_width = int(round(self._norm() * self.track_rect.width))
            if fill_width > 0:
                filled_rect = pygame.Rect(self.track_rect.left, self.track_rect.top, fill_width, self.track_rect.height)
                pygame.draw.rect(surface, fill_color, filled_rect, border_radius=4)
            knob_x, knob_y = self.knob_pos()
            pygame.draw.circle(surface, knob_color, (knob_x, knob_y), 9)

            label_surf = font.render(self.label, True, label_color)
            value_surf = font.render(f"{self.value:+.3f}", True, label_color)
            surface.blit(label_surf, (32, self.track_rect.y - 2))
            surface.blit(value_surf, (650, self.track_rect.y - 2))

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("OSC TCP Goal Controller (pygame)")
    screen = pygame.display.set_mode((760, 470))
    clock = pygame.time.Clock()
    title_font = pygame.font.SysFont("Menlo", 20)
    font = pygame.font.SysFont("Menlo", 16)
    small_font = pygame.font.SysFont("Menlo", 14)

    slider_specs = [
        ("x", "X (m)", args.x_min, args.x_max, args.x),
        ("y", "Y (m)", args.y_min, args.y_max, args.y),
        ("z", "Z (m)", args.z_min, args.z_max, args.z),
        ("roll", "Roll (rad)", -math.pi, math.pi, args.roll),
        ("pitch", "Pitch (rad)", -math.pi, math.pi, args.pitch),
        ("yaw", "Yaw (rad)", -math.pi, math.pi, args.yaw),
        ("speed", "Speed (m/s)", 0.0, args.speed_max, args.speed),
    ]
    sliders: list[Slider] = [Slider(name, label, min_v, max_v, value, 78 + i * 45) for i, (name, label, min_v, max_v, value) in enumerate(slider_specs)]
    sliders_by_name = {s.name: s for s in sliders}
    selected_index = 0

    if not args.quiet:
        print("Pygame GUI target controller running. Close the window to stop.")
        print(f"Streaming /tcp/goal to {args.sim_ip}:{args.sim_port} at {send_hz:.1f} Hz")

    send_count = 0
    last_send = 0.0
    last_status = 0.0
    running = True
    try:
        while running:
            force_send = False
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    break
                if event.type == pygame.KEYDOWN:
                    if event.key in (pygame.K_ESCAPE, pygame.K_q):
                        running = False
                        break
                    if event.key == pygame.K_r:
                        sliders_by_name["roll"].set_value(0.0)
                        sliders_by_name["pitch"].set_value(0.0)
                        sliders_by_name["yaw"].set_value(0.0)
                        force_send = True
                    elif event.key == pygame.K_TAB:
                        selected_index = (selected_index + 1) % len(sliders)
                    elif pygame.K_1 <= event.key <= pygame.K_7:
                        selected_index = min(len(sliders) - 1, event.key - pygame.K_1)
                    elif event.key in (pygame.K_LEFT, pygame.K_RIGHT, pygame.K_DOWN, pygame.K_UP):
                        direction = 1.0 if event.key in (pygame.K_RIGHT, pygame.K_UP) else -1.0
                        current = sliders[selected_index]
                        step_scale = 0.01 if (event.mod & pygame.KMOD_SHIFT) else 0.002
                        step = max(1e-4, (current.max_value - current.min_value) * step_scale)
                        current.set_value(current.value + direction * step)
                        force_send = True
                    elif event.key == pygame.K_SPACE:
                        force_send = True

                for idx, slider in enumerate(sliders):
                    changed = slider.handle_event(event)
                    if changed:
                        selected_index = idx
                        force_send = True

            now = time.time()
            if force_send or (now - last_send >= send_interval):
                x = sliders_by_name["x"].value
                y = sliders_by_name["y"].value
                z = sliders_by_name["z"].value
                roll = sliders_by_name["roll"].value
                pitch = sliders_by_name["pitch"].value
                yaw = sliders_by_name["yaw"].value
                speed = max(0.0, sliders_by_name["speed"].value)
                quat = quat_from_rpy(roll, pitch, yaw)
                payload = [x, y, z, quat[0], quat[1], quat[2], quat[3], speed]
                client.send_message("/tcp/goal", payload)
                send_count += 1
                last_send = now

                if (not args.quiet) and status_interval > 0.0 and (now - last_status >= status_interval):
                    print(
                        f"Sent #{send_count} xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}) "
                        f"rpy=({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}) speed={speed:.3f}",
                        flush=True,
                    )
                    last_status = now

            screen.fill((17, 21, 30))
            title = title_font.render("TCP Goal Sender (pygame)", True, (238, 243, 252))
            subtitle = small_font.render(
                "Drag sliders or use keys 1-7 + arrows, Shift=faster, R=zero RPY, Space=send, Esc=quit",
                True,
                (148, 161, 185),
            )
            screen.blit(title, (24, 20))
            screen.blit(subtitle, (24, 48))

            for idx, slider in enumerate(sliders):
                slider.draw(screen, font, idx == selected_index)

            roll = sliders_by_name["roll"].value
            pitch = sliders_by_name["pitch"].value
            yaw = sliders_by_name["yaw"].value
            quat = quat_from_rpy(roll, pitch, yaw)
            footer = small_font.render(
                f"q=({quat[0]:+.3f}, {quat[1]:+.3f}, {quat[2]:+.3f}, {quat[3]:+.3f})   "
                f"send_hz={send_hz:.1f}   ui_hz={ui_hz:.1f}   sent={send_count}",
                True,
                (167, 180, 204),
            )
            screen.blit(footer, (24, 430))

            pygame.display.flip()
            clock.tick(ui_hz)
    finally:
        pygame.quit()


def run_pybullet_gui_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    try:
        import pybullet as p
        import pybullet_data
    except ImportError as exc:
        raise ValueError("PyBullet GUI backend is not available in this Python environment") from exc

    send_hz = max(1.0, float(args.send_hz))
    ui_hz = max(5.0, float(args.ui_hz))
    send_interval = 1.0 / send_hz
    ui_interval = 1.0 / ui_hz
    status_interval = 0.0 if args.status_hz <= 0.0 else (1.0 / float(args.status_hz))

    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise ValueError("Failed to open PyBullet GUI window")

    try:
        if args.fast_gui:
            # Keep GUI widgets enabled; this mode depends on user debug sliders.
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            if hasattr(p, "COV_ENABLE_RGB_BUFFER_PREVIEW"):
                p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            if hasattr(p, "COV_ENABLE_DEPTH_BUFFER_PREVIEW"):
                p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            if hasattr(p, "COV_ENABLE_SEGMENTATION_MARK_PREVIEW"):
                p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            if hasattr(p, "COV_ENABLE_WIREFRAME"):
                p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45.0,
            cameraPitch=-30.0,
            cameraTargetPosition=[args.x, args.y, args.z],
        )

        s_x = p.addUserDebugParameter("x (m)", args.x_min, args.x_max, args.x)
        s_y = p.addUserDebugParameter("y (m)", args.y_min, args.y_max, args.y)
        s_z = p.addUserDebugParameter("z (m)", args.z_min, args.z_max, args.z)
        s_roll = p.addUserDebugParameter("roll (rad)", -math.pi, math.pi, args.roll)
        s_pitch = p.addUserDebugParameter("pitch (rad)", -math.pi, math.pi, args.pitch)
        s_yaw = p.addUserDebugParameter("yaw (rad)", -math.pi, math.pi, args.yaw)
        s_speed = p.addUserDebugParameter("speed (m/s)", 0.0, args.speed_max, args.speed)

        marker_bid: Optional[int] = None
        if not args.no_marker:
            marker_vsid = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.02, rgbaColor=[1.0, 0.2, 0.2, 0.8])
            marker_bid = p.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=marker_vsid,
                basePosition=[args.x, args.y, args.z],
            )

        if not args.quiet:
            print("PyBullet GUI target controller running. Close the PyBullet window to stop.")
            print(f"Streaming /tcp/goal to {args.sim_ip}:{args.sim_port} at {send_hz:.1f} Hz")

        send_count = 0
        last_ui = 0.0
        last_send = 0.0
        last_status = 0.0
        last_marker_xyz: Optional[tuple[float, float, float]] = None
        while p.isConnected():
            now = time.time()
            if now - last_ui < ui_interval:
                time.sleep(0.002)
                continue
            last_ui = now

            try:
                x = float(p.readUserDebugParameter(s_x))
                y = float(p.readUserDebugParameter(s_y))
                z = float(p.readUserDebugParameter(s_z))
                roll = float(p.readUserDebugParameter(s_roll))
                pitch = float(p.readUserDebugParameter(s_pitch))
                yaw = float(p.readUserDebugParameter(s_yaw))
                speed = max(0.0, float(p.readUserDebugParameter(s_speed)))
            except p.error:
                # During startup/shutdown the GUI can briefly reject parameter reads.
                if not p.isConnected():
                    break
                time.sleep(0.01)
                continue

            quat = quat_from_rpy(roll, pitch, yaw)

            if marker_bid is not None:
                xyz = (x, y, z)
                if (
                    last_marker_xyz is None
                    or abs(xyz[0] - last_marker_xyz[0]) > 1e-4
                    or abs(xyz[1] - last_marker_xyz[1]) > 1e-4
                    or abs(xyz[2] - last_marker_xyz[2]) > 1e-4
                ):
                    p.resetBasePositionAndOrientation(marker_bid, [x, y, z], [0.0, 0.0, 0.0, 1.0])
                    last_marker_xyz = xyz

            if now - last_send >= send_interval:
                payload = [x, y, z, quat[0], quat[1], quat[2], quat[3], speed]
                client.send_message("/tcp/goal", payload)
                send_count += 1
                last_send = now

                if (not args.quiet) and status_interval > 0.0 and (now - last_status >= status_interval):
                    print(
                        f"Sent #{send_count} xyz=({x:+.3f}, {y:+.3f}, {z:+.3f}) "
                        f"rpy=({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f}) speed={speed:.3f}",
                        flush=True,
                    )
                    last_status = now

            time.sleep(0.001)
    finally:
        if p.isConnected():
            p.disconnect()


def run_gui_mode(args: argparse.Namespace, client: SimpleUDPClient) -> None:
    backend = args.backend
    if backend == "tk":
        run_tk_gui_mode(args, client)
        return
    if backend == "pygame":
        run_pygame_gui_mode(args, client)
        return
    if backend == "pybullet":
        run_pybullet_gui_mode(args, client)
        return

    # Auto: prefer Tk, then pygame, then PyBullet sliders as last resort.
    try:
        run_tk_gui_mode(args, client)
    except ValueError as tk_error:
        print(f"GUI backend auto-switch: {tk_error}", file=sys.stderr)
        try:
            print("Falling back to pygame GUI backend...", file=sys.stderr)
            run_pygame_gui_mode(args, client)
        except ValueError as pygame_error:
            print(f"GUI backend auto-switch: {pygame_error}", file=sys.stderr)
            print("Falling back to PyBullet GUI backend...", file=sys.stderr)
            run_pybullet_gui_mode(args, client)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="OSC TCP goal sender for PyBullet arm simulator")
    parser.add_argument("--sim-ip", default="127.0.0.1", help="Simulator OSC input IP")
    parser.add_argument("--sim-port", type=int, default=9000, help="Simulator OSC input port")
    parser.add_argument("--repeat", type=int, default=3, help="How many times to repeat each sent goal")
    parser.add_argument("--interval", type=float, default=0.02, help="Delay between repeated sends (seconds)")
    parser.add_argument("--listen", action="store_true", help="Listen and print simulator OSC output")
    parser.add_argument("--listen-ip", default="0.0.0.0", help="OSC listen IP for simulator outputs")
    parser.add_argument("--listen-port", type=int, default=9001, help="OSC listen port for simulator outputs")

    subparsers = parser.add_subparsers(dest="mode", required=True)

    goal = subparsers.add_parser("goal", help="Send one goal from CLI args")
    goal.add_argument("--x", type=float, required=True, help="Target x (m)")
    goal.add_argument("--y", type=float, required=True, help="Target y (m)")
    goal.add_argument("--z", type=float, required=True, help="Target z (m)")
    goal_group = goal.add_mutually_exclusive_group()
    goal_group.add_argument("--quat", nargs=4, type=float, metavar=("QX", "QY", "QZ", "QW"), help="Target quaternion")
    goal_group.add_argument("--rpy", nargs=3, type=float, metavar=("R", "P", "Y"), help="Target roll pitch yaw (rad)")
    goal.add_argument("--roll", type=float, default=0.0, help="Roll (rad) if --quat/--rpy not used")
    goal.add_argument("--pitch", type=float, default=0.0, help="Pitch (rad) if --quat/--rpy not used")
    goal.add_argument("--yaw", type=float, default=0.0, help="Yaw (rad) if --quat/--rpy not used")
    goal.add_argument("--speed", type=float, default=0.2, help="Linear speed m/s")

    repl = subparsers.add_parser("repl", help="Interactive goal sender")
    repl.add_argument(
        "--default-speed",
        type=float,
        default=0.2,
        help="Shown in prompt docs (speed still set per-line input)",
    )

    gui = subparsers.add_parser("gui", help="GUI slider panel to stream /tcp/goal")
    gui.add_argument("--x", type=float, default=0.45, help="Initial x (m)")
    gui.add_argument("--y", type=float, default=0.0, help="Initial y (m)")
    gui.add_argument("--z", type=float, default=0.35, help="Initial z (m)")
    gui.add_argument("--roll", type=float, default=0.0, help="Initial roll (rad)")
    gui.add_argument("--pitch", type=float, default=0.0, help="Initial pitch (rad)")
    gui.add_argument("--yaw", type=float, default=0.0, help="Initial yaw (rad)")
    gui.add_argument("--speed", type=float, default=0.2, help="Initial speed (m/s)")
    gui.add_argument("--x-min", type=float, default=0.1, help="Minimum x slider value (m)")
    gui.add_argument("--x-max", type=float, default=0.9, help="Maximum x slider value (m)")
    gui.add_argument("--y-min", type=float, default=-0.6, help="Minimum y slider value (m)")
    gui.add_argument("--y-max", type=float, default=0.6, help="Maximum y slider value (m)")
    gui.add_argument("--z-min", type=float, default=0.1, help="Minimum z slider value (m)")
    gui.add_argument("--z-max", type=float, default=0.9, help="Maximum z slider value (m)")
    gui.add_argument("--speed-max", type=float, default=1.0, help="Maximum speed slider value (m/s)")
    gui.add_argument("--send-hz", type=float, default=20.0, help="How fast to stream goals over OSC")
    gui.add_argument("--ui-hz", type=float, default=30.0, help="How fast to poll/update GUI controls")
    gui.add_argument("--status-hz", type=float, default=1.0, help="Console status print rate (set 0 to disable)")
    gui.add_argument("--no-marker", action="store_true", help="Disable moving target sphere marker in GUI")
    gui.add_argument("--quiet", action="store_true", help="Reduce console output")
    gui.add_argument(
        "--fast-gui",
        action="store_true",
        help="Disable some GUI visual extras (shadows/widgets) for better rendering performance.",
    )
    gui.add_argument(
        "--backend",
        choices=("auto", "tk", "pygame", "pybullet"),
        default="auto",
        help="GUI backend. auto tries Tk, then pygame, then PyBullet sliders",
    )

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.mode == "goal" and args.rpy is not None:
        args.roll = args.rpy[0]
        args.pitch = args.rpy[1]
        args.yaw = args.rpy[2]

    listener_server: Optional[ThreadingOSCUDPServer] = None
    listener_thread: Optional[threading.Thread] = None
    if args.listen:
        if args.listen_port == args.sim_port:
            print("listen-port must differ from sim-port", file=sys.stderr)
            return 2
        listener_server, listener_thread = start_listener(args.listen_ip, args.listen_port)
        print(f"Listening for OSC output on {args.listen_ip}:{args.listen_port}")

    client = SimpleUDPClient(args.sim_ip, args.sim_port)
    print(f"Sending OSC input to {args.sim_ip}:{args.sim_port}")

    try:
        if args.mode == "goal":
            run_goal_mode(args, client)
        elif args.mode == "repl":
            run_repl_mode(args, client)
        elif args.mode == "gui":
            run_gui_mode(args, client)
        else:
            parser.error(f"Unknown mode: {args.mode}")
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    finally:
        if listener_server is not None:
            listener_server.shutdown()
            listener_server.server_close()
        if listener_thread is not None:
            listener_thread.join(timeout=1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
