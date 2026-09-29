#!/usr/bin/env python3
"""Windows Tkinter central controller for the GRISE ESP32 velocity path.

This tool sends only body velocity commands using the existing six-field UDP
protocol. It does not activate distance goals, camera, radar, or autonomy.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import queue
import signal
import socket
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable

from esp32_udp_test import expected_wheel_targets


DEFAULT_IP = "10.182.7.50"
COMMAND_PORT = 8888
TELEMETRY_PORT = 8889
COMMAND_RATE_HZ = 20.0
TELEMETRY_TIMEOUT_S = 1.0
STOP_DURATION_S = 0.6
CONNECT_STOP_S = 0.6
MAX_LINEAR_CM_S = 50.0
MAX_ANGULAR_RAD_S = 3.0
MOTION_KEYS = frozenset(("W", "S", "A", "D", "Q", "E"))

STATE_LABEL_KO = {
    "DISCONNECTED": "연결 안 됨",
    "CONNECTED_STOPPED": "연결됨 · 정지",
    "RUNNING": "수동 운전 중",
    "TELEMETRY_LOST": "텔레메트리 끊김",
    "ESTOP": "비상 정지",
}
STATUS_LABEL_KO = {"RUN": "운전 (RUN)", "SLOW": "저속 (SLOW)", "STOP": "정지 (STOP)"}
MODE_LABEL_KO = {"NETWORK": "네트워크 (NETWORK)", "MANUAL_PWM": "수동 PWM (MANUAL_PWM)"}


class ControlState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTED_STOPPED = "CONNECTED_STOPPED"
    RUNNING = "RUNNING"
    TELEMETRY_LOST = "TELEMETRY_LOST"
    ESTOP = "ESTOP"


@dataclass(frozen=True)
class TelemetryState:
    seq: int
    ms: int | float
    status: str
    mode: str
    rssi: int | float | None
    counts: tuple[int | float, int | float, int | float]
    rpm: tuple[float, float, float]
    wheel_speed: tuple[float, float, float]
    target_speed: tuple[float, float, float]
    received_at: float
    raw: dict[str, Any]


def _three_finite(payload: dict[str, Any], key: str) -> tuple[Any, Any, Any]:
    values = payload.get(key)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{key} must contain three values")
    if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
        raise ValueError(f"{key} contains a non-finite value")
    return tuple(values)


def validate_telemetry(payload: Any, *, received_at: float | None = None) -> TelemetryState:
    if not isinstance(payload, dict) or payload.get("type") != "telemetry":
        raise ValueError("unexpected telemetry payload")
    if type(payload.get("seq")) is not int:
        raise ValueError("seq must be an integer")
    if not isinstance(payload.get("status"), str) or not isinstance(payload.get("mode"), str):
        raise ValueError("status/mode missing")
    ms = payload.get("ms", 0)
    if type(ms) not in (int, float) or not math.isfinite(ms):
        raise ValueError("ms must be finite")
    rssi = payload.get("rssi")
    if rssi is not None and (type(rssi) not in (int, float) or not math.isfinite(rssi)):
        raise ValueError("rssi must be finite")
    counts = _three_finite(payload, "counts")
    rpm = _three_finite(payload, "rpm")
    wheel_speed = _three_finite(payload, "wheel_speed")
    target_speed = _three_finite(payload, "target_speed")
    return TelemetryState(
        seq=payload["seq"], ms=ms, status=payload["status"], mode=payload["mode"],
        rssi=rssi, counts=counts, rpm=tuple(float(x) for x in rpm),
        wheel_speed=tuple(float(x) for x in wheel_speed),
        target_speed=tuple(float(x) for x in target_speed),
        received_at=time.monotonic() if received_at is None else received_at,
        raw=payload,
    )


class CommandSequence:
    def __init__(self) -> None:
        self._seq = 0
        self._lock = threading.Lock()

    def packet(self, sent_at: float, vx: float, vy: float, w: float,
               status: str) -> dict[str, int | float | str]:
        if status not in ("RUN", "SLOW", "STOP"):
            raise ValueError("invalid status")
        with self._lock:
            self._seq += 1
            seq = self._seq
        return {"seq": seq, "t": float(sent_at), "vx": float(vx),
                "vy": float(vy), "w": float(w), "status": status}


class ManualControl:
    """Thread-safe manual motion state with fail-closed transitions."""

    def __init__(self, telemetry_timeout_s: float = TELEMETRY_TIMEOUT_S,
                 connect_stop_s: float = CONNECT_STOP_S) -> None:
        self.telemetry_timeout_s = telemetry_timeout_s
        self.connect_stop_s = connect_stop_s
        self.state = ControlState.DISCONNECTED
        self.transport_active = False
        self.last_telemetry_at: float | None = None
        self.connection_started_at: float | None = None
        self._telemetry_confirmed = False
        self._estop_latched = False
        self._pressed: set[str] = set()
        self._lock = threading.RLock()

    def start_connection(self, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            self.transport_active = True
            self.state = (ControlState.ESTOP if self._estop_latched
                          else ControlState.DISCONNECTED)
            self.last_telemetry_at = None
            self.connection_started_at = now
            self._telemetry_confirmed = False
            self._pressed.clear()

    def disconnect(self) -> None:
        with self._lock:
            self.transport_active = False
            self.state = (ControlState.ESTOP if self._estop_latched
                          else ControlState.DISCONNECTED)
            self.last_telemetry_at = None
            self.connection_started_at = None
            self._telemetry_confirmed = False
            self._pressed.clear()

    def on_telemetry(self, telemetry: TelemetryState | dict[str, Any], *, now: float) -> bool:
        with self._lock:
            was_unavailable = not self._telemetry_confirmed
            self.last_telemetry_at = now
            sync_complete = (self.connection_started_at is not None and
                             now - self.connection_started_at >= self.connect_stop_s)
            if sync_complete:
                self._telemetry_confirmed = True
            if not self._estop_latched:
                if was_unavailable:
                    self._pressed.clear()
                if sync_complete:
                    self.state = (ControlState.RUNNING if self._pressed
                                  else ControlState.CONNECTED_STOPPED)
            return was_unavailable and self._telemetry_confirmed

    def check_timeout(self, *, now: float) -> bool:
        with self._lock:
            if (not self.transport_active or self.last_telemetry_at is None or
                    self.state in (ControlState.DISCONNECTED, ControlState.ESTOP,
                                   ControlState.TELEMETRY_LOST)):
                return False
            if now - self.last_telemetry_at <= self.telemetry_timeout_s:
                return False
            self._pressed.clear()
            self._telemetry_confirmed = False
            self.state = ControlState.TELEMETRY_LOST
            return True

    def press(self, key: str) -> None:
        key = key.upper()
        if key not in MOTION_KEYS:
            return
        with self._lock:
            if self.state not in (ControlState.CONNECTED_STOPPED, ControlState.RUNNING):
                return
            self._pressed.add(key)
            self.state = ControlState.RUNNING

    def release(self, key: str) -> None:
        with self._lock:
            self._pressed.discard(key.upper())
            if self.state == ControlState.RUNNING and not self._pressed:
                self.state = ControlState.CONNECTED_STOPPED

    def stop(self) -> None:
        with self._lock:
            self._pressed.clear()
            if self.state == ControlState.RUNNING:
                self.state = ControlState.CONNECTED_STOPPED

    def activate_estop(self) -> None:
        with self._lock:
            self._pressed.clear()
            self._estop_latched = True
            self.state = ControlState.ESTOP

    def release_estop(self, *, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._pressed.clear()
            self._estop_latched = False
            if not self.transport_active:
                self.state = ControlState.DISCONNECTED
            elif (not self._telemetry_confirmed or self.last_telemetry_at is None or
                  now - self.last_telemetry_at > self.telemetry_timeout_s):
                self.state = ControlState.TELEMETRY_LOST
            else:
                self.state = ControlState.CONNECTED_STOPPED

    def snapshot_state(self) -> tuple[ControlState, float | None]:
        with self._lock:
            return self.state, self.last_telemetry_at

    def command(self, linear_speed: float, angular_speed: float) -> tuple[float, float, float, str]:
        with self._lock:
            if self.state != ControlState.RUNNING:
                return 0.0, 0.0, 0.0, "STOP"
            x = float("W" in self._pressed) - float("S" in self._pressed)
            y = float("A" in self._pressed) - float("D" in self._pressed)
            rotation = float("Q" in self._pressed) - float("E" in self._pressed)
        magnitude = math.hypot(x, y)
        if magnitude > 0:
            vx = x / magnitude * linear_speed
            vy = y / magnitude * linear_speed
        else:
            vx = vy = 0.0
        w = rotation * angular_speed
        if vx == 0.0 and vy == 0.0 and w == 0.0:
            self.stop()
            return 0.0, 0.0, 0.0, "STOP"
        return vx, vy, w, "RUN"


class Esp32UdpClient:
    def __init__(self, ip: str, command_port: int, telemetry_port: int,
                 control: ManualControl, *, rate_hz: float = COMMAND_RATE_HZ,
                 stop_duration_s: float = STOP_DURATION_S,
                 event_sink: Callable[[str, Any], None] | None = None,
                 speed_provider: Callable[[], tuple[float, float]] | None = None) -> None:
        self.target = (ip, command_port)
        self.telemetry_port = telemetry_port
        self.control = control
        self.period = 1.0 / rate_hz
        self.stop_duration_s = stop_duration_s
        self.event_sink = event_sink or (lambda kind, value: None)
        self.speed_provider = speed_provider or (lambda: (10.0, 0.3))
        self.sequence = CommandSequence()
        self.sock: socket.socket | None = None
        self.latest: TelemetryState | None = None
        self._stop_event = threading.Event()
        self._send_lock = threading.Lock()
        self._threads: list[threading.Thread] = []
        self._started = False
        self.started_at: float | None = None
        self.local_ip: str | None = None
        self.sent_packets = 0
        self.received_datagrams = 0
        self.valid_telemetry_packets = 0
        self.invalid_telemetry_packets = 0
        self._absence_reported = False
        self._last_invalid_report_at = 0.0

    def start(self) -> None:
        if self._started:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self.telemetry_port))
        except Exception:
            sock.close()
            raise
        sock.settimeout(0.1)
        self.sock = sock
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(self.target)
            self.local_ip = probe.getsockname()[0]
        finally:
            probe.close()
        self._stop_event.clear()
        self.control.start_connection()
        self._started = True
        self.started_at = time.monotonic()
        self.event_sink("transport_started", {
            "local_ip": self.local_ip,
            "local_port": self.telemetry_port,
            "remote_ip": self.target[0],
            "remote_port": self.target[1],
        })
        self._threads = [
            threading.Thread(target=self._sender_loop, name="esp32-command", daemon=True),
            threading.Thread(target=self._receiver_loop, name="esp32-telemetry", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def _send(self, vx: float, vy: float, w: float, status: str) -> None:
        if self.sock is None:
            return
        packet = self.sequence.packet(time.monotonic(), vx, vy, w, status)
        wire = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
        with self._send_lock:
            self.sock.sendto(wire, self.target)
            self.sent_packets += 1

    def _sender_loop(self) -> None:
        deadline = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            try:
                if (self.latest is None and self.started_at is not None and
                        not self._absence_reported and
                        now - self.started_at > TELEMETRY_TIMEOUT_S):
                    self._absence_reported = True
                    self.event_sink("telemetry_absent", self.diagnostics())
                if self.control.check_timeout(now=now):
                    self.event_sink("telemetry_lost", None)
                linear, angular = self.speed_provider()
                vx, vy, w, status = self.control.command(linear, angular)
                self._send(vx, vy, w, status)
            except Exception as exc:
                self.control.activate_estop()
                try:
                    self._send(0.0, 0.0, 0.0, "STOP")
                except OSError:
                    pass
                self.event_sink("network_error", exc)
            deadline += self.period
            if deadline <= now:
                deadline = now + self.period
            self._stop_event.wait(max(0.0, deadline - time.monotonic()))

    def _receiver_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                assert self.sock is not None
                wire, sender = self.sock.recvfrom(8192)
            except socket.timeout:
                continue
            except OSError:
                return
            self.received_datagrams += 1
            if sender[0] != self.target[0]:
                continue
            try:
                state = validate_telemetry(json.loads(wire.decode("utf-8")))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                self.invalid_telemetry_packets += 1
                now = time.monotonic()
                if now - self._last_invalid_report_at >= 1.0:
                    self._last_invalid_report_at = now
                    self.event_sink("invalid_telemetry", {
                        "error": exc, "sender": sender, "bytes": len(wire),
                        "count": self.invalid_telemetry_packets,
                    })
                continue
            self.valid_telemetry_packets += 1
            self.latest = state
            restored = self.control.on_telemetry(state, now=state.received_at)
            self.event_sink("telemetry_restored" if restored else "telemetry", state)

    def diagnostics(self) -> dict[str, Any]:
        return {
            "local_ip": self.local_ip,
            "local_port": self.telemetry_port,
            "remote_ip": self.target[0],
            "remote_port": self.target[1],
            "sent": self.sent_packets,
            "received": self.received_datagrams,
            "valid": self.valid_telemetry_packets,
            "invalid": self.invalid_telemetry_packets,
        }

    def disconnect(self) -> None:
        if not self._started:
            return
        self.control.stop()
        self._stop_event.set()
        for thread in self._threads:
            if thread.name == "esp32-command":
                thread.join(timeout=0.5)
        deadline = time.monotonic() + self.stop_duration_s
        while time.monotonic() < deadline:
            try:
                self._send(0.0, 0.0, 0.0, "STOP")
            except OSError as exc:
                self.event_sink("network_error", exc)
                break
            time.sleep(self.period)
        sock = self.sock
        self.sock = None
        if sock is not None:
            sock.close()
        for thread in self._threads:
            if thread.name == "esp32-telemetry":
                thread.join(timeout=0.3)
        self._threads.clear()
        self._started = False
        self.started_at = None
        self.control.disconnect()
        self.event_sink("disconnected", None)


class CentralControllerApp:
    def __init__(self, root: Any, initial_ip: str = DEFAULT_IP) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk, self.ttk, self.root = tk, ttk, root
        self.root.title("GRISE 3WD 로봇 중앙 관제")
        self.root.geometry("950x760")
        self.control = ManualControl()
        self.client: Esp32UdpClient | None = None
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.last_command = (0.0, 0.0, 0.0, "STOP")
        self._last_gui_state = ControlState.DISCONNECTED

        self.ip_var = tk.StringVar(value=initial_ip)
        self.command_port_var = tk.StringVar(value=str(COMMAND_PORT))
        self.telemetry_port_var = tk.StringVar(value=str(TELEMETRY_PORT))
        self.linear_var = tk.StringVar(value="10.0")
        self.angular_var = tk.StringVar(value="0.30")
        self.connection_var = tk.StringVar(value=STATE_LABEL_KO["DISCONNECTED"])
        self.age_var = tk.StringVar(value="--")
        self.mode_var = tk.StringVar(value="--")
        self.status_var = tk.StringVar(value="--")
        self.seq_var = tk.StringVar(value="--")
        self.rssi_var = tk.StringVar(value="--")
        self.estop_var = tk.StringVar(value="비상 정지 해제됨")
        self.expected_vars = [tk.StringVar(value="0.00") for _ in range(3)]
        self.motor_vars = [[tk.StringVar(value="--") for _ in range(4)] for _ in range(3)]

        self._build_ui()
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.report_callback_exception = self._callback_exception
        self.root.after(100, self._refresh)

    def _build_ui(self) -> None:
        tk, ttk = self.tk, self.ttk
        outer = ttk.Frame(self.root, padding=10)
        outer.pack(fill="both", expand=True)

        connection = ttk.LabelFrame(outer, text="연결", padding=8)
        connection.pack(fill="x")
        for col, (label, variable, width) in enumerate((
                ("ESP32 IP", self.ip_var, 16), ("명령 포트", self.command_port_var, 7),
                ("텔레메트리 포트", self.telemetry_port_var, 7))):
            ttk.Label(connection, text=label).grid(row=0, column=col * 2, padx=4)
            ttk.Entry(connection, textvariable=variable, width=width).grid(row=0, column=col * 2 + 1)
        ttk.Button(connection, text="연결", command=self.connect).grid(row=0, column=6, padx=8)
        ttk.Button(connection, text="연결 해제", command=self.disconnect).grid(row=0, column=7)
        ttk.Label(connection, text="연결 상태:").grid(row=1, column=0, sticky="e")
        ttk.Label(connection, textvariable=self.connection_var, width=20).grid(
            row=1, column=1, columnspan=2, sticky="w")
        ttk.Label(connection, text="마지막 텔레메트리 경과 시간:").grid(row=1, column=3, sticky="e")
        ttk.Label(connection, textvariable=self.age_var).grid(row=1, column=4, sticky="w")

        estop = tk.Label(outer, textvariable=self.estop_var, bg="#245b2a", fg="white",
                         font=("Segoe UI", 18, "bold"), pady=8)
        estop.pack(fill="x", pady=8)

        motion = ttk.LabelFrame(outer, text="수동 주행 제어", padding=8)
        motion.pack(fill="x")
        ttk.Label(motion, text="직선 속도").grid(row=0, column=0)
        ttk.Spinbox(motion, from_=0.1, to=MAX_LINEAR_CM_S, increment=0.5,
                    textvariable=self.linear_var, width=8).grid(row=0, column=1)
        ttk.Label(motion, text="cm/s   회전 속도").grid(row=0, column=2)
        ttk.Spinbox(motion, from_=0.01, to=MAX_ANGULAR_RAD_S, increment=0.05,
                    textvariable=self.angular_var, width=8).grid(row=0, column=3)
        ttk.Label(motion, text="rad/s").grid(row=0, column=4)
        buttons = (("전진 (W)", "W", 1, 2), ("좌측 이동 (A)", "A", 2, 1),
                   ("정지 (Space)", None, 2, 2), ("우측 이동 (D)", "D", 2, 3),
                   ("후진 (S)", "S", 3, 2), ("반시계 회전 (Q)", "Q", 1, 0),
                   ("시계 회전 (E)", "E", 1, 4))
        for text, key, row, column in buttons:
            button = ttk.Button(motion, text=text, width=18)
            button.grid(row=row, column=column, padx=4, pady=4)
            if key is None:
                button.configure(command=self.stop)
            else:
                button.bind("<ButtonPress-1>", lambda event, k=key: self._motion_press(k))
                button.bind("<ButtonRelease-1>", lambda event, k=key: self._motion_release(k))
                button.bind("<Leave>", lambda event, k=key: self._motion_release(k))
        tk.Button(motion, text="비상 정지 (E-STOP / ESC)", bg="#b00020", fg="white",
                  font=("Segoe UI", 11, "bold"), command=self.estop).grid(
                      row=4, column=0, columnspan=3, sticky="ew", pady=6)
        ttk.Button(motion, text="비상 정지 해제", command=self.release_estop).grid(
            row=4, column=3, columnspan=2, sticky="ew", pady=6)

        status = ttk.LabelFrame(outer, text="로봇 상태 / 텔레메트리", padding=8)
        status.pack(fill="x", pady=8)
        labels = (("모드", self.mode_var), ("상태", self.status_var),
                  ("순번", self.seq_var), ("신호 세기", self.rssi_var))
        for i, (label, variable) in enumerate(labels):
            ttk.Label(status, text=f"{label}:").grid(row=0, column=i * 2, sticky="e", padx=3)
            ttk.Label(status, textvariable=variable, width=13).grid(row=0, column=i * 2 + 1, sticky="w")
        ttk.Label(status, text="모터").grid(row=1, column=0)
        for col, title in enumerate(("이론값", "목표", "측정", "RPM", "카운트"), 1):
            ttk.Label(status, text=title).grid(row=1, column=col, padx=8)
        for motor in range(3):
            ttk.Label(status, text=f"M{motor + 1}").grid(row=motor + 2, column=0)
            ttk.Label(status, textvariable=self.expected_vars[motor]).grid(row=motor + 2, column=1)
            for col in range(4):
                ttk.Label(status, textvariable=self.motor_vars[motor][col], width=12).grid(
                    row=motor + 2, column=col + 2)

        logs = ttk.LabelFrame(outer, text="이벤트 로그", padding=5)
        logs.pack(fill="both", expand=True)
        self.log_text = tk.Text(logs, height=12, state="disabled", font=("Consolas", 9))
        self.log_text.pack(fill="both", expand=True)

    def _speeds(self) -> tuple[float, float]:
        try:
            linear, angular = float(self.linear_var.get()), float(self.angular_var.get())
        except ValueError:
            return 0.0, 0.0
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return 0.0, 0.0
        return max(0.0, min(linear, MAX_LINEAR_CM_S)), max(0.0, min(angular, MAX_ANGULAR_RAD_S))

    def _event(self, kind: str, value: Any) -> None:
        self.events.put((kind, value))

    def log(self, message: str) -> None:
        line = f"{time.strftime('%H:%M:%S')} {message}\n"
        self.log_text.configure(state="normal")
        self.log_text.insert("end", line)
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 300:
            self.log_text.delete("1.0", f"{lines - 300}.0")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def connect(self) -> None:
        if self.client is not None:
            return
        try:
            ip = str(ipaddress.ip_address(self.ip_var.get().strip()))
            command_port = int(self.command_port_var.get())
            telemetry_port = int(self.telemetry_port_var.get())
            if not (1 <= command_port <= 65535 and 1 <= telemetry_port <= 65535):
                raise ValueError("port must be 1..65535")
            client = Esp32UdpClient(ip, command_port, telemetry_port, self.control,
                                    event_sink=self._event, speed_provider=self._speeds)
            client.start()
            self.client = client
            self.connection_var.set("텔레메트리 대기")
            self.log(f"ESP32 {ip} 연결 시작; STOP 동기화 중")
        except Exception as exc:
            self.log(f"연결 실패: {exc}")
            self.connection_var.set(STATE_LABEL_KO["DISCONNECTED"])

    def disconnect(self) -> None:
        client, self.client = self.client, None
        if client is not None:
            client.disconnect()
            self.log("연결 해제됨; STOP 반복 전송 완료")
        self.connection_var.set(STATE_LABEL_KO["DISCONNECTED"])

    def _motion_press(self, key: str) -> None:
        self.control.press(key)
        after, _ = self.control.snapshot_state()
        if after == ControlState.RUNNING:
            vx, vy, w, _ = self.control.command(*self._speeds())
            self.log(f"주행 명령 vx={vx:+.2f} vy={vy:+.2f} w={w:+.2f} ({key})")

    def _motion_release(self, key: str) -> None:
        before, _ = self.control.snapshot_state()
        self.control.release(key)
        after, _ = self.control.snapshot_state()
        if before == ControlState.RUNNING and after == ControlState.CONNECTED_STOPPED:
            self.log("STOP (주행 키 해제)")

    def stop(self) -> None:
        self.control.stop()
        self.log("STOP")

    def estop(self) -> None:
        self.control.activate_estop()
        self.log("비상 정지 활성화")

    def release_estop(self) -> None:
        self.control.release_estop()
        self.log("비상 정지 해제; STOP 상태로 대기")

    def _key_press(self, event: Any) -> None:
        key = event.keysym.upper()
        if key == "ESCAPE":
            self.estop()
        elif key == "SPACE":
            self.stop()
        elif key in MOTION_KEYS and event.widget.winfo_class() not in ("Entry", "TEntry", "Spinbox", "TSpinbox"):
            self._motion_press(key)

    def _key_release(self, event: Any) -> None:
        key = event.keysym.upper()
        if key in MOTION_KEYS:
            self._motion_release(key)

    def _refresh(self) -> None:
        try:
            while True:
                kind, value = self.events.get_nowait()
                if kind == "transport_started":
                    self.log(
                        f"UDP 준비: {value['local_ip']}:{value['local_port']} -> "
                        f"{value['remote_ip']}:{value['remote_port']}")
                    local_ip = value.get("local_ip")
                    if local_ip:
                        local_net = ipaddress.ip_network(f"{local_ip}/24", strict=False)
                        remote_net = ipaddress.ip_network(f"{value['remote_ip']}/24", strict=False)
                        if local_net != remote_net:
                            self.log(
                                "네트워크 경고: 현재 ESP32 펌웨어의 /24 서브넷 기준으로 "
                                f"노트북({local_net})과 ESP32({remote_net})가 서로 다른 망입니다")
                elif kind == "telemetry_restored":
                    self.connection_var.set(STATE_LABEL_KO["CONNECTED_STOPPED"])
                    self.log("텔레메트리 수신/복구; STOP 상태로 대기")
                elif kind == "telemetry_lost":
                    self.connection_var.set(STATE_LABEL_KO["TELEMETRY_LOST"])
                    self.log("텔레메트리 끊김; 안전 STOP 유지")
                elif kind == "telemetry_absent":
                    self.connection_var.set("텔레메트리 없음")
                    self.log(
                        "텔레메트리 없음: "
                        f"STOP 송신={value['sent']}건, UDP 수신={value['received']}건, "
                        f"정상={value['valid']}건, 거부={value['invalid']}건; "
                        "ESP32 Serial의 'UDP rx/accepted/rejected'를 확인하세요")
                elif kind == "invalid_telemetry":
                    self.log(
                        f"잘못된 텔레메트리 거부: 송신자={value['sender']} "
                        f"크기={value['bytes']} 누적={value['count']} 오류={value['error']}")
                elif kind == "network_error":
                    self.log(f"UDP 오류: {value}")
        except queue.Empty:
            pass

        state, received_at = self.control.snapshot_state()
        if state != self._last_gui_state:
            self._last_gui_state = state
        if state in (ControlState.CONNECTED_STOPPED, ControlState.RUNNING,
                     ControlState.TELEMETRY_LOST, ControlState.ESTOP):
            self.connection_var.set(STATE_LABEL_KO[state.value])
        self.estop_var.set("비상 정지 활성화" if state == ControlState.ESTOP else "비상 정지 해제됨")
        estop_color = "#b00020" if state == ControlState.ESTOP else "#245b2a"
        # The E-STOP label is the only direct child Label with this text variable.
        for widget in self.root.winfo_children()[0].winfo_children():
            if isinstance(widget, self.tk.Label) and str(widget.cget("textvariable")) == str(self.estop_var):
                widget.configure(bg=estop_color)
        self.age_var.set("--" if received_at is None else f"{max(0.0, time.monotonic() - received_at) * 1000:.0f} ms")
        linear, angular = self._speeds()
        self.last_command = self.control.command(linear, angular)
        for variable, value in zip(self.expected_vars, expected_wheel_targets(*self.last_command[:3])):
            variable.set(f"{value:+.2f}")
        telemetry = self.client.latest if self.client is not None else None
        if (self.client is not None and telemetry is None and self.client.started_at is not None and
                time.monotonic() - self.client.started_at > TELEMETRY_TIMEOUT_S):
            self.connection_var.set("텔레메트리 없음")
        if telemetry is not None:
            self.mode_var.set(MODE_LABEL_KO.get(telemetry.mode, telemetry.mode))
            self.status_var.set(STATUS_LABEL_KO.get(telemetry.status, telemetry.status))
            self.seq_var.set(str(telemetry.seq))
            self.rssi_var.set("--" if telemetry.rssi is None else f"{telemetry.rssi:g} dBm")
            for motor in range(3):
                values = (telemetry.target_speed[motor], telemetry.wheel_speed[motor],
                          telemetry.rpm[motor], telemetry.counts[motor])
                for variable, value in zip(self.motor_vars[motor], values):
                    variable.set(f"{value:+.2f}" if isinstance(value, float) else str(value))
        self.root.after(100, self._refresh)

    def _callback_exception(self, exc_type: type[BaseException], exc: BaseException,
                            traceback: Any) -> None:
        self.control.activate_estop()
        self.log(f"GUI 오류: {exc}; 비상 정지 활성화")

    def close(self) -> None:
        self.control.activate_estop()
        self.disconnect()
        self.root.destroy()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GRISE Windows laptop central controller")
    parser.add_argument("--ip", default=DEFAULT_IP, help="initial ESP32 IPv4 address")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    import tkinter as tk
    root = tk.Tk()
    app = CentralControllerApp(root, args.ip)
    signal.signal(signal.SIGINT, lambda signum, frame: root.after(0, app.close))
    try:
        root.mainloop()
    finally:
        app.control.activate_estop()
        app.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
