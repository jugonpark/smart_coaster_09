from __future__ import annotations

import json
import math
import re
import socket
import threading
import time

import cv2

import config


class MonitorReceiver:
    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((config.MONITOR_BIND_HOST, config.MONITOR_UDP_PORT))
        self._sock.settimeout(0.5)
        self._lock = threading.Lock()
        self._latest: dict = {}
        self._last_rx = 0.0  # local monotonic arrival; packet t is display data only
        self._running = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            try:
                data, _ = self._sock.recvfrom(65535)
                msg = json.loads(data.decode("utf-8"), parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if not isinstance(msg, dict) or not _finite_json(msg):
                    raise ValueError("invalid monitoring packet")
                with self._lock:
                    self._latest = msg
                    self._last_rx = time.monotonic()
            except socket.timeout:
                continue
            except (OSError, UnicodeError, ValueError, RecursionError):
                continue

    def snapshot(self) -> tuple[dict, bool]:
        with self._lock:
            age = time.monotonic() - self._last_rx if self._last_rx else float("inf")
            online = age <= config.MONITOR_STALE_S
            return (dict(self._latest) if online else {}), online

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1)


def _finite_json(value) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(_finite_json(item) for item in value)
    return value is None or isinstance(value, (str, int, bool))


def _text(value, missing="UNKNOWN") -> str:
    if isinstance(value, str) and value:
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value):
        return str(value)
    return missing


def _motion_value(status: dict, command: str, name: str) -> str:
    if name in status:
        return _numeric_text(status[name])
    found = re.search(rf"(?:^|\s){name}=([^\s]+)", command)
    if found:
        try:
            value = float(found.group(1))
            if math.isfinite(value):
                return found.group(1)
        except ValueError:
            pass
    return "N/A"


def _numeric_text(value) -> str:
    if isinstance(value, bool) or not isinstance(value, (str, int, float)):
        return "N/A"
    try:
        return str(value) if math.isfinite(float(value)) else "N/A"
    except (ValueError, OverflowError):
        return "N/A"


def format_status_lines(status: dict, online: bool, camera_state: str = "CAMERA_OFFLINE") -> list[str]:
    status = status if online and isinstance(status, dict) else {}
    command = _text(status.get("command"), "")
    command_status = command.split()[0] if command.split() else "N/A"
    if command_status not in ("RUN", "SLOW", "STOP"):
        command_status = _text(status.get("status"), "N/A")
    escape = _text(status.get("escape"), "")
    parts = escape.split()
    direction = _text(status.get("escape_direction"), parts[0] if parts else "UNKNOWN")
    target = _numeric_text(status.get("target_distance_cm"))
    if target == "N/A" and len(parts) > 1 and re.fullmatch(r"\d+(?:\.\d+)?cm", parts[1]):
        target = parts[1]
    elif target != "N/A":
        target += "cm"
    return [
        f"PI: {'ONLINE' if online else 'OFFLINE'}",
        f"CAMERA: {camera_state}",
        f"ESP32: {_text(status.get('esp32'))}",
        f"RADAR: {_text(status.get('radar'))}",
        f"SYSTEM: {_text(status.get('state'))}",
        f"RISK: {_text(status.get('risk'))}",
        f"THREAT: {_text(status.get('threat_direction'))}",
        f"ESCAPE: {direction}",
        f"TARGET: {target}",
        f"STATUS: {command_status}",
        f"VX: {_motion_value(status, command, 'vx')}",
        f"VY: {_motion_value(status, command, 'vy')}",
        f"W: {_motion_value(status, command, 'w')}",
    ]


def draw_status(frame, status: dict, online: bool, camera_state: str = "CAMERA_OFFLINE"):
    lines = format_status_lines(status, online, camera_state)
    y = 28
    for line in lines:
        cv2.putText(frame, line, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (0, 255, 0) if online else (0, 0, 255), 2, cv2.LINE_AA)
        y += 28
    return frame
