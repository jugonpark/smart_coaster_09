#!/usr/bin/env python3
"""Windows Tkinter central controller for the GRISE ESP32 typed velocity path.

This tool sends typed body velocity, STOP, and heartbeat commands. It does not
activate distance goals, camera, radar, or autonomy.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import ipaddress
import json
import math
import queue
import secrets
import signal
import shutil
import socket
import tempfile
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Callable

from esp32_udp_test import expected_wheel_targets


DEFAULT_IP = "10.232.69.103"
COMMAND_PORT = 8888
TELEMETRY_PORT = 8889
COMMAND_RATE_HZ = 20.0
TELEMETRY_TIMEOUT_S = 1.0
STOP_DURATION_S = 0.6
CONNECT_STOP_S = 0.6
MAX_LINEAR_CM_S = 50.0
MAX_ANGULAR_RAD_S = 3.0
MOTION_KEYS = frozenset(("W", "S", "A", "D", "Q", "E"))
WHEEL_STOP_THRESHOLD_CM_S = 0.5
WHEEL_TRACKING_TOLERANCE_CM_S = 0.75
LOG_INTERVAL_SEC = 1.0

CSV_COLUMNS = (
    "timestamp", "elapsed_s", "esp32_ip", "connected", "telemetry_valid",
    "telemetry_age_ms", "rssi", "mode", "status", "seq",
    "cmd_vx_cm_s", "cmd_vy_cm_s", "cmd_w_rad_s", "command_linear_speed_cm_s",
    "motion_elapsed_s", "command_expected_distance_cm", "command_expected_rotation_deg",
    "m1_target_cm_s", "m1_measured_cm_s", "m1_error_cm_s", "m1_rpm", "m1_pwm", "m1_encoder_count",
    "m2_target_cm_s", "m2_measured_cm_s", "m2_error_cm_s", "m2_rpm", "m2_pwm", "m2_encoder_count",
    "m3_target_cm_s", "m3_measured_cm_s", "m3_error_cm_s", "m3_rpm", "m3_pwm", "m3_encoder_count",
    "max_wheel_speed_difference_cm_s", "udp_sent_count", "udp_received_count",
    "telemetry_accepted_count", "telemetry_rejected_count",
)

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
    pwm: tuple[int | None, int | None, int | None]
    state: str
    fault: str
    session_id: int | None
    boot_id: int | None
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
    seq = payload.get("last_seq", payload.get("seq"))
    if type(seq) is not int:
        raise ValueError("seq must be an integer")
    state = payload.get("state")
    if state is not None and not isinstance(state, str):
        raise ValueError("state must be a string")
    status = ("RUN" if state in {"VELOCITY", "DISTANCE_ACTIVE", "DISTANCE_BRAKING"}
              else "STOP") if state is not None else payload.get("status")
    if not isinstance(status, str) or not isinstance(payload.get("mode"), str):
        raise ValueError("status/mode missing")
    ms = payload.get("uptime_ms", payload.get("ms", 0))
    if type(ms) not in (int, float) or not math.isfinite(ms):
        raise ValueError("ms must be finite")
    rssi = payload.get("wifi_rssi", payload.get("rssi"))
    if rssi is not None and (type(rssi) not in (int, float) or not math.isfinite(rssi)):
        raise ValueError("rssi must be finite")
    normalized = dict(payload)
    normalized["counts"] = payload.get("encoder_count", payload.get("counts"))
    normalized["wheel_speed"] = payload.get("wheel_speed")
    normalized["target_speed"] = payload.get("wheel_target", payload.get("target_speed"))
    counts = _three_finite(normalized, "counts")
    wheel_speed = _three_finite(normalized, "wheel_speed")
    target_speed = _three_finite(normalized, "target_speed")
    rpm_values = payload.get("rpm")
    if rpm_values is None:
        rpm = tuple(float(speed) * 60.0 / (2.0 * math.pi * 2.9)
                    for speed in wheel_speed)
    else:
        rpm = tuple(float(x) for x in _three_finite(payload, "rpm"))
    pwm_values = payload.get("wheel_pwm", payload.get("pwm"))
    if pwm_values is None:
        pwm: tuple[int | None, int | None, int | None] = (None, None, None)
    else:
        if (not isinstance(pwm_values, list) or len(pwm_values) != 3 or
                not all(type(value) is int and -255 <= value <= 255 for value in pwm_values)):
            raise ValueError("pwm must contain three integers in -255..255")
        pwm = tuple(pwm_values)
    return TelemetryState(
        seq=seq, ms=ms, status=status, mode=payload["mode"],
        rssi=rssi, counts=counts, rpm=rpm,
        wheel_speed=tuple(float(x) for x in wheel_speed),
        target_speed=tuple(float(x) for x in target_speed),
        pwm=pwm, state=state or status, fault=str(payload.get("fault", "NONE")),
        session_id=payload.get("session_id"), boot_id=payload.get("boot_id"),
        received_at=time.monotonic() if received_at is None else received_at,
        raw=payload,
    )


def wheel_tracking_status(target: float, measured: float) -> str:
    if abs(target) < WHEEL_STOP_THRESHOLD_CM_S:
        return "정지"
    if (abs(measured) >= WHEEL_STOP_THRESHOLD_CM_S and
            math.copysign(1.0, target) != math.copysign(1.0, measured)):
        return "방향 이상"
    magnitude_error = abs(target) - abs(measured)
    if magnitude_error > WHEEL_TRACKING_TOLERANCE_CM_S:
        return "느림"
    if magnitude_error < -WHEEL_TRACKING_TOLERANCE_CM_S:
        return "빠름"
    return "정상"


def active_wheel_speed_spread(telemetry: TelemetryState) -> float:
    active = [abs(measured) for target, measured in
              zip(telemetry.target_speed, telemetry.wheel_speed)
              if abs(target) >= WHEEL_STOP_THRESHOLD_CM_S]
    return max(active) - min(active) if len(active) >= 2 else 0.0


def format_motor_snapshot(telemetry: TelemetryState) -> str:
    lines = ["[모터 스냅샷]"]
    for motor in range(3):
        pwm = "--" if telemetry.pwm[motor] is None else f"{telemetry.pwm[motor]:+d}"
        lines.append(
            f"M{motor + 1} Target={telemetry.target_speed[motor]:+.2f} "
            f"Measured={telemetry.wheel_speed[motor]:+.2f} "
            f"RPM={telemetry.rpm[motor]:+.1f} PWM={pwm} "
            f"Count={int(telemetry.counts[motor])}")
    lines.append(f"활성 바퀴 measured magnitude 최대 편차 = "
                 f"{active_wheel_speed_spread(telemetry):.2f} cm/s")
    return "\n".join(lines)


class CommandSequence:
    def __init__(self, session_id: int | None = None) -> None:
        self._seq = 0
        self.session_id = session_id if session_id is not None else (secrets.randbits(32) or 1)
        if type(self.session_id) is not int or self.session_id <= 0:
            raise ValueError("session_id must be a nonzero integer")
        self._lock = threading.Lock()

    def _base(self, command_type: str) -> dict[str, int | str]:
        with self._lock:
            self._seq += 1
            seq = self._seq
        return {"type": command_type, "session_id": self.session_id, "seq": seq}

    def packet(self, sent_at: float, vx: float, vy: float, w: float,
               status: str) -> dict[str, int | float | str]:
        if status not in ("RUN", "SLOW", "STOP"):
            raise ValueError("invalid status")
        if status == "STOP":
            return self._base("stop")
        packet: dict[str, int | float | str] = self._base("cmd_vel")
        packet.update({"vx": float(vx), "vy": float(vy), "w": float(w),
                       "status": status})
        return packet

    def heartbeat(self) -> dict[str, int | str]:
        return self._base("heartbeat")

    def move(self, vx: float, vy: float, w: float, status: str, *,
             motion_id: int, target_distance_cm: float) -> dict[str, int | float | str]:
        if status not in ("RUN", "SLOW"):
            raise ValueError("distance command status must be RUN or SLOW")
        if type(motion_id) is not int or motion_id < 0:
            raise ValueError("motion_id must be a nonnegative integer")
        packet: dict[str, int | float | str] = self._base("cmd_move")
        packet.update({"vx": float(vx), "vy": float(vy), "w": float(w),
                       "status": status, "motion_id": motion_id,
                       "target_distance_cm": float(target_distance_cm)})
        return packet


@dataclass(frozen=True)
class MotionMeasurementSnapshot:
    elapsed_s: float
    expected_distance_cm: float
    expected_angle_rad: float
    expected_angle_deg: float
    linear_speed_cm_s: float
    angular_speed_rad_s: float
    command_label: str
    active: bool


class MotionMeasurement:
    """Command-level time and distance estimate, independent of motor safety."""

    def __init__(self) -> None:
        self._active = False
        self._started_at: float | None = None
        self._last_update_at: float | None = None
        self._elapsed_s = 0.0
        self._expected_distance_cm = 0.0
        self._expected_angle_rad = 0.0
        self._linear_speed_cm_s = 0.0
        self._angular_speed_rad_s = 0.0
        self._command_label = "없음"

    @staticmethod
    def _label(keys: tuple[str, ...] | list[str] | str) -> str:
        if isinstance(keys, str):
            return keys
        return "+".join(key for key in ("W", "A", "S", "D", "Q", "E") if key in keys) or "없음"

    def _integrate_until(self, now: float) -> None:
        if not self._active or self._last_update_at is None:
            return
        delta = max(0.0, now - self._last_update_at)
        self._elapsed_s += delta
        self._expected_distance_cm += self._linear_speed_cm_s * delta
        self._expected_angle_rad += abs(self._angular_speed_rad_s) * delta
        self._last_update_at = now

    def begin_or_update(self, *, vx: float, vy: float, w: float,
                        label: tuple[str, ...] | list[str] | str,
                        now: float) -> None:
        if not self._active:
            self._active = True
            self._started_at = now
            self._last_update_at = now
            self._elapsed_s = 0.0
            self._expected_distance_cm = 0.0
            self._expected_angle_rad = 0.0
        else:
            self._integrate_until(now)
        self._linear_speed_cm_s = math.hypot(float(vx), float(vy))
        self._angular_speed_rad_s = float(w)
        self._command_label = self._label(label)

    def finish(self, *, now: float, label: str | None = None) -> MotionMeasurementSnapshot:
        self._integrate_until(now)
        self._active = False
        if label is not None:
            self._command_label = label
        return self.snapshot(now=now)

    def reset(self, *, now: float) -> None:
        if self._active:
            self._integrate_until(now)
            self._started_at = now
            self._last_update_at = now
        else:
            self._started_at = None
            self._last_update_at = None
        self._elapsed_s = 0.0
        self._expected_distance_cm = 0.0
        self._expected_angle_rad = 0.0
        self._linear_speed_cm_s = 0.0 if not self._active else self._linear_speed_cm_s
        self._angular_speed_rad_s = 0.0 if not self._active else self._angular_speed_rad_s
        self._command_label = "없음" if not self._active else self._command_label

    def snapshot(self, *, now: float) -> MotionMeasurementSnapshot:
        self._integrate_until(now)
        return MotionMeasurementSnapshot(
            elapsed_s=self._elapsed_s,
            expected_distance_cm=self._expected_distance_cm,
            expected_angle_rad=self._expected_angle_rad,
            expected_angle_deg=math.degrees(self._expected_angle_rad),
            linear_speed_cm_s=self._linear_speed_cm_s,
            angular_speed_rad_s=self._angular_speed_rad_s,
            command_label=self._command_label,
            active=self._active,
        )


class SessionLogger:
    """Temporary UTF-8 CSV session with explicit export and cleanup."""

    def __init__(self, interval_s: float = LOG_INTERVAL_SEC) -> None:
        self.interval_s = float(interval_s)
        self.temp_path: Path | None = None
        self.recording = False
        self.row_count = 0
        self.started_at: float | None = None
        self.stopped_elapsed_s = 0.0
        self.next_sample_at: float | None = None
        self.saved = False

    @property
    def has_unsaved_data(self) -> bool:
        return self.temp_path is not None and self.row_count > 0 and not self.saved

    def elapsed(self, now: float) -> float:
        if self.started_at is None:
            return 0.0
        return max(0.0, now - self.started_at) if self.recording else self.stopped_elapsed_s

    def start(self, *, now: float) -> Path:
        self.cleanup()
        handle = tempfile.NamedTemporaryFile(prefix="grise_session_", suffix=".csv", delete=False)
        handle.close()
        self.temp_path = Path(handle.name)
        with self.temp_path.open("w", newline="", encoding="utf-8-sig") as stream:
            csv.DictWriter(stream, fieldnames=CSV_COLUMNS).writeheader()
        self.recording = True
        self.row_count = 0
        self.started_at = now
        self.stopped_elapsed_s = 0.0
        self.next_sample_at = now
        self.saved = False
        return self.temp_path

    def due(self, *, now: float) -> bool:
        return self.recording and self.next_sample_at is not None and now >= self.next_sample_at

    def append_snapshot(self, row: dict[str, Any], *, now: float) -> None:
        if not self.recording or self.temp_path is None:
            raise RuntimeError("logging session is not active")
        output = {column: row.get(column, "") for column in CSV_COLUMNS}
        output["timestamp"] = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        output["elapsed_s"] = f"{self.elapsed(now):.3f}"
        with self.temp_path.open("a", newline="", encoding="utf-8") as stream:
            csv.DictWriter(stream, fieldnames=CSV_COLUMNS).writerow(output)
        self.row_count += 1
        self.saved = False
        self.next_sample_at = now + self.interval_s

    def stop(self, *, now: float) -> None:
        if self.recording:
            self.stopped_elapsed_s = self.elapsed(now)
            self.recording = False
            self.next_sample_at = None

    def export_csv(self, destination: str | Path) -> Path:
        if self.temp_path is None or not self.temp_path.exists():
            raise RuntimeError("no logging session to export")
        destination_path = Path(destination)
        shutil.copy2(self.temp_path, destination_path)
        self.saved = True
        return destination_path

    def reset(self) -> None:
        self.stop(now=time.monotonic())
        if self.temp_path is not None:
            try:
                self.temp_path.unlink(missing_ok=True)
            finally:
                self.temp_path = None
        self.row_count = 0
        self.started_at = None
        self.stopped_elapsed_s = 0.0
        self.next_sample_at = None
        self.saved = False

    def cleanup(self) -> None:
        self.reset()


def build_log_snapshot(*, esp32_ip: str, connected: bool,
                       command: tuple[float, float, float, str],
                       motion: MotionMeasurementSnapshot,
                       telemetry: TelemetryState | None,
                       telemetry_age_ms: float | None,
                       diagnostics: dict[str, Any]) -> dict[str, Any]:
    telemetry_valid = telemetry is not None and telemetry_age_ms is not None
    row: dict[str, Any] = {
        "esp32_ip": esp32_ip,
        "connected": str(bool(connected)).lower(),
        "telemetry_valid": str(telemetry_valid).lower(),
        "telemetry_age_ms": "" if telemetry_age_ms is None else f"{telemetry_age_ms:.1f}",
        "rssi": "" if telemetry is None or telemetry.rssi is None else telemetry.rssi,
        "mode": "" if telemetry is None else telemetry.mode,
        "status": telemetry.status if telemetry is not None else command[3],
        "seq": "" if telemetry is None else telemetry.seq,
        "cmd_vx_cm_s": f"{command[0]:.3f}",
        "cmd_vy_cm_s": f"{command[1]:.3f}",
        "cmd_w_rad_s": f"{command[2]:.3f}",
        "command_linear_speed_cm_s": f"{math.hypot(command[0], command[1]):.3f}",
        "motion_elapsed_s": f"{motion.elapsed_s:.3f}",
        "command_expected_distance_cm": f"{motion.expected_distance_cm:.3f}",
        "command_expected_rotation_deg": f"{motion.expected_angle_deg:.3f}",
        "max_wheel_speed_difference_cm_s": "" if telemetry is None else f"{active_wheel_speed_spread(telemetry):.3f}",
        "udp_sent_count": diagnostics.get("sent", 0),
        "udp_received_count": diagnostics.get("received", 0),
        "telemetry_accepted_count": diagnostics.get("valid", 0),
        "telemetry_rejected_count": diagnostics.get("invalid", 0),
    }
    if telemetry is not None:
        for motor in range(3):
            prefix = f"m{motor + 1}_"
            target = telemetry.target_speed[motor]
            measured = telemetry.wheel_speed[motor]
            row.update({
                prefix + "target_cm_s": f"{target:.3f}",
                prefix + "measured_cm_s": f"{measured:.3f}",
                prefix + "error_cm_s": f"{target - measured:.3f}",
                prefix + "rpm": f"{telemetry.rpm[motor]:.3f}",
                prefix + "pwm": "" if telemetry.pwm[motor] is None else telemetry.pwm[motor],
                prefix + "encoder_count": int(telemetry.counts[motor]),
            })
    return row


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

    def pressed_keys(self) -> tuple[str, ...]:
        with self._lock:
            return tuple(key for key in ("W", "A", "S", "D", "Q", "E")
                         if key in self._pressed)

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
        self.root.geometry("1280x860")
        self.root.minsize(1040, 720)
        self.control = ManualControl()
        self.client: Esp32UdpClient | None = None
        self.events: queue.Queue[tuple[str, Any]] = queue.Queue()
        self.last_command = (0.0, 0.0, 0.0, "STOP")
        self.motion_measurement = MotionMeasurement()
        self.session_logger = SessionLogger()
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
        self.header_status_var = tk.StringVar(value="연결 안 됨")
        self.header_ip_var = tk.StringVar(value="ESP32 IP --")
        self.header_rssi_var = tk.StringVar(value="Wi-Fi RSSI --")
        self.network_detail_var = tk.StringVar(value="UDP 송신 0 · 수신 0 · 정상 0 · 거부 0")
        self.measurement_command_var = tk.StringVar(value="없음")
        self.measurement_elapsed_var = tk.StringVar(value="0.00 s")
        self.measurement_speed_var = tk.StringVar(value="0.00 cm/s")
        self.measurement_distance_var = tk.StringVar(value="0.00 cm")
        self.measurement_angular_speed_var = tk.StringVar(value="0.00 rad/s")
        self.measurement_angle_var = tk.StringVar(value="0.0°")
        self.measurement_state_var = tk.StringVar(value="정지 / 측정 대기")
        self.expected_vars = [tk.StringVar(value="0.00") for _ in range(3)]
        self.motor_vars = [[tk.StringVar(value="--") for _ in range(7)] for _ in range(3)]
        self.balance_vars = [tk.StringVar(value=f"M{i + 1} |Measured| -- cm/s") for i in range(3)]
        self.balance_spread_var = tk.StringVar(value="활성 바퀴 최대 속도 편차 -- cm/s")
        self.logging_status_var = tk.StringVar(value="○ 대기")
        self.logging_elapsed_var = tk.StringVar(value="00:00:00")
        self.logging_rows_var = tk.StringVar(value="0")
        self.logging_interval_var = tk.StringVar(value=f"{LOG_INTERVAL_SEC:.1f} s")

        self._build_ui()
        self.root.bind_all("<KeyPress>", self._key_press)
        self.root.bind_all("<KeyRelease>", self._key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.report_callback_exception = self._callback_exception
        self.root.after(100, self._refresh)

    def _build_ui(self) -> None:
        tk, ttk = self.tk, self.ttk
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Panel.TLabelframe", background="#ffffff")
        style.configure("Panel.TLabelframe.Label", font=("Segoe UI", 11, "bold"), foreground="#263238")
        style.configure("Panel.TFrame", background="#ffffff")
        style.configure("Action.TButton", padding=(10, 6), font=("Segoe UI", 10))
        style.configure("MotorNormal.TLabel", foreground="#2e7d32")
        style.configure("MotorWarning.TLabel", foreground="#ef6c00")
        style.configure("MotorDanger.TLabel", foreground="#c62828", font=("Segoe UI", 9, "bold"))

        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(5, weight=1)

        header = tk.Frame(outer, bg="#ffffff", padx=14, pady=12)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        header.columnconfigure(1, weight=1)
        tk.Label(header, text="GRISE 3WD 로봇 중앙 관제", bg="#ffffff", fg="#263238",
                 font=("Segoe UI", 20, "bold")).grid(row=0, column=0, sticky="w")
        self.header_status_dot = tk.Label(header, text="●", bg="#ffffff", fg="#9e9e9e",
                                          font=("Segoe UI", 16))
        self.header_status_dot.grid(row=0, column=2, padx=(20, 5))
        tk.Label(header, textvariable=self.header_status_var, bg="#ffffff", fg="#455a64",
                 font=("Segoe UI", 10, "bold")).grid(row=0, column=3, padx=5)
        tk.Label(header, textvariable=self.header_ip_var, bg="#ffffff", fg="#607d8b",
                 font=("Segoe UI", 10)).grid(row=0, column=4, padx=10)
        tk.Label(header, textvariable=self.header_rssi_var, bg="#ffffff", fg="#607d8b",
                 font=("Segoe UI", 10)).grid(row=0, column=5, padx=5)

        body = ttk.Frame(outer)
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(0, weight=1)
        body.columnconfigure(1, weight=2)
        body.columnconfigure(2, weight=1)

        connection = ttk.LabelFrame(body, text="연결 설정", style="Panel.TLabelframe", padding=12)
        connection.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        for row, (label, variable, width) in enumerate((
                ("ESP32 IP", self.ip_var, 18), ("명령 포트", self.command_port_var, 8),
                ("텔레메트리 포트", self.telemetry_port_var, 8))):
            ttk.Label(connection, text=label).grid(row=row, column=0, sticky="w", pady=4)
            ttk.Entry(connection, textvariable=variable, width=width).grid(row=row, column=1, sticky="ew", pady=4)
        ttk.Button(connection, text="연결", style="Action.TButton", command=self.connect).grid(
            row=3, column=0, columnspan=2, sticky="ew", pady=(12, 4))
        ttk.Button(connection, text="연결 해제", style="Action.TButton", command=self.disconnect).grid(
            row=4, column=0, columnspan=2, sticky="ew", pady=4)
        ttk.Separator(connection).grid(row=5, column=0, columnspan=2, sticky="ew", pady=12)
        ttk.Label(connection, text="네트워크 상세", font=("Segoe UI", 9, "bold")).grid(
            row=6, column=0, columnspan=2, sticky="w")
        ttk.Label(connection, textvariable=self.network_detail_var, wraplength=210,
                  foreground="#607d8b").grid(row=7, column=0, columnspan=2, sticky="w", pady=4)
        connection.columnconfigure(1, weight=1)

        center = ttk.Frame(body)
        center.grid(row=0, column=1, sticky="nsew", padx=8)
        center.columnconfigure(0, weight=1)
        motion = ttk.LabelFrame(center, text="수동 주행 제어", style="Panel.TLabelframe", padding=12)
        motion.grid(row=0, column=0, sticky="ew")
        ttk.Label(motion, text="이동 속도").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(motion, from_=0.1, to=MAX_LINEAR_CM_S, increment=0.5,
                    textvariable=self.linear_var, width=8).grid(row=0, column=1, padx=5)
        ttk.Label(motion, text="cm/s").grid(row=0, column=2, sticky="w")
        ttk.Label(motion, text="회전 속도").grid(row=0, column=3, sticky="w", padx=(20, 0))
        ttk.Spinbox(motion, from_=0.01, to=MAX_ANGULAR_RAD_S, increment=0.05,
                    textvariable=self.angular_var, width=8).grid(row=0, column=4, padx=5)
        ttk.Label(motion, text="rad/s").grid(row=0, column=5, sticky="w")
        buttons = (("W\n전진", "W", 1, 2), ("A\n좌측", "A", 2, 1),
                   ("정지\nSpace", None, 2, 2), ("D\n우측", "D", 2, 3),
                   ("S\n후진", "S", 3, 2), ("Q\n반시계", "Q", 1, 1),
                   ("E\n시계", "E", 1, 3))
        for text, key, row, column in buttons:
            button = ttk.Button(motion, text=text, style="Action.TButton", width=11)
            button.grid(row=row, column=column, padx=4, pady=4, sticky="ew")
            if key is None:
                button.configure(command=self.stop)
            else:
                button.bind("<ButtonPress-1>", lambda event, k=key: self._motion_press(k))
                button.bind("<ButtonRelease-1>", lambda event, k=key: self._motion_release(k))
                button.bind("<Leave>", lambda event, k=key: self._motion_release(k))
        tk.Button(motion, text="비상 정지 (E-STOP)", bg="#c62828", activebackground="#8e0000",
                  fg="white", font=("Segoe UI", 11, "bold"), relief="flat",
                  command=self.estop).grid(row=4, column=0, columnspan=3, sticky="ew", pady=(10, 4))
        ttk.Button(motion, text="비상 정지 해제", style="Action.TButton", command=self.release_estop).grid(
            row=4, column=3, columnspan=3, sticky="ew", pady=(10, 4))
        for column in range(6):
            motion.columnconfigure(column, weight=1)

        measurement = ttk.LabelFrame(center, text="주행 측정", style="Panel.TLabelframe", padding=12)
        measurement.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        measurement.columnconfigure(1, weight=1)
        measurement_rows = (("주행 시간", self.measurement_elapsed_var),
                            ("명령 기준 예상 이동거리", self.measurement_distance_var),
                            ("명령 기준 예상 회전각", self.measurement_angle_var),
                            ("현재 명령", self.measurement_command_var))
        for row, (label, variable) in enumerate(measurement_rows):
            ttk.Label(measurement, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Label(measurement, textvariable=variable, font=("Segoe UI", 11, "bold")).grid(
                row=row, column=1, sticky="e", pady=3)
        ttk.Button(measurement, text="측정 초기화", command=self.reset_measurement).grid(
            row=0, column=2, rowspan=2, padx=(18, 0), sticky="ns")
        ttk.Label(measurement, textvariable=self.measurement_state_var,
                  foreground="#607d8b").grid(row=3, column=0, columnspan=3, sticky="w", pady=(5, 0))

        status = ttk.LabelFrame(body, text="로봇 상태", style="Panel.TLabelframe", padding=12)
        status.grid(row=0, column=2, sticky="nsew", padx=(8, 0))
        status_rows = (("제어 모드", self.mode_var), ("동작 상태", self.status_var),
                       ("텔레메트리", self.connection_var), ("마지막 수신", self.age_var),
                       ("Wi-Fi RSSI", self.rssi_var))
        for row, (label, variable) in enumerate(status_rows):
            ttk.Label(status, text=label).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Label(status, textvariable=variable, font=("Segoe UI", 10, "bold")).grid(
                row=row, column=1, sticky="e", pady=5)
        status.columnconfigure(1, weight=1)

        recording = ttk.LabelFrame(body, text="데이터 기록", style="Panel.TLabelframe", padding=12)
        recording.grid(row=1, column=2, sticky="new", padx=(8, 0), pady=(10, 0))
        recording_rows = (("상태", self.logging_status_var),
                          ("기록 시간", self.logging_elapsed_var),
                          ("기록 행", self.logging_rows_var),
                          ("주기", self.logging_interval_var))
        for row, (label, variable) in enumerate(recording_rows):
            ttk.Label(recording, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Label(recording, textvariable=variable, font=("Segoe UI", 9, "bold")).grid(
                row=row, column=1, sticky="e", pady=3)
        ttk.Button(recording, text="기록 시작", command=self.start_logging).grid(
            row=4, column=0, sticky="ew", pady=(10, 3), padx=(0, 3))
        ttk.Button(recording, text="기록 중지", command=self.stop_logging).grid(
            row=4, column=1, sticky="ew", pady=(10, 3), padx=(3, 0))
        ttk.Button(recording, text="CSV 저장", command=self.save_csv).grid(
            row=5, column=0, sticky="ew", pady=3, padx=(0, 3))
        ttk.Button(recording, text="기록 초기화", command=self.reset_logging).grid(
            row=5, column=1, sticky="ew", pady=3, padx=(3, 0))
        recording.columnconfigure(0, weight=1)
        recording.columnconfigure(1, weight=1)

        estop = tk.Label(outer, textvariable=self.estop_var, bg="#2e7d32", fg="white",
                         font=("Segoe UI", 11, "bold"), pady=6)
        estop.grid(row=2, column=0, sticky="ew", pady=(10, 10))
        self.estop_widget = estop

        motors = ttk.LabelFrame(outer, text="모터 실시간 상태", style="Panel.TLabelframe", padding=8)
        motors.grid(row=3, column=0, sticky="nsew", pady=(0, 8))
        for column, title in enumerate(("Motor", "Target\ncm/s", "Measured\ncm/s",
                                        "Error\ncm/s", "RPM", "PWM", "Encoder\ncount", "상태")):
            ttk.Label(motors, text=title, font=("Segoe UI", 9, "bold")).grid(
                row=0, column=column, sticky="ew", padx=8, pady=2)
        self.motor_status_labels = []
        for motor in range(3):
            ttk.Label(motors, text=f"M{motor + 1}").grid(row=motor + 1, column=0, sticky="w", padx=8)
            for column in range(6):
                ttk.Label(motors, textvariable=self.motor_vars[motor][column], width=16).grid(
                    row=motor + 1, column=column + 1, sticky="ew", padx=8)
            status_label = ttk.Label(motors, textvariable=self.motor_vars[motor][6],
                                     style="MotorNormal.TLabel", width=12)
            status_label.grid(row=motor + 1, column=7, sticky="ew", padx=8)
            self.motor_status_labels.append(status_label)
        ttk.Button(motors, text="현재값 기록", command=self.snapshot_motors).grid(
            row=4, column=6, columnspan=2, sticky="e", padx=8, pady=(8, 0))
        for column in range(8):
            motors.columnconfigure(column, weight=1)

        balance = ttk.LabelFrame(outer, text="주행 균형", style="Panel.TLabelframe", padding=8)
        balance.grid(row=4, column=0, sticky="ew", pady=(0, 8))
        for column, variable in enumerate(self.balance_vars):
            ttk.Label(balance, textvariable=variable).grid(row=0, column=column, sticky="w", padx=10)
            balance.columnconfigure(column, weight=1)
        ttk.Label(balance, textvariable=self.balance_spread_var, font=("Segoe UI", 9, "bold")).grid(
            row=1, column=0, columnspan=3, sticky="w", padx=10, pady=(5, 0))

        logs = ttk.LabelFrame(outer, text="이벤트 로그", style="Panel.TLabelframe", padding=6)
        logs.grid(row=5, column=0, sticky="nsew")
        logs.columnconfigure(0, weight=1)
        logs.rowconfigure(1, weight=1)
        ttk.Button(logs, text="로그 지우기", command=self.clear_log).grid(row=0, column=1, sticky="e")
        self.log_text = tk.Text(logs, height=7, state="disabled", font=("Consolas", 9),
                                bg="#fafafa", fg="#37474f", relief="flat", padx=8, pady=5)
        self.log_text.grid(row=1, column=0, columnspan=2, sticky="nsew")
        scrollbar = ttk.Scrollbar(logs, orient="vertical", command=self.log_text.yview)
        scrollbar.grid(row=1, column=2, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)

    def clear_log(self) -> None:
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def snapshot_motors(self) -> None:
        telemetry = self.client.latest if self.client is not None else None
        if telemetry is None:
            self.log("모터 스냅샷 실패: 수신된 텔레메트리가 없습니다")
            return
        self.log(format_motor_snapshot(telemetry))

    def _confirm_discard_log(self) -> bool:
        if not self.session_logger.has_unsaved_data:
            return True
        from tkinter import messagebox
        return messagebox.askokcancel(
            "미저장 기록 확인",
            "현재 기록된 데이터가 아직 저장되지 않았습니다.\n"
            "계속하면 현재 임시 기록이 삭제됩니다.",
            parent=self.root,
        )

    def start_logging(self) -> None:
        if self.session_logger.recording:
            return
        if self.session_logger.temp_path is not None and not self._confirm_discard_log():
            return
        self.session_logger.start(now=time.monotonic())
        self.log(f"데이터 기록 시작\n주기={self.session_logger.interval_s:.1f}초")
        self._refresh_logging_status(time.monotonic())

    def stop_logging(self) -> None:
        if not self.session_logger.recording:
            return
        now = time.monotonic()
        self.session_logger.stop(now=now)
        self.log(f"데이터 기록 중지\n기록시간={self.session_logger.elapsed(now):.1f}초\n"
                 f"기록행={self.session_logger.row_count}")
        self._refresh_logging_status(now)

    def save_csv(self) -> None:
        if self.session_logger.temp_path is None:
            self.log("CSV 저장 실패: 기록된 세션이 없습니다")
            return
        from tkinter import filedialog
        default_name = f"GRISE_drive_log_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
        destination = filedialog.asksaveasfilename(
            parent=self.root,
            title="주행 로그 CSV 저장",
            defaultextension=".csv",
            filetypes=(("CSV 파일", "*.csv"),),
            initialfile=default_name,
        )
        if not destination:
            return
        exported = self.session_logger.export_csv(destination)
        self.log(f"CSV 저장 완료:\n{exported}")

    def reset_logging(self) -> None:
        if self.session_logger.temp_path is not None and not self._confirm_discard_log():
            return
        self.session_logger.reset()
        self._refresh_logging_status(time.monotonic())
        self.log("데이터 기록 초기화")

    def _record_log_sample_if_due(self, now: float) -> None:
        if not self.session_logger.due(now=now):
            return
        telemetry = self.client.latest if self.client is not None else None
        telemetry_age_ms: float | None = None
        if telemetry is not None:
            telemetry_age_ms = max(0.0, now - telemetry.received_at) * 1000.0
            if telemetry_age_ms > TELEMETRY_TIMEOUT_S * 1000.0:
                telemetry = None
        diagnostics = (self.client.diagnostics() if self.client is not None else
                       {"sent": 0, "received": 0, "valid": 0, "invalid": 0})
        motion = self.motion_measurement.snapshot(now=now)
        row = build_log_snapshot(
            esp32_ip=(self.client.target[0] if self.client is not None
                      else self.ip_var.get().strip()),
            connected=self.client is not None,
            command=self.last_command,
            motion=motion,
            telemetry=telemetry,
            telemetry_age_ms=telemetry_age_ms,
            diagnostics=diagnostics,
        )
        self.session_logger.append_snapshot(row, now=now)

    def _refresh_logging_status(self, now: float) -> None:
        elapsed = int(self.session_logger.elapsed(now))
        hours, remainder = divmod(elapsed, 3600)
        minutes, seconds = divmod(remainder, 60)
        self.logging_status_var.set("● 기록 중" if self.session_logger.recording else "○ 대기")
        self.logging_elapsed_var.set(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        self.logging_rows_var.set(str(self.session_logger.row_count))
        self.logging_interval_var.set(f"{self.session_logger.interval_s:.1f} s")

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
        self._finish_motion("연결 해제")
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
            keys = self.control.pressed_keys()
            was_active = self.motion_measurement.snapshot(now=time.monotonic()).active
            self.motion_measurement.begin_or_update(
                vx=vx, vy=vy, w=w, label=keys, now=time.monotonic())
            if not was_active:
                self.log(f"주행 시작: {'+'.join(keys)}\nvx={vx:+.2f} vy={vy:+.2f} w={w:+.2f}")

    def _motion_release(self, key: str) -> None:
        before, _ = self.control.snapshot_state()
        self.control.release(key)
        after, _ = self.control.snapshot_state()
        if before == ControlState.RUNNING and after == ControlState.CONNECTED_STOPPED:
            self._finish_motion("키 해제")
        elif before == ControlState.RUNNING:
            vx, vy, w, _ = self.control.command(*self._speeds())
            self.motion_measurement.begin_or_update(
                vx=vx, vy=vy, w=w, label=self.control.pressed_keys(), now=time.monotonic())

    def stop(self) -> None:
        self.control.stop()
        self._finish_motion("정지")

    def estop(self) -> None:
        self.control.activate_estop()
        self._finish_motion("비상 정지")
        self.log("비상 정지 활성화")

    def release_estop(self) -> None:
        self.control.release_estop()
        self.log("비상 정지 해제; STOP 상태로 대기")

    def _finish_motion(self, reason: str) -> None:
        now = time.monotonic()
        snapshot = self.motion_measurement.snapshot(now=now)
        if not snapshot.active:
            return
        final = self.motion_measurement.finish(now=now)
        details = [f"주행 종료: {final.command_label}",
                   f"유지시간={final.elapsed_s:.2f}초"]
        if final.expected_distance_cm > 0.0:
            details.append(f"명령 기준 예상 이동거리={final.expected_distance_cm:.2f}cm")
        if final.expected_angle_rad > 0.0:
            details.append(f"명령 기준 예상 회전각={final.expected_angle_deg:.1f}°")
        self.log("\n".join(details) + f" ({reason})")

    def reset_measurement(self) -> None:
        self.motion_measurement.reset(now=time.monotonic())
        self._refresh_measurement(time.monotonic())

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
        estop_color = "#c62828" if state == ControlState.ESTOP else "#2e7d32"
        self.estop_widget.configure(bg=estop_color)
        status_label = STATE_LABEL_KO.get(state.value, "연결 안 됨")
        status_color = {ControlState.CONNECTED_STOPPED: "#2e7d32",
                        ControlState.RUNNING: "#2e7d32",
                        ControlState.TELEMETRY_LOST: "#ef6c00",
                        ControlState.ESTOP: "#c62828"}.get(state, "#9e9e9e")
        self.header_status_var.set(status_label)
        self.header_status_dot.configure(fg=status_color)
        self.header_ip_var.set(
            f"ESP32 IP {self.client.target[0]}" if self.client is not None
            else f"ESP32 IP {self.ip_var.get().strip() or '--'}")
        self.age_var.set("--" if received_at is None else f"{max(0.0, time.monotonic() - received_at) * 1000:.0f} ms")
        linear, angular = self._speeds()
        self.last_command = self.control.command(linear, angular)
        if state == ControlState.RUNNING:
            self.motion_measurement.begin_or_update(
                vx=self.last_command[0], vy=self.last_command[1], w=self.last_command[2],
                label=self.control.pressed_keys(), now=time.monotonic())
        else:
            self._finish_motion("안전 정지")
        self._refresh_measurement(time.monotonic())
        for variable, value in zip(self.expected_vars, expected_wheel_targets(*self.last_command[:3])):
            variable.set(f"{value:+.2f}")
        telemetry = self.client.latest if self.client is not None else None
        if self.client is not None:
            diagnostics = self.client.diagnostics()
            self.network_detail_var.set(
                f"UDP 송신 {diagnostics['sent']} · 수신 {diagnostics['received']} · "
                f"정상 {diagnostics['valid']} · 거부 {diagnostics['invalid']}")
        else:
            self.network_detail_var.set("UDP 송신 0 · 수신 0 · 정상 0 · 거부 0")
        if (self.client is not None and telemetry is None and self.client.started_at is not None and
                time.monotonic() - self.client.started_at > TELEMETRY_TIMEOUT_S):
            self.connection_var.set("텔레메트리 없음")
        if telemetry is not None:
            self.mode_var.set(MODE_LABEL_KO.get(telemetry.mode, telemetry.mode))
            self.status_var.set(STATUS_LABEL_KO.get(telemetry.status, telemetry.status))
            self.seq_var.set(str(telemetry.seq))
            self.rssi_var.set("--" if telemetry.rssi is None else f"{telemetry.rssi:g} dBm")
            self.header_rssi_var.set("Wi-Fi RSSI " + ("--" if telemetry.rssi is None else f"{telemetry.rssi:g} dBm"))
            for motor in range(3):
                target = telemetry.target_speed[motor]
                measured = telemetry.wheel_speed[motor]
                error = target - measured
                tracking = wheel_tracking_status(target, measured)
                pwm = telemetry.pwm[motor]
                display_values = (
                    f"{target:+.2f}", f"{measured:+.2f}", f"{error:+.2f}",
                    f"{telemetry.rpm[motor]:+.1f}",
                    "--" if pwm is None else f"{pwm:+d}",
                    str(int(telemetry.counts[motor])), tracking,
                )
                for variable, value in zip(self.motor_vars[motor], display_values):
                    variable.set(value)
                status_style = ("MotorDanger.TLabel" if tracking == "방향 이상" else
                                "MotorWarning.TLabel" if tracking in ("느림", "빠름") else
                                "MotorNormal.TLabel")
                self.motor_status_labels[motor].configure(style=status_style)
                self.balance_vars[motor].set(
                    f"M{motor + 1} |Measured| {abs(measured):.2f} cm/s")
            self.balance_spread_var.set(
                f"활성 바퀴 최대 속도 편차 {active_wheel_speed_spread(telemetry):.2f} cm/s")
        elif self.client is None:
            self.header_rssi_var.set("Wi-Fi RSSI --")
        logging_now = time.monotonic()
        self._record_log_sample_if_due(logging_now)
        self._refresh_logging_status(logging_now)
        self.root.after(100, self._refresh)

    def _refresh_measurement(self, now: float) -> None:
        snapshot = self.motion_measurement.snapshot(now=now)
        direction = {"Q": "Q · 반시계 방향", "E": "E · 시계 방향"}
        self.measurement_command_var.set(direction.get(snapshot.command_label, snapshot.command_label))
        self.measurement_elapsed_var.set(f"{snapshot.elapsed_s:.2f} s")
        self.measurement_speed_var.set(f"{snapshot.linear_speed_cm_s:.2f} cm/s")
        self.measurement_distance_var.set(f"{snapshot.expected_distance_cm:.2f} cm")
        self.measurement_angular_speed_var.set(f"{snapshot.angular_speed_rad_s:+.2f} rad/s")
        self.measurement_angle_var.set(f"{snapshot.expected_angle_deg:.1f}°")
        self.measurement_state_var.set("측정 중" if snapshot.active else "정지 / 측정 완료")

    def _callback_exception(self, exc_type: type[BaseException], exc: BaseException,
                            traceback: Any) -> None:
        self.control.activate_estop()
        self.log(f"GUI 오류: {exc}; 비상 정지 활성화")

    def close(self) -> None:
        self.control.activate_estop()
        self.disconnect()
        self.session_logger.stop(now=time.monotonic())
        self.session_logger.cleanup()
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
