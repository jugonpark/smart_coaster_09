from __future__ import annotations

import json
import math
import socket
import threading
import time
from dataclasses import dataclass, field

import config


@dataclass
class TelemetryState:
    received: bool = False
    age_s: float | None = None
    seq: int = -1
    status: str = "UNKNOWN"
    mode: str = "UNKNOWN"
    state: str = "UNKNOWN"
    fault: str = "NONE"
    session_id: int | None = None
    encoder_counts: list[int] = field(default_factory=lambda: [0, 0, 0])
    rpm: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    wheel_speed_cm_s: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_speed_cm_s: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    wheel_pwm: list[int] | None = None
    rssi: int | None = None
    boot_id: int | None = None
    motion_id: int | None = None
    goal_active: bool = False
    goal_reached: bool = False
    goal_progress_cm: float | None = None
    target_distance_cm: float | None = None
    lateral_error_cm: float | None = None
    action_dtheta_rad: float | None = None


def _finite_array(payload: dict, typed: str, legacy: str | None = None) -> list[float]:
    values = payload.get(typed)
    if values is None and legacy is not None:
        values = payload.get(legacy)
    if not isinstance(values, list) or len(values) != 3:
        raise ValueError(f"{typed} must have three entries")
    if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
        raise ValueError(f"{typed} contains a non-finite value")
    return [float(value) for value in values]


def parse_telemetry(payload: dict) -> TelemetryState:
    if not isinstance(payload, dict) or payload.get("type") != "telemetry":
        raise ValueError("unexpected telemetry payload")
    counts = _finite_array(payload, "encoder_count", "counts")
    speeds = _finite_array(payload, "wheel_speed")
    targets = _finite_array(payload, "wheel_target", "target_speed")
    if payload.get("rpm") is None:
        rpm = [speed * 60.0 / (2.0 * math.pi * 2.9) for speed in speeds]
    else:
        rpm = _finite_array(payload, "rpm")
    pwm = payload.get("wheel_pwm", payload.get("pwm"))
    if pwm is not None and (
        not isinstance(pwm, list) or len(pwm) != 3 or
        not all(type(value) is int and -255 <= value <= 255 for value in pwm)
    ):
        raise ValueError("wheel_pwm must contain three signed integers")

    for key in ("boot_id", "session_id", "motion_id"):
        value = payload.get(key)
        if value is not None and (type(value) is not int or value < 0):
            raise ValueError(f"invalid {key}")
    numeric_aliases = {
        "goal_progress_cm": "goal_progress_cm",
        "target_distance_cm": "goal_target_cm",
        "lateral_error_cm": "lateral_error_cm",
        "action_dtheta_rad": "odom_dtheta_rad",
    }
    optional: dict[str, float | None] = {}
    for destination, source in numeric_aliases.items():
        value = payload.get(source, payload.get(destination))
        if value is not None and (type(value) not in (int, float) or not math.isfinite(value)):
            raise ValueError(f"invalid {source}")
        optional[destination] = None if value is None else float(value)

    state = str(payload.get("state", payload.get("status", "UNKNOWN")))
    status = ("RUN" if state in {"VELOCITY", "DISTANCE_ACTIVE", "DISTANCE_BRAKING"}
              else "STOP") if "state" in payload else str(payload.get("status", "UNKNOWN"))
    seq = payload.get("last_seq", payload.get("seq", -1))
    if type(seq) is not int:
        raise ValueError("invalid sequence")
    goal_active = state in {"DISTANCE_ACTIVE", "DISTANCE_BRAKING"}
    goal_reached = state == "GOAL_REACHED"
    if "goal_active" in payload:
        if type(payload["goal_active"]) is not bool:
            raise ValueError("invalid goal_active")
        goal_active = payload["goal_active"]
    if "goal_reached" in payload:
        if type(payload["goal_reached"]) is not bool:
            raise ValueError("invalid goal_reached")
        goal_reached = payload["goal_reached"]

    rssi = payload.get("wifi_rssi", payload.get("rssi"))
    if rssi is not None and (type(rssi) not in (int, float) or not math.isfinite(rssi)):
        raise ValueError("invalid RSSI")
    return TelemetryState(
        received=True, seq=seq, status=status,
        mode=str(payload.get("mode", "UNKNOWN")), state=state,
        fault=str(payload.get("fault", "NONE")),
        session_id=payload.get("session_id"),
        encoder_counts=[int(value) for value in counts], rpm=rpm,
        wheel_speed_cm_s=speeds, target_speed_cm_s=targets,
        wheel_pwm=None if pwm is None else list(pwm), rssi=rssi,
        boot_id=payload.get("boot_id"), motion_id=payload.get("motion_id"),
        goal_active=goal_active, goal_reached=goal_reached,
        goal_progress_cm=optional["goal_progress_cm"],
        target_distance_cm=optional["target_distance_cm"],
        lateral_error_cm=optional["lateral_error_cm"],
        action_dtheta_rad=optional["action_dtheta_rad"],
    )


class TelemetryReceiver:
    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT))
        self._sock.settimeout(0.5)
        self._lock = threading.Lock()
        self._latest = TelemetryState()
        self._last_rx = 0.0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print(f"[TEL] listening :{config.TELEMETRY_PORT}")

    def _loop(self) -> None:
        while self._running:
            try:
                raw, _ = self._sock.recvfrom(8192)
                state = parse_telemetry(json.loads(raw.decode("utf-8")))
                with self._lock:
                    self._latest = state
                    self._last_rx = time.time()
            except socket.timeout:
                continue
            except (json.JSONDecodeError, UnicodeDecodeError, OSError, ValueError,
                    TypeError, OverflowError) as exc:
                if self._running:
                    print(f"[TEL] rejected packet: {exc}")

    def latest(self) -> TelemetryState:
        with self._lock:
            state = TelemetryState(**{
                key: getattr(self._latest, key)
                for key in self._latest.__dataclass_fields__
            })
            if self._last_rx:
                state.age_s = time.time() - self._last_rx
            return state

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
