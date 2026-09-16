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
    encoder_counts: list[int] = field(default_factory=lambda: [0, 0, 0])
    rpm: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    wheel_speed_cm_s: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    target_speed_cm_s: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rssi: int | None = None
    boot_id: int | None = None
    motion_id: int | None = None
    goal_active: bool = False
    goal_reached: bool = False
    goal_progress_cm: float | None = None
    target_distance_cm: float | None = None
    lateral_error_cm: float | None = None
    action_dtheta_rad: float | None = None


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
                d = json.loads(raw.decode("utf-8"))
                if not isinstance(d, dict) or d.get("type") != "telemetry":
                    raise ValueError("unexpected telemetry payload")
                arrays = [d.get(key) for key in ("counts", "rpm", "wheel_speed", "target_speed")]
                if any(not isinstance(values, list) or len(values) != 3 for values in arrays):
                    raise ValueError("telemetry wheel arrays must have three entries")
                if not all(isinstance(v, (int, float)) and math.isfinite(v)
                           for values in arrays for v in values):
                    raise ValueError("non-finite telemetry value")
                for key in ("boot_id", "motion_id"):
                    value = d.get(key)
                    if value is not None and (type(value) is not int or value < 0):
                        raise ValueError(f"invalid {key}")
                for key in ("goal_active", "goal_reached"):
                    if key in d and type(d[key]) is not bool:
                        raise ValueError(f"invalid {key}")
                for key in ("goal_progress_cm", "target_distance_cm",
                            "lateral_error_cm", "action_dtheta_rad"):
                    value = d.get(key)
                    if value is not None and (type(value) not in (int, float) or
                                              not math.isfinite(value)):
                        raise ValueError(f"invalid {key}")
                t = TelemetryState(
                    received=True,
                    seq=int(d.get("seq", -1)),
                    status=str(d.get("status", "UNKNOWN")),
                    encoder_counts=[int(x) for x in d.get("counts", [0,0,0])],
                    rpm=[float(x) for x in d.get("rpm", [0,0,0])],
                    wheel_speed_cm_s=[float(x) for x in d.get("wheel_speed", [0,0,0])],
                    target_speed_cm_s=[float(x) for x in d.get("target_speed", [0,0,0])],
                    rssi=d.get("rssi"),
                    boot_id=d.get("boot_id"), motion_id=d.get("motion_id"),
                    goal_active=d.get("goal_active", False),
                    goal_reached=d.get("goal_reached", False),
                    goal_progress_cm=d.get("goal_progress_cm"),
                    target_distance_cm=d.get("target_distance_cm"),
                    lateral_error_cm=d.get("lateral_error_cm"),
                    action_dtheta_rad=d.get("action_dtheta_rad"),
                )
                with self._lock:
                    self._latest = t
                    self._last_rx = time.time()
            except socket.timeout:
                continue
            except (OSError, ValueError, TypeError, OverflowError) as exc:
                if self._running:
                    print(f"[TEL] rejected packet: {exc}")
                continue

    def latest(self) -> TelemetryState:
        with self._lock:
            x = TelemetryState(**{k: getattr(self._latest, k) for k in self._latest.__dataclass_fields__})
            if self._last_rx:
                x.age_s = time.time() - self._last_rx
            return x

    def close(self) -> None:
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
