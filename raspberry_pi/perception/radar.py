"""GRISE radar state boundary; fake JSON is not a TI UART frame."""
from __future__ import annotations

import json
import math
import socket
import time
from dataclasses import dataclass
from pathlib import Path

import config


@dataclass(frozen=True)
class RadarTarget:
    distance_cm: float
    angle_deg: float
    approach_speed_cm_s: float
    radial_velocity_cm_s: float | None = None  # normalized: positive is receding
    approaching: bool = False
    timestamp: float | None = None  # local monotonic receive time
    target_id: int | None = None

    def __post_init__(self) -> None:
        if self.radial_velocity_cm_s is None:
            object.__setattr__(self, "radial_velocity_cm_s", -self.approach_speed_cm_s)
        object.__setattr__(self, "approaching", self.approach_speed_cm_s > 0)

    @property
    def ttc_s(self) -> float | None:
        # Existing consumer property; E does not classify risk.
        return self.distance_cm / self.approach_speed_cm_s if self.approach_speed_cm_s > 1.0 else None


@dataclass(frozen=True)
class RadarState:
    connected: bool = False
    target: RadarTarget | None = None  # existing nearest-target interface
    age_s: float | None = None
    source: str = "disabled"
    valid: bool = False
    timestamp: float | None = None
    frame_id: int | None = None
    targets: tuple[RadarTarget, ...] = ()
    status: str = "OFFLINE"
    error: str | None = None

    @property
    def nearest_target(self) -> RadarTarget | None:
        return self.target


def _number(value, name: str, low: float, high: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    value = float(value)
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} out of range")
    return value


def parse_fake_packet(raw: bytes | str, *, received_at: float,
                      source: str = "udp_fake") -> RadarState:
    """Parse normalized GRISE JSON only; no TI binary layout is assumed."""
    if not math.isfinite(received_at):
        raise ValueError("invalid receive timestamp")
    data = json.loads(raw)
    if not isinstance(data, dict):
        raise ValueError("radar packet must be an object")
    frame_id = data.get("frame_id")
    if frame_id is not None and (type(frame_id) is not int or frame_id < 0):
        raise ValueError("invalid frame_id")
    entries = data.get("targets")
    if entries is None:
        entries = [] if data.get("target_present") is False else [data]
    if not isinstance(entries, list) or len(entries) > config.RADAR_MAX_TARGETS:
        raise ValueError("invalid targets list")
    targets = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("invalid target")
        distance = _number(entry.get("distance_cm"), "distance_cm", 0, config.RADAR_MAX_DISTANCE_CM)
        angle = _number(entry.get("angle_deg"), "angle_deg", -180, 180)
        approach = _number(entry.get("approach_speed_cm_s"), "approach_speed_cm_s",
                           -config.RADAR_MAX_RADIAL_SPEED_CM_S, config.RADAR_MAX_RADIAL_SPEED_CM_S)
        target_id = entry.get("target_id")
        if target_id is not None and (type(target_id) is not int or target_id < 0):
            raise ValueError("invalid target_id")
        targets.append(RadarTarget(distance, -180.0 if angle == 180 else angle,
                                   approach, -approach, approach > 0, received_at, target_id))
    nearest = min(targets, key=lambda target: target.distance_cm, default=None)
    return RadarState(True, nearest, 0.0, source, True, received_at, frame_id,
                      tuple(targets), "TARGET_DETECTED" if targets else "NO_TARGET")


class SerialFrameParser:
    """Boundary awaiting the active firmware format and actual byte fixtures."""

    def feed(self, data: bytes):
        raise NotImplementedError("SAMPLE SERIAL FRAME REQUIRED")


class RadarReceiver:
    """Nonblocking UDP fake transport; read() does not wait for a packet."""

    def __init__(self) -> None:
        self.backend = config.RADAR_BACKEND or ("udp_fake" if config.RADAR_ENABLED else "disabled")
        self._sock: socket.socket | None = None
        self._replay: RadarReplay | None = None
        self._latest = RadarState(source=self.backend)
        self._next_open_at = 0.0
        if self.backend == "udp_fake":
            self._open()
        elif self.backend == "replay":
            self._open_replay()
        elif self.backend not in ("disabled", "serial"):
            raise ValueError(f"unknown radar backend: {self.backend}")

    def _open_replay(self) -> None:
        try:
            if not config.RADAR_REPLAY_PATH:
                raise ValueError("GRISE_RADAR_REPLAY_PATH is required")
            self._replay = RadarReplay(config.RADAR_REPLAY_PATH)
        except (OSError, ValueError) as exc:
            self._latest = RadarState(source="replay", error=str(exc))
            self._next_open_at = time.monotonic() + config.RADAR_RECONNECT_S

    def _open(self) -> None:
        candidate = None
        try:
            candidate = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            candidate.bind((config.RADAR_BIND_HOST, config.RADAR_UDP_PORT))
            candidate.setblocking(False)
            self._sock = candidate
            print(f"[RADAR] fake UDP listening :{config.RADAR_UDP_PORT}")
        except OSError as exc:
            if candidate is not None:
                candidate.close()
            self._sock = None
            self._latest = RadarState(source=self.backend, error=str(exc))
            self._next_open_at = time.monotonic() + config.RADAR_RECONNECT_S

    def read(self) -> RadarState:
        now = time.monotonic()
        if self.backend == "serial":
            return RadarState(source="serial", error="SAMPLE SERIAL FRAME REQUIRED")
        if self.backend == "disabled":
            return RadarState()
        if self.backend == "replay":
            if self._replay is None and now >= self._next_open_at:
                self._open_replay()
            return self._replay.read() if self._replay is not None else self._latest
        if self._sock is None:
            if now >= self._next_open_at:
                self._open()
            if self._sock is None:
                return self._latest
        for _ in range(config.RADAR_MAX_PACKETS_PER_READ):
            try:
                raw, _ = self._sock.recvfrom(config.RADAR_MAX_PACKET_BYTES)
            except BlockingIOError:
                break
            except OSError as exc:
                self._sock.close()
                self._sock = None
                self._next_open_at = time.monotonic() + config.RADAR_RECONNECT_S
                self._latest = RadarState(source=self.backend, error=str(exc))
                break
            received_at = time.monotonic()
            try:
                self._latest = parse_fake_packet(raw, received_at=received_at, source=self.backend)
            except (ValueError, TypeError, UnicodeDecodeError, OverflowError) as exc:
                self._latest = RadarState(True, None, 0.0, self.backend, False,
                                          received_at, status="PARSE_ERROR", error=str(exc))
        latest = self._latest
        if latest.timestamp is None:
            return latest
        age = time.monotonic() - latest.timestamp
        if age > config.RADAR_STALE_S:
            return RadarState(False, None, age, self.backend, False, latest.timestamp,
                              latest.frame_id, status="DATA_STALE")
        return RadarState(latest.connected, latest.target, age, latest.source,
                          latest.valid, latest.timestamp, latest.frame_id,
                          latest.targets, latest.status, latest.error)

    def close(self) -> None:
        if self._replay is not None:
            self._replay.close()
            self._replay = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None


class RadarReplay:
    """Read one normalized fake JSON packet per line from an offline fixture."""

    def __init__(self, path: str | Path) -> None:
        self._file = Path(path).open("rb")

    def read(self) -> RadarState:
        raw = self._file.readline()
        if not raw:
            return RadarState(source="replay")
        now = time.monotonic()
        try:
            return parse_fake_packet(raw, received_at=now, source="replay")
        except (ValueError, TypeError, UnicodeDecodeError, OverflowError) as exc:
            return RadarState(True, None, 0.0, "replay", False, now,
                              status="PARSE_ERROR", error=str(exc))

    def close(self) -> None:
        self._file.close()
