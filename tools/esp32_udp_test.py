#!/usr/bin/env python3
"""Safely exercise the ESP32 typed velocity-control UDP path from Windows.

Example (raise all wheels before running):
    python tools/esp32_udp_test.py --ip 192.168.0.50

The script sends ``cmd_vel`` plus repeated typed ``stop`` packets. It does not
send ``cmd_move``, so ESP32 distance control is not engaged.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import math
import secrets
import socket
import sys
import time
from typing import Any


COMMAND_PORT = 8888
TELEMETRY_PORT = 8889
INITIAL_STOP_SECONDS = 1.0
FINAL_STOP_SECONDS = 1.0
DISPLAY_INTERVAL_SECONDS = 0.2
MAX_LINEAR_CM_S = 15.0
MAX_ANGULAR_RAD_S = 1.0
MAX_WHEEL_CM_S = 20.0
ROBOT_RADIUS_CM = 9.0
WHEEL_ANGLES_DEG = (0.0, 120.0, 240.0)


def finite_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError("must be finite")
    return result


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a bounded body-velocity test to the GRISE ESP32.")
    parser.add_argument("--ip", required=True, help="ESP32 IPv4 address")
    parser.add_argument("--vx", type=finite_float, default=10.0, help="forward cm/s")
    parser.add_argument("--vy", type=finite_float, default=0.0, help="left cm/s")
    parser.add_argument("--w", type=finite_float, default=0.0, help="CCW rad/s")
    parser.add_argument("--duration", type=finite_float, default=3.0, help="RUN seconds")
    parser.add_argument("--rate", type=finite_float, default=20.0, help="command Hz")
    args = parser.parse_args(argv)
    try:
        address = ipaddress.ip_address(args.ip)
    except ValueError as exc:
        parser.error(str(exc))
    if address.version != 4:
        parser.error("--ip must be an IPv4 address")
    if args.duration <= 0:
        parser.error("--duration must be positive")
    # 10 Hz leaves ample margin under the firmware's 300 ms watchdog.
    if args.rate < 10 or args.rate > 100:
        parser.error("--rate must be between 10 and 100 Hz")
    if math.hypot(args.vx, args.vy) > MAX_LINEAR_CM_S:
        parser.error("linear speed exceeds firmware limit (15 cm/s)")
    if abs(args.w) > MAX_ANGULAR_RAD_S:
        parser.error("angular speed exceeds firmware limit (1 rad/s)")
    return args


def build_command(session_id: int, seq: int, vx: float, vy: float, w: float,
                  status: str) -> dict[str, int | float | str]:
    if type(session_id) is not int or session_id <= 0:
        raise ValueError("session_id must be a nonzero integer")
    if status not in ("RUN", "SLOW", "STOP"):
        raise ValueError("invalid command status")
    base: dict[str, int | float | str] = {
        "type": "stop" if status == "STOP" else "cmd_vel",
        "session_id": session_id,
        "seq": seq,
    }
    if status != "STOP":
        base.update({"vx": vx, "vy": vy, "w": w, "status": status})
    return base


def expected_wheel_targets(vx: float, vy: float, w: float) -> tuple[float, ...]:
    targets = []
    for angle_deg in WHEEL_ANGLES_DEG:
        angle = math.radians(angle_deg)
        targets.append(-math.sin(angle) * vx + math.cos(angle) * vy +
                       ROBOT_RADIUS_CM * w)
    peak = max(abs(value) for value in targets)
    if peak > MAX_WHEEL_CM_S:
        scale = MAX_WHEEL_CM_S / peak
        targets = [value * scale for value in targets]
    return tuple(targets)


def validate_telemetry(payload: Any) -> dict[str, Any]:
    if not isinstance(payload, dict) or payload.get("type") != "telemetry":
        raise ValueError("unexpected telemetry payload")
    state = payload.get("state")
    status = ("RUN" if state in {"VELOCITY", "DISTANCE_ACTIVE", "DISTANCE_BRAKING"}
              else "STOP") if isinstance(state, str) else payload.get("status")
    if not isinstance(status, str) or not isinstance(payload.get("mode"), str):
        raise ValueError("telemetry status/mode missing")
    seq = payload.get("last_seq", payload.get("seq"))
    if type(seq) is not int:
        raise ValueError("telemetry seq must be an integer")
    normalized = dict(payload)
    normalized["seq"] = seq
    normalized["status"] = status
    aliases = {
        "counts": "encoder_count",
        "wheel_speed": "wheel_speed",
        "target_speed": "wheel_target",
    }
    for key, typed_key in aliases.items():
        values = payload.get(typed_key, payload.get(key))
        if not isinstance(values, list) or len(values) != 3:
            raise ValueError(f"{key} must contain three values")
        if not all(type(value) in (int, float) and math.isfinite(value) for value in values):
            raise ValueError(f"{key} contains a non-finite value")
        normalized[key] = values
    rpm = payload.get("rpm")
    if not isinstance(rpm, list) or len(rpm) != 3 or not all(
            type(value) in (int, float) and math.isfinite(value) for value in rpm):
        raise ValueError("rpm must contain three finite values")
    normalized["rpm"] = rpm
    pwm = payload.get("wheel_pwm", payload.get("pwm"))
    if pwm is not None:
        if not isinstance(pwm, list) or len(pwm) != 3 or not all(
                type(value) is int and -255 <= value <= 255 for value in pwm):
            raise ValueError("pwm must contain three signed integers")
        normalized["pwm"] = pwm
    return normalized


def format_telemetry(payload: dict[str, Any]) -> str:
    lines = ["-" * 64,
             f"status : {payload['status']}",
             f"mode   : {payload['mode']}",
             f"state  : {payload.get('state', '--')}",
             f"fault  : {payload.get('fault', '--')}",
             f"seq    : {payload['seq']}",
             f"counts : {payload['counts']}"]
    for index in range(3):
        lines.append(
            f"M{index + 1} target: {payload['target_speed'][index]:+8.2f} cm/s | "
            f"measured: {payload['wheel_speed'][index]:+8.2f} cm/s | "
            f"rpm: {payload['rpm'][index]:+8.2f}")
    lines.append("-" * 64)
    return "\n".join(lines)


class VelocityTest:
    def __init__(self, ip: str, rate: float) -> None:
        self.target = (ip, COMMAND_PORT)
        self.ip = ip
        self.period = 1.0 / rate
        self.session_id = secrets.randbits(32) or 1
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", TELEMETRY_PORT))
        self.sock.setblocking(False)
        self.last_telemetry: dict[str, Any] | None = None
        self.last_display_at = 0.0

    def close(self) -> None:
        self.sock.close()

    def send(self, vx: float, vy: float, w: float, status: str) -> None:
        self.seq += 1
        packet = build_command(self.session_id, self.seq, vx, vy, w, status)
        wire = json.dumps(packet, separators=(",", ":"), allow_nan=False).encode("utf-8")
        self.sock.sendto(wire, self.target)

    def receive_available(self, *, display: bool = True) -> bool:
        received = False
        while True:
            try:
                wire, sender = self.sock.recvfrom(8192)
            except BlockingIOError:
                break
            if sender[0] != self.ip:
                continue
            try:
                payload = validate_telemetry(json.loads(wire.decode("utf-8")))
            except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                print(f"[TEL] rejected packet: {exc}", file=sys.stderr)
                continue
            self.last_telemetry = payload
            received = True
        now = time.monotonic()
        if display and self.last_telemetry is not None and now - self.last_display_at >= DISPLAY_INTERVAL_SECONDS:
            print(format_telemetry(self.last_telemetry))
            self.last_display_at = now
        return received

    def phase(self, duration: float, vx: float, vy: float, w: float,
              status: str, *, display: bool = True) -> None:
        deadline = time.monotonic() + duration
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= next_send:
                self.send(vx, vy, w, status)
                next_send += self.period
                if next_send <= now:
                    next_send = now + self.period
            self.receive_available(display=display)
            time.sleep(min(0.01, max(0.0, next_send - time.monotonic())))

    def safe_stop(self, duration: float = FINAL_STOP_SECONDS) -> None:
        self.phase(duration, 0.0, 0.0, 0.0, "STOP", display=False)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    expected = expected_wheel_targets(args.vx, args.vy, args.w)
    print(f"ESP32 command target: {args.ip}:{COMMAND_PORT}")
    print(f"Telemetry listener : 0.0.0.0:{TELEMETRY_PORT}")
    print("Expected firmware IK targets:")
    for index, value in enumerate(expected, 1):
        print(f"  M{index}: {value:+.2f} cm/s")

    test: VelocityTest | None = None
    try:
        test = VelocityTest(args.ip, args.rate)
        print("[1/3] Sending STOP for sequence resynchronization...")
        test.phase(INITIAL_STOP_SECONDS, 0.0, 0.0, 0.0, "STOP")
        # Drain a packet that may have arrived at the end of the STOP phase.
        test.receive_available()
        if test.last_telemetry is None:
            print("[ABORT] No valid ESP32 telemetry received; RUN was not sent.", file=sys.stderr)
            return 2
        print(f"[2/3] Telemetry confirmed; sending RUN for {args.duration:.2f}s...")
        test.phase(args.duration, args.vx, args.vy, args.w, "RUN")
        print("[3/3] Sending final STOP...")
        test.safe_stop()
        return 0
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Ctrl+C received; sending STOP...", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}; sending STOP...", file=sys.stderr)
        return 1
    finally:
        if test is not None:
            try:
                test.safe_stop()
            except OSError as exc:
                print(f"[STOP] UDP STOP send failed: {exc}", file=sys.stderr)
            test.close()


if __name__ == "__main__":
    raise SystemExit(main())
