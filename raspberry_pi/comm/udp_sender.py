"""Nonblocking typed UDP command transport for the ESP32 controller."""

from __future__ import annotations

import json
import secrets
import socket
import time
from dataclasses import dataclass

import config


@dataclass(frozen=True)
class CommandPacket:
    type: str
    session_id: int
    seq: int
    vx: float | None = None
    vy: float | None = None
    w: float | None = None
    status: str | None = None
    motion_id: int | None = None
    target_distance_cm: float | None = None

    def to_json(self) -> str:
        payload = {
            key: value for key, value in self.__dict__.items()
            if value is not None
        }
        for key in ("vx", "vy", "w", "target_distance_cm"):
            if key in payload:
                payload[key] = round(float(payload[key]), 3)
        return json.dumps(payload, separators=(",", ":"), allow_nan=False)


class UdpSender:
    def __init__(self, ip: str | None = None, port: int | None = None) -> None:
        self.addr = (ip or config.ESP32_IP, port or config.ESP32_PORT)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setblocking(False)
        self.session_id = secrets.randbits(32) or 1
        self._seq = 0
        self._min_interval = 1.0 / config.UDP_SEND_HZ
        self._last_send_t = 0.0
        self.sent_count = 0
        self.error_count = 0
        print(f"[UDP] 대상 {self.addr[0]}:{self.addr[1]}, {config.UDP_SEND_HZ}Hz")

    def _next(self, command_type: str, **values) -> CommandPacket:
        self._seq += 1
        return CommandPacket(
            type=command_type, session_id=self.session_id, seq=self._seq,
            **values,
        )

    def _transmit(self, packet: CommandPacket, *, force: bool) -> CommandPacket | None:
        now = time.monotonic()
        if not force and (now - self._last_send_t) < self._min_interval:
            return None
        try:
            self._sock.sendto(packet.to_json().encode("utf-8"), self.addr)
            self.sent_count += 1
        except (BlockingIOError, OSError) as exc:
            self.error_count += 1
            if self.error_count % 60 == 1:
                print(f"[UDP] 송신 실패 ({self.error_count}회): {exc}")
        self._last_send_t = now
        return packet

    def send(self, vx: float, vy: float, w: float, status: str,
             force: bool = False, *, motion_id: int | None = None,
             target_distance_cm: float | None = None) -> CommandPacket | None:
        if status not in {"RUN", "SLOW", "STOP"}:
            raise ValueError("invalid command status")
        if (motion_id is None) != (target_distance_cm is None):
            raise ValueError("motion_id and target_distance_cm must be supplied together")
        if status == "STOP":
            packet = self._next("stop")
        elif motion_id is not None:
            packet = self._next(
                "cmd_move", vx=float(vx), vy=float(vy), w=float(w),
                status=status, motion_id=motion_id,
                target_distance_cm=float(target_distance_cm),
            )
        else:
            packet = self._next(
                "cmd_vel", vx=float(vx), vy=float(vy), w=float(w),
                status=status,
            )
        return self._transmit(packet, force=force)

    def send_heartbeat(self, *, force: bool = False) -> CommandPacket | None:
        return self._transmit(self._next("heartbeat"), force=force)

    def send_stop(self) -> None:
        """Send repeated immediate STOP packets to tolerate UDP loss."""
        for _ in range(5):
            self.send(0.0, 0.0, 0.0, "STOP", force=True)
            time.sleep(0.01)

    def close(self) -> None:
        self.send_stop()
        self._sock.close()
        print(f"[UDP] 종료. 전송 {self.sent_count}건, 실패 {self.error_count}건")
