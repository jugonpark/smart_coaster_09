from __future__ import annotations

import json
import socket
import time

import config


class MonitorSender:
    def __init__(self) -> None:
        self.addr = (config.MONITOR_IP, config.MONITOR_PORT)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.min_interval = 1.0 / config.MONITOR_SEND_HZ
        self.last_send = 0.0

    def send(self, *, world, risk, plan, command, telemetry, camera_ok: bool) -> None:
        now = time.time()
        if now - self.last_send < self.min_interval:
            return
        target = world.radar_target
        threat = getattr(world, "threat", None)
        radar_txt = "DISCONNECTED" if not world.radar_connected else (
            f"{target.distance_cm:.0f}cm {target.approach_speed_cm_s:.0f}cm/s" if target else "NO_TARGET"
        )
        msg = {
            "t": round(now, 3),
            "state": "AVOID" if command.status != "STOP" else "WATCH/STOP",
            "camera": "ONLINE" if camera_ok else "OFFLINE",
            "robot": "TRACKED" if world.robot.detected else "LOST",
            "risk": risk.level,
            "radar": radar_txt,
            "escape": f"{plan.direction} {plan.target_distance_cm:.0f}cm",
            "command": f"{command.status} vx={command.vx:.1f} vy={command.vy:.1f} w={command.w:.2f}",
            "esp32": "ONLINE" if telemetry.received and telemetry.age_s is not None and telemetry.age_s <= config.TELEMETRY_STALE_S else "OFFLINE",
            "blocked": {
                "front": world.blocked.front, "front_left": world.blocked.front_left,
                "left": world.blocked.left, "back_left": world.blocked.back_left,
                "rear": world.blocked.rear, "back_right": world.blocked.back_right,
                "right": world.blocked.right, "front_right": world.blocked.front_right,
            },
            "reason": command.reason,
            "risk_reason": getattr(risk, "reason", None),
            "vision_valid": getattr(world, "vision_valid", None),
            "radar_valid": getattr(world, "radar_valid", None),
            "threat_detected": getattr(threat, "detected", None),
            "threat_direction": getattr(threat, "direction", None),
            "threat_distance_cm": getattr(threat, "distance_cm", None),
            "threat_speed_cm_s": getattr(threat, "approach_speed_cm_s", None),
            "ttc_s": getattr(threat, "ttc_s", None),
            "escape_valid": getattr(plan, "valid", None),
            "escape_direction": getattr(plan, "direction", None),
            "escape_speed_cm_s": getattr(plan, "speed_cm_s", None),
            "escape_target_distance_cm": getattr(plan, "target_distance_cm", None),
            "candidate_directions": getattr(plan, "candidate_directions", ()),
            "blocked_directions": getattr(plan, "blocked_directions", ()),
            "safety_reason": command.reason,
        }
        self._send(msg, now)

    def send_camera_offline(self) -> None:
        self.send_unavailable("camera offline", camera_ok=False)

    def send_unavailable(self, reason: str, *, camera_ok: bool) -> None:
        now = time.time()
        if now - self.last_send < self.min_interval:
            return
        self._send({
            "t": round(now, 3), "state": "WATCH/STOP",
            "camera": "ONLINE" if camera_ok else "OFFLINE",
            "robot": "LOST", "risk": "UNKNOWN", "radar": "UNKNOWN",
            "escape": "NONE 0cm", "command": "STOP vx=0.0 vy=0.0 w=0.00",
            "esp32": "UNKNOWN", "blocked": {}, "reason": reason,
        }, now)

    def _send(self, msg: dict, now: float) -> None:
        try:
            self.sock.sendto(json.dumps(msg, separators=(",", ":")).encode(), self.addr)
            self.last_send = now
        except OSError as exc:
            print(f"[MONITOR] send failed: {exc}")

    def close(self) -> None:
        self.sock.close()
