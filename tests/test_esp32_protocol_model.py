import math
from dataclasses import dataclass, replace
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino" / "esp32_omni_controller"
HEADER = FIRMWARE / "network_protocol.h"
SOURCE = FIRMWARE / "network_protocol.cpp"


@dataclass(frozen=True)
class Command:
    kind: str
    session: int
    seq: int
    status: str = "RUN"
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    motion_id: int = -1
    distance: float = 0.0


class AcceptanceModel:
    """Independent executable contract for the firmware acceptance layer."""

    def __init__(self):
        self.session = None
        self.last_seq = None
        self.handshake = False
        self.moving = False
        self.mailbox = None
        self.revision = 0
        self.last_seen = None

    def accept(self, command):
        if command.session <= 0:
            return "FAULT"
        if self.session != command.session:
            self.session = command.session
            self.last_seq = command.seq
            self.handshake = not self.moving
            if self.moving:
                self.moving = False
                self.mailbox = None
                return "IMMEDIATE_STOP"
        elif self.last_seq is not None and command.seq <= self.last_seq:
            return "IGNORE"
        else:
            self.last_seq = command.seq

        self.last_seen = command.kind
        if command.kind in {"stop", "heartbeat"}:
            self.handshake = True
            if command.kind == "stop":
                self.moving = False
                self.mailbox = None
                return "IMMEDIATE_STOP"
            return "ACCEPTED_LIVENESS"
        if command.kind in {"cmd_vel", "cmd_move"}:
            if not self.handshake:
                return "NEEDS_HANDSHAKE"
            self.revision += 1
            self.mailbox = (self.revision, command)
            self.moving = True
            return "ACCEPTED_MOTION"
        return "ACCEPTED_RESET"


def normalize(packet):
    if "type" in packet:
        kind = packet["type"]
        session = packet.get("session_id")
    else:
        has_id = "motion_id" in packet
        has_distance = "target_distance_cm" in packet
        if has_id != has_distance:
            raise ValueError("partial legacy distance command")
        kind = "stop" if packet.get("status") == "STOP" else ("cmd_move" if has_id else "cmd_vel")
        session = 0x4C454741
    if kind not in {"cmd_vel", "cmd_move", "stop", "heartbeat", "reset_fault"}:
        raise ValueError("unknown command type")
    if not isinstance(session, int) or isinstance(session, bool) or session == 0:
        raise ValueError("invalid session")
    seq = packet.get("seq")
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        raise ValueError("invalid sequence")
    numbers = [packet.get(name, 0.0) for name in ("vx", "vy", "w")]
    if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in numbers):
        raise ValueError("invalid motion value")
    if math.hypot(numbers[0], numbers[1]) > 30.0 or abs(numbers[2]) > 2.0:
        raise ValueError("hard speed violation")
    return Command(
        kind, session, seq, packet.get("status", "RUN"), *numbers,
        packet.get("motion_id", -1), packet.get("target_distance_cm", 0.0)
    )


def test_typed_and_legacy_packets_normalize_to_one_command_shape():
    typed = normalize({"type": "cmd_move", "session_id": 9, "seq": 2, "status": "RUN", "vx": 10, "vy": 0, "w": 0, "motion_id": 4, "target_distance_cm": 30})
    legacy = normalize({"seq": 2, "status": "RUN", "vx": 10, "vy": 0, "w": 0, "motion_id": 4, "target_distance_cm": 30})
    assert replace(typed, session=legacy.session) == legacy


@pytest.mark.parametrize("session", [None, 0, 1.5, True])
def test_typed_session_id_must_be_nonzero_integer(session):
    with pytest.raises(ValueError):
        normalize({"type": "stop", "session_id": session, "seq": 1})


def test_partial_legacy_distance_and_nonfinite_or_hard_limit_values_are_rejected():
    with pytest.raises(ValueError):
        normalize({"seq": 1, "status": "RUN", "vx": 1, "vy": 0, "w": 0, "motion_id": 2})
    for value in (math.nan, math.inf, 31.0):
        with pytest.raises(ValueError):
            normalize({"type": "cmd_vel", "session_id": 1, "seq": 1, "vx": value, "vy": 0, "w": 0})


def test_duplicate_and_stale_sequences_are_ignored_after_first_acceptance():
    model = AcceptanceModel()
    assert model.accept(Command("stop", 10, 8)) == "IMMEDIATE_STOP"
    assert model.accept(Command("cmd_vel", 10, 9, vx=5)) == "ACCEPTED_MOTION"
    assert model.accept(Command("cmd_vel", 10, 9, vx=8)) == "IGNORE"
    assert model.accept(Command("cmd_vel", 10, 7, vx=8)) == "IGNORE"
    assert model.mailbox[1].vx == 5


def test_heartbeat_refreshes_liveness_without_replacing_motion_mailbox():
    model = AcceptanceModel()
    model.accept(Command("stop", 10, 1))
    motion = Command("cmd_vel", 10, 2, vx=6)
    model.accept(motion)
    before = model.mailbox
    assert model.accept(Command("heartbeat", 10, 3)) == "ACCEPTED_LIVENESS"
    assert model.mailbox == before


def test_latest_motion_wins_across_velocity_and_distance_types():
    model = AcceptanceModel()
    model.accept(Command("stop", 10, 1))
    model.accept(Command("cmd_vel", 10, 2, vx=4))
    model.accept(Command("cmd_move", 10, 3, vx=5, motion_id=7, distance=15))
    assert model.mailbox[0] == 2
    assert model.mailbox[1].kind == "cmd_move"


def test_new_session_while_moving_stops_then_requires_handshake_and_retry():
    model = AcceptanceModel()
    model.accept(Command("stop", 10, 1))
    model.accept(Command("cmd_vel", 10, 2, vx=4))
    assert model.accept(Command("cmd_vel", 20, 1, vx=7)) == "IMMEDIATE_STOP"
    assert model.mailbox is None
    assert model.accept(Command("cmd_vel", 20, 2, vx=7)) == "NEEDS_HANDSHAKE"
    assert model.accept(Command("heartbeat", 20, 3)) == "ACCEPTED_LIVENESS"
    assert model.accept(Command("cmd_vel", 20, 4, vx=7)) == "ACCEPTED_MOTION"


def test_cpp_contract_has_decode_acceptance_separation_and_single_mailbox_slot():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    assert "enum class CommandType" in header
    assert "struct NormalizedCommand" in header
    assert "struct CommandMailbox" in header
    assert "NormalizedCommand command" in header
    assert "localRevision" in header
    assert "std::queue" not in header + source
    assert "decodePacket" in source
    assert "acceptCommand" in source
    assert source.index("DecodeResult decodePacket") < source.index("AcceptanceResult acceptCommand")
    assert "LEGACY_SESSION_ID" in source
