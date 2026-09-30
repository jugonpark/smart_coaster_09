import json
import socket
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

from comm.udp_sender import UdpSender
from comm.telemetry_receiver import TelemetryReceiver
import config
from core.motion_goal import MotionGoals


def telemetry(**kw):
    values = dict(received=True, age_s=0.01, boot_id=7, status="RUN",
                  motion_id=None, goal_active=True, goal_reached=False)
    values.update(kw)
    return SimpleNamespace(**values)


class MotionGoalTests(unittest.TestCase):
    def test_optional_telemetry_and_invalid_extension(self):
        old_host, old_port = config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT
        config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = "127.0.0.1", 0
        rx = TelemetryReceiver()
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        base = {"type": "telemetry", "seq": 1, "status": "STOP",
                "counts": [0, 0, 0], "rpm": [0, 0, 0],
                "wheel_speed": [0, 0, 0], "target_speed": [0, 0, 0]}
        try:
            addr = rx._sock.getsockname()
            tx.sendto(json.dumps({**base, "boot_id": 7, "motion_id": 11,
                                  "goal_reached": True, "goal_progress_cm": 30.0}).encode(), addr)
            deadline = time.monotonic() + 1
            while not rx.latest().received and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual((rx.latest().motion_id, rx.latest().goal_progress_cm), (11, 30))
            self.assertTrue(rx.latest().goal_reached)
            tx.sendto(json.dumps({**base, "seq": 2, "goal_progress_cm": float("nan")}).encode(), addr)
            time.sleep(0.04)
            self.assertEqual(rx.latest().seq, 1)
        finally:
            tx.close()
            rx.close()
            config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = old_host, old_port

    def test_stable_id_completion_and_fresh_replan(self):
        goals = MotionGoals(initial_id=100)
        command = SimpleNamespace(status="RUN")
        plan = SimpleNamespace(direction="BACK", target_distance_cm=30)
        self.assertIsNone(goals.select(command, plan, telemetry(), 1))  # STOP handshake
        first = goals.select(command, plan, telemetry(), 1)
        self.assertEqual(first, (101, 30))
        self.assertEqual(goals.select(command, plan, telemetry(), 2), first)
        reached = telemetry(motion_id=101, goal_reached=True)
        self.assertIsNone(goals.select(command, plan, reached, 2))
        self.assertIsNone(goals.select(command, plan, reached, 2))
        self.assertEqual(goals.select(command, plan, reached, 3), (102, 30))

    def test_stop_and_boot_change_require_new_goal(self):
        goals = MotionGoals(initial_id=9)
        run = SimpleNamespace(status="SLOW")
        stop = SimpleNamespace(status="STOP")
        plan = SimpleNamespace(direction="LEFT", target_distance_cm=15)
        self.assertIsNone(goals.select(run, plan, telemetry(), 1))
        self.assertEqual(goals.select(run, plan, telemetry(), 1), (10, 15))
        self.assertIsNone(goals.select(stop, plan, telemetry(), 1))
        self.assertEqual(goals.select(run, plan, telemetry(), 2), (11, 15))
        self.assertIsNone(goals.select(run, plan, telemetry(boot_id=8), 2))
        self.assertEqual(goals.select(run, plan, telemetry(boot_id=8), 3), (12, 15))

    def test_esp_cancel_requires_stop_and_new_snapshot(self):
        goals = MotionGoals(initial_id=20)
        run = SimpleNamespace(status="RUN")
        plan = SimpleNamespace(direction="BACK", target_distance_cm=30)
        goals.select(run, plan, telemetry(), 1)
        self.assertEqual(goals.select(run, plan, telemetry(), 1), (21, 30))
        canceled = telemetry(motion_id=21, goal_active=False, status="STOP")
        self.assertIsNone(goals.select(run, plan, canceled, 1))
        self.assertIsNone(goals.select(run, plan, canceled, 1))
        self.assertEqual(goals.select(run, plan, canceled, 2), (22, 30))

    def test_typed_stop_and_distance_wire_semantics_are_separate(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(1)
        tx = UdpSender("127.0.0.1", rx.getsockname()[1])
        try:
            tx.send(0, 0, 0, "STOP", force=True)
            self.assertEqual(set(json.loads(rx.recv(4096))),
                             {"type", "session_id", "seq"})
            tx.send(-15, 0, 0, "SLOW", force=True,
                    motion_id=17, target_distance_cm=15)
            packet = json.loads(rx.recv(4096))
            self.assertEqual(packet["type"], "cmd_move")
            self.assertEqual((packet["motion_id"], packet["target_distance_cm"]), (17, 15))
        finally:
            tx.close()
            rx.close()


if __name__ == "__main__":
    unittest.main()
