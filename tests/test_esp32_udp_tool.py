import importlib.util
import json
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "esp32_udp_test.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("esp32_udp_test", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class Esp32UdpToolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tool = load_tool()

    def test_command_matches_typed_velocity_protocol(self):
        packet = self.tool.build_command(123, 7, 10.0, 0.0, 0.0, "RUN")
        self.assertEqual(set(packet), {"type", "session_id", "seq", "vx", "vy", "w", "status"})
        self.assertEqual(packet,
                         {"type": "cmd_vel", "session_id": 123, "seq": 7,
                          "vx": 10.0, "vy": 0.0,
                          "w": 0.0, "status": "RUN"})
        self.assertNotIn("motion_id", json.dumps(packet))

    def test_stop_packet_is_minimal_and_session_is_stable(self):
        test = self.tool.VelocityTest.__new__(self.tool.VelocityTest)
        test.session_id = 77
        test.seq = 0
        first = self.tool.build_command(test.session_id, 1, 0, 0, 0, "STOP")
        second = self.tool.build_command(test.session_id, 2, 1, 0, 0, "RUN")
        self.assertEqual(first, {"type": "stop", "session_id": 77, "seq": 1})
        self.assertEqual(second["session_id"], 77)

    def test_expected_targets_match_firmware_inverse_kinematics(self):
        targets = self.tool.expected_wheel_targets(10.0, 0.0, 0.0)
        self.assertAlmostEqual(targets[0], 0.0, places=6)
        self.assertAlmostEqual(targets[1], -5.0 * math.sqrt(3), places=5)
        self.assertAlmostEqual(targets[2], 5.0 * math.sqrt(3), places=5)

    def test_telemetry_requires_actual_three_wheel_fields(self):
        valid = {"type": "telemetry", "seq": 9, "status": "RUN", "mode": "NETWORK",
                 "counts": [1, 2, 3], "rpm": [4, 5, 6],
                 "wheel_speed": [7, 8, 9], "target_speed": [0, -8.66, 8.66]}
        self.assertEqual(self.tool.validate_telemetry(valid), valid)
        for changed in ({**valid, "counts": [1, 2]},
                        {**valid, "rpm": [1, float("nan"), 3]},
                        {**valid, "status": None}):
            with self.subTest(changed=changed):
                with self.assertRaises(ValueError):
                    self.tool.validate_telemetry(changed)

    def test_arguments_reject_watchdog_unsafe_rate_and_nonfinite_motion(self):
        with self.assertRaises(SystemExit):
            self.tool.parse_args(["--ip", "192.168.0.50", "--rate", "3"])
        with self.assertRaises(SystemExit):
            self.tool.parse_args(["--ip", "192.168.0.50", "--vx", "nan"])
        args = self.tool.parse_args(["--ip", "192.168.0.50"])
        self.assertEqual((args.vx, args.vy, args.w, args.duration, args.rate),
                         (10.0, 0.0, 0.0, 3.0, 20.0))

    def test_main_runs_stop_then_run_then_repeated_safe_stop(self):
        instances = []

        class FakeTest:
            def __init__(self, ip, rate):
                self.calls = []
                self.last_telemetry = {"type": "telemetry"}
                instances.append(self)

            def phase(self, duration, vx, vy, w, status, **kwargs):
                self.calls.append(("phase", duration, vx, vy, w, status))

            def receive_available(self):
                self.calls.append(("receive",))

            def safe_stop(self):
                self.calls.append(("safe_stop",))

            def close(self):
                self.calls.append(("close",))

        with patch.object(self.tool, "VelocityTest", FakeTest):
            result = self.tool.main(["--ip", "192.168.0.50"])
        self.assertEqual(result, 0)
        self.assertEqual(instances[0].calls,
                         [("phase", 1.0, 0.0, 0.0, 0.0, "STOP"),
                          ("receive",),
                          ("phase", 3.0, 10.0, 0.0, 0.0, "RUN"),
                          ("safe_stop",), ("safe_stop",), ("close",)])

    def test_main_never_runs_without_telemetry(self):
        instances = []

        class NoTelemetryTest:
            def __init__(self, ip, rate):
                self.statuses = []
                self.last_telemetry = None
                instances.append(self)

            def phase(self, duration, vx, vy, w, status, **kwargs):
                self.statuses.append(status)

            def receive_available(self):
                return False

            def safe_stop(self):
                self.statuses.append("STOP")

            def close(self):
                pass

        with patch.object(self.tool, "VelocityTest", NoTelemetryTest):
            result = self.tool.main(["--ip", "192.168.0.50"])
        self.assertEqual(result, 2)
        self.assertNotIn("RUN", instances[0].statuses)
        self.assertEqual(instances[0].statuses, ["STOP", "STOP"])


if __name__ == "__main__":
    unittest.main()
