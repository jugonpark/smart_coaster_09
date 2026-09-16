import json
import math
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from perception.radar import RadarReceiver, RadarReplay, SerialFrameParser, parse_fake_packet


def packet(value):
    return json.dumps(value).encode()


class RadarTests(unittest.TestCase):
    def test_legacy_single_target_and_velocity_angle_convention(self):
        state = parse_fake_packet(packet({"distance_cm": 62.4, "angle_deg": -8.3,
                                          "approach_speed_cm_s": 41.2}), received_at=10)
        self.assertTrue(state.connected and state.valid)
        self.assertEqual(state.status, "TARGET_DETECTED")
        self.assertEqual(state.target, state.nearest_target)
        self.assertEqual(state.target.angle_deg, -8.3)
        self.assertEqual(state.target.radial_velocity_cm_s, -41.2)
        self.assertTrue(state.target.approaching)
        self.assertEqual(state.target.timestamp, 10)
        leaving = parse_fake_packet(packet({"distance_cm": 50, "angle_deg": 180,
                                             "approach_speed_cm_s": -10}), received_at=11)
        self.assertEqual(leaving.target.angle_deg, -180)
        self.assertFalse(leaving.target.approaching)
        self.assertEqual(leaving.target.radial_velocity_cm_s, 10)

    def test_empty_and_multiple_targets(self):
        empty = parse_fake_packet(b'{"targets":[],"frame_id":4}', received_at=2)
        self.assertTrue(empty.connected and empty.valid)
        self.assertEqual(empty.status, "NO_TARGET")
        self.assertIsNone(empty.target)
        self.assertEqual(empty.frame_id, 4)
        entries = [{"distance_cm": d, "angle_deg": a, "approach_speed_cm_s": 0}
                   for d, a in ((80, 20), (30, -15), (40, 0))]
        state = parse_fake_packet(packet({"targets": entries}), received_at=3)
        self.assertEqual(len(state.targets), 3)
        self.assertEqual(state.nearest_target.distance_cm, 30)
        self.assertEqual([t.distance_cm for t in state.targets], [80, 30, 40])

    def test_bad_packets_are_rejected(self):
        base = {"distance_cm": 20, "angle_deg": 0, "approach_speed_cm_s": 1}
        bad = [b'{bad', b'[]', b'{"targets":[null]}',
               packet({**base, "distance_cm": -1}),
               packet({**base, "distance_cm": math.nan}),
               packet({**base, "angle_deg": math.inf}),
               packet({**base, "angle_deg": 181}),
               packet({**base, "approach_speed_cm_s": 501}),
               packet({**base, "distance_cm": 1001}),
               packet({**base, "frame_id": -1}),
               packet({**base, "distance_cm": "20"})]
        for raw in bad:
            with self.subTest(raw=raw), self.assertRaises((ValueError, TypeError)):
                parse_fake_packet(raw, received_at=1)

    def test_udp_error_recovery_stale_and_disconnect(self):
        with patch.object(config, "RADAR_BACKEND", "udp_fake"), patch.object(
                config, "RADAR_BIND_HOST", "127.0.0.1"), patch.object(
                config, "RADAR_UDP_PORT", 0), patch.object(config, "RADAR_STALE_S", 0.04):
            receiver = RadarReceiver()
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.assertEqual(receiver.read().status, "OFFLINE")
                address = receiver._sock.getsockname()
                sender.sendto(b'{bad', address)
                error = receiver.read()
                self.assertEqual(error.status, "PARSE_ERROR")
                self.assertFalse(error.valid)
                sender.sendto(b'{"targets":[]}', address)
                self.assertEqual(receiver.read().status, "NO_TARGET")
                sender.sendto(packet({"distance_cm": 12, "angle_deg": 0,
                                      "approach_speed_cm_s": 3}), address)
                self.assertEqual(receiver.read().target.distance_cm, 12)
                time.sleep(0.06)
                stale = receiver.read()
                self.assertEqual(stale.status, "DATA_STALE")
                self.assertFalse(stale.connected or stale.valid)
                self.assertIsNone(stale.target)
            finally:
                sender.close()
                receiver.close()

    def test_replay_continues_after_malformed_line(self):
        with tempfile.NamedTemporaryFile(dir=Path(__file__).parent, suffix=".jsonl",
                                         delete=False) as fixture:
            path = Path(fixture.name)
            fixture.write(b'{bad\n{"targets":[]}\n' + packet(
                {"distance_cm": 7, "angle_deg": 0, "approach_speed_cm_s": 1}) + b'\n')
        try:
            replay = RadarReplay(path)
            try:
                self.assertEqual(replay.read().status, "PARSE_ERROR")
                self.assertEqual(replay.read().status, "NO_TARGET")
                self.assertEqual(replay.read().target.distance_cm, 7)
                self.assertEqual(replay.read().status, "OFFLINE")
            finally:
                replay.close()
            with patch.object(config, "RADAR_BACKEND", "replay"), patch.object(
                    config, "RADAR_REPLAY_PATH", str(path)):
                receiver = RadarReceiver()
                try:
                    self.assertEqual(receiver.read().status, "PARSE_ERROR")
                    self.assertEqual(receiver.read().status, "NO_TARGET")
                finally:
                    receiver.close()
        finally:
            path.unlink()

    def test_serial_backend_marks_missing_data_and_parser_boundary(self):
        with patch.object(config, "RADAR_BACKEND", "serial"):
            receiver = RadarReceiver()
            state = receiver.read()
            self.assertFalse(state.connected or state.valid)
            self.assertIn("SAMPLE SERIAL FRAME REQUIRED", state.error)
            receiver.close()
        with self.assertRaises(NotImplementedError):
            SerialFrameParser().feed(b'not a verified frame')
        with patch.object(config, "RADAR_BACKEND", "replay"), patch.object(
                config, "RADAR_REPLAY_PATH", ""):
            receiver = RadarReceiver()
            self.assertEqual(receiver.read().status, "OFFLINE")
            self.assertFalse(receiver.read().valid)


if __name__ == "__main__":
    unittest.main()
