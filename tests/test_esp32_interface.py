import json
import socket
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from comm.telemetry_receiver import TelemetryReceiver
from comm.udp_sender import UdpSender


def telemetry_packet():
    return {"type": "telemetry", "seq": 12, "ms": 12345, "status": "STOP",
            "rssi": -52, "counts": [10, 20, 30], "rpm": [1.0, 2.0, 3.0],
            "wheel_speed": [4.0, 5.0, 6.0], "target_speed": [0.0, 0.0, 0.0]}


class Esp32InterfaceTests(unittest.TestCase):
    def test_pi_sender_wire_command_has_required_fields_and_units(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        sender = UdpSender("127.0.0.1", receiver.getsockname()[1])
        try:
            sender.send(12.3, -4.5, 0.35, "RUN", force=True)
            wire = json.loads(receiver.recvfrom(4096)[0])
            self.assertEqual(set(wire), {"seq", "t", "vx", "vy", "w", "status"})
            self.assertEqual((wire["vx"], wire["vy"], wire["w"], wire["status"]),
                             (12.3, -4.5, 0.35, "RUN"))
            self.assertGreaterEqual(wire["seq"], 0)
        finally:
            sender.close()
            receiver.close()

    def test_three_wheel_telemetry_and_malformed_packets(self):
        old_host, old_port = config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT
        config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = "127.0.0.1", 0
        receiver = TelemetryReceiver()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            addr = receiver._sock.getsockname()
            for bad in (b'{bad',
                        json.dumps({**telemetry_packet(), "counts": [1, 2]}).encode(),
                        json.dumps({**telemetry_packet(), "wheel_speed": [1, float("nan"), 3]}).encode(),
                        json.dumps({**telemetry_packet(), "rpm": [1, float("inf"), 3]}).encode()):
                sender.sendto(bad, addr)
            time.sleep(0.05)
            self.assertFalse(receiver.latest().received)
            sender.sendto(json.dumps(telemetry_packet()).encode(), addr)
            deadline = time.monotonic() + 1
            while not receiver.latest().received and time.monotonic() < deadline:
                time.sleep(0.01)
            state = receiver.latest()
            self.assertTrue(state.received)
            self.assertEqual(state.encoder_counts, [10, 20, 30])
            self.assertEqual(state.rpm, [1.0, 2.0, 3.0])
            self.assertEqual(state.wheel_speed_cm_s, [4.0, 5.0, 6.0])
            self.assertEqual(state.target_speed_cm_s, [0.0, 0.0, 0.0])
            self.assertEqual((state.seq, state.status, state.rssi), (12, "STOP", -52))
            time.sleep(0.06)
            sender.sendto(json.dumps({**telemetry_packet(), "counts": []}).encode(), addr)
            time.sleep(0.03)
            self.assertEqual(receiver.latest().seq, 12)
            self.assertGreater(receiver.latest().age_s, 0.06)
            time.sleep(config.TELEMETRY_STALE_S)
            self.assertGreater(receiver.latest().age_s, config.TELEMETRY_STALE_S)
        finally:
            sender.close()
            receiver.close()
            config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = old_host, old_port


if __name__ == "__main__":
    unittest.main()
