import json
import socket
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from comm.telemetry_receiver import TelemetryReceiver


class TelemetryTests(unittest.TestCase):
    def test_malformed_packet_does_not_kill_receiver_or_mark_online(self):
        old_host, old_port = config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT
        config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = "127.0.0.1", 0
        receiver = TelemetryReceiver()
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            addr = receiver._sock.getsockname()
            sender.sendto(b'{"counts":null}', addr)
            time.sleep(0.05)
            self.assertFalse(receiver.latest().received)
            valid = {"type": "telemetry", "seq": 7, "status": "STOP", "counts": [1, 2, 3],
                     "rpm": [0, 0, 0], "wheel_speed": [0, 0, 0],
                     "target_speed": [0, 0, 0], "rssi": -40}
            sender.sendto(json.dumps(valid).encode(), addr)
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline and not receiver.latest().received:
                time.sleep(0.01)
            self.assertEqual(receiver.latest().seq, 7)
        finally:
            sender.close()
            receiver.close()
            config.TELEMETRY_BIND_HOST, config.TELEMETRY_PORT = old_host, old_port


if __name__ == "__main__":
    unittest.main()
