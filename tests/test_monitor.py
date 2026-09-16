import json
import socket
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from comm.monitor_sender import MonitorSender


class MonitorTests(unittest.TestCase):
    def test_camera_failure_is_reported_to_laptop(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        old_ip, old_port = config.MONITOR_IP, config.MONITOR_PORT
        config.MONITOR_IP, config.MONITOR_PORT = "127.0.0.1", receiver.getsockname()[1]
        sender = None
        try:
            sender = MonitorSender()
            sender.send_camera_offline()
            payload = json.loads(receiver.recvfrom(4096)[0])
            self.assertEqual(payload["camera"], "OFFLINE")
            self.assertEqual(payload["command"], "STOP vx=0.0 vy=0.0 w=0.00")
            self.assertEqual(payload["reason"], "camera offline")
        finally:
            if sender:
                sender.close()
            receiver.close()
            config.MONITOR_IP, config.MONITOR_PORT = old_ip, old_port

    def test_fresh_zero_age_telemetry_is_online(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        old_ip, old_port = config.MONITOR_IP, config.MONITOR_PORT
        config.MONITOR_IP, config.MONITOR_PORT = "127.0.0.1", receiver.getsockname()[1]
        sender = None
        try:
            sender = MonitorSender()
            blocked = SimpleNamespace(front=False, front_left=False, left=False,
                                      back_left=False, rear=False, back_right=False,
                                      right=False, front_right=False)
            world = SimpleNamespace(radar_target=None, radar_connected=False,
                                    robot=SimpleNamespace(detected=True), blocked=blocked)
            sender.send(world=world, risk=SimpleNamespace(level="SAFE"),
                        plan=SimpleNamespace(direction="NONE", target_distance_cm=0),
                        command=SimpleNamespace(status="STOP", vx=0, vy=0, w=0,
                                                reason="safe"),
                        telemetry=SimpleNamespace(received=True, age_s=0), camera_ok=True)
            payload = json.loads(receiver.recvfrom(4096)[0])
            self.assertEqual(payload["esp32"], "ONLINE")
        finally:
            if sender:
                sender.close()
            receiver.close()
            config.MONITOR_IP, config.MONITOR_PORT = old_ip, old_port

    def test_perception_failure_monitor_message_keeps_camera_status(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        old_ip, old_port = config.MONITOR_IP, config.MONITOR_PORT
        config.MONITOR_IP, config.MONITOR_PORT = "127.0.0.1", receiver.getsockname()[1]
        sender = None
        try:
            sender = MonitorSender()
            sender.send_unavailable("YOLO failed", camera_ok=True)
            payload = json.loads(receiver.recvfrom(4096)[0])
            self.assertEqual(payload["camera"], "ONLINE")
            self.assertEqual(payload["reason"], "YOLO failed")
            self.assertTrue(payload["command"].startswith("STOP"))
        finally:
            if sender:
                sender.close()
            receiver.close()
            config.MONITOR_IP, config.MONITOR_PORT = old_ip, old_port


if __name__ == "__main__":
    unittest.main()
