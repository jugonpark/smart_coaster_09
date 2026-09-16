import json
import socket
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "raspberry_pi"))
sys.path.insert(0, str(ROOT / "laptop"))

import config
from comm.udp_sender import UdpSender
from perception.network_camera import NetworkCamera
from camera_streamer import CameraStreamer


class FakeCapture:
    def isOpened(self):
        return True

    def set(self, *_):
        return True

    def read(self):
        return True, np.full((48, 64, 3), 120, dtype=np.uint8)

    def release(self):
        pass


class TransportTests(unittest.TestCase):
    def test_laptop_mjpeg_is_decoded_by_pi_network_camera(self):
        import camera_streamer
        laptop_config = SimpleNamespace(
            CAMERA_INDEX=0, FRAME_WIDTH=64, FRAME_HEIGHT=48, TARGET_FPS=30,
            FLIP_HORIZONTAL=False, MJPEG_HOST="127.0.0.1", MJPEG_PORT=0,
            JPEG_QUALITY=80, CAMERA_RETRY_S=0.1, CAMERA_FRAME_STALE_S=0.5,
        )
        with patch.object(camera_streamer, "config", laptop_config):
            with patch.object(camera_streamer.cv2, "VideoCapture", return_value=FakeCapture()):
                streamer = CameraStreamer()
            camera = None
            try:
                streamer.start()
                port = streamer._server.server_address[1]
                camera = NetworkCamera(f"http://127.0.0.1:{port}/stream.mjpg")
                deadline = time.monotonic() + 3
                ok, frame = False, None
                while time.monotonic() < deadline and not ok:
                    ok, frame = camera.read()
                self.assertTrue(ok)
                self.assertEqual(frame.shape, (48, 64, 3))
            finally:
                if camera:
                    camera.close()
                streamer.close()

    def test_pi_command_packet_preserves_stop_run_slow_wire_format(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        sender = UdpSender("127.0.0.1", receiver.getsockname()[1])
        try:
            for status, vx in (("STOP", 0), ("RUN", 12.3), ("SLOW", 4.5)):
                sender.send(vx, 0, 0, status, force=True)
                payload = json.loads(receiver.recvfrom(4096)[0])
                self.assertEqual(set(payload), {"seq", "t", "vx", "vy", "w", "status"})
                self.assertEqual((payload["status"], payload["vx"]), (status, vx))
        finally:
            sender.close()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
