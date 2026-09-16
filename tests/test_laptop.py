import importlib.util
import json
import os
import socket
import sys
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "laptop"))
import camera_streamer
import monitor as laptop_monitor


def laptop_settings(**updates):
    values = dict(CAMERA_INDEX=0, FRAME_WIDTH=64, FRAME_HEIGHT=48, TARGET_FPS=30,
                  JPEG_QUALITY=80, FLIP_HORIZONTAL=False, MJPEG_HOST="127.0.0.1",
                  MJPEG_PORT=0, CAMERA_RETRY_S=0.12, CAMERA_FRAME_STALE_S=0.5,
                  MONITOR_BIND_HOST="127.0.0.1",
                  MONITOR_UDP_PORT=0, MONITOR_STALE_S=0.08)
    values.update(updates)
    return SimpleNamespace(**values)


class FakeCapture:
    def __init__(self, opened=True):
        self.opened = opened
        self.released = False

    def isOpened(self):
        return self.opened

    def set(self, *_):
        return True

    def read(self):
        return (True, np.full((48, 64, 3), 100, dtype=np.uint8)) if self.opened else (False, None)

    def release(self):
        self.released = True


class LaptopTests(unittest.TestCase):
    def test_camera_offline_then_reconnects_and_serves_mjpeg(self):
        broken, working = FakeCapture(False), FakeCapture(True)
        permit_reconnect = threading.Event()

        def make_capture(index):
            if not broken.released:
                return broken
            if not permit_reconnect.wait(2):
                raise TimeoutError("camera reconnect not released")
            return working

        settings = laptop_settings()
        with patch.object(camera_streamer, "config", settings), patch.object(
                camera_streamer.cv2, "VideoCapture", side_effect=make_capture):
            camera = camera_streamer.CameraStreamer()
            camera.start()
            base = f"http://127.0.0.1:{camera._server.server_address[1]}"
            try:
                self.assertEqual(camera.camera_state, "CAMERA_OFFLINE")
                self.assertFalse(json.load(urllib.request.urlopen(base + "/health", timeout=1))["ok"])
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(base + "/snapshot.jpg", timeout=1)
                self.assertEqual(error.exception.code, 503)
                with self.assertRaises(urllib.error.HTTPError) as error:
                    urllib.request.urlopen(base + "/stream.mjpg", timeout=1)
                self.assertEqual(error.exception.code, 503)
                permit_reconnect.set()
                deadline = time.monotonic() + 2
                while camera.camera_state != "CAMERA_ONLINE" and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(camera.camera_state, "CAMERA_ONLINE")
                with urllib.request.urlopen(base + "/stream.mjpg", timeout=2) as stream:
                    self.assertIn("multipart/x-mixed-replace", stream.headers["Content-Type"])
                    content = stream.read(128)
                    self.assertIn(b"--frame\r\nContent-Type: image/jpeg\r\n", content)
            finally:
                permit_reconnect.set()
                camera.close()
            self.assertTrue(working.released)
            self.assertEqual(camera.camera_state, "CAMERA_OFFLINE")

    def test_camera_backend_open_exception_is_retried(self):
        settings = laptop_settings(CAMERA_RETRY_S=0.02)
        working = FakeCapture()
        with patch.object(camera_streamer, "config", settings), patch.object(
                camera_streamer.cv2, "VideoCapture", side_effect=[RuntimeError("USB missing"), working]):
            camera = camera_streamer.CameraStreamer()
            try:
                camera.start()
                deadline = time.monotonic() + 1
                while camera.camera_state != "CAMERA_ONLINE" and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(camera.camera_state, "CAMERA_ONLINE")
            finally:
                camera.close()

    def test_running_camera_loss_clears_frame_then_recovers(self):
        fail = threading.Event()
        reconnect = threading.Event()

        class FlakyCapture(FakeCapture):
            def read(self):
                return (False, None) if fail.is_set() else super().read()

        flaky, recovered = FlakyCapture(), FakeCapture()

        def make_capture(index):
            if not flaky.released:
                return flaky
            reconnect.wait(2)
            return recovered

        settings = laptop_settings(CAMERA_RETRY_S=0.03)
        with patch.object(camera_streamer, "config", settings), patch.object(
                camera_streamer.cv2, "VideoCapture", side_effect=make_capture):
            camera = camera_streamer.CameraStreamer()
            camera.start()
            try:
                deadline = time.monotonic() + 1
                while camera.camera_state != "CAMERA_ONLINE" and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(camera.camera_state, "CAMERA_ONLINE")
                fail.set()
                deadline = time.monotonic() + 1
                while camera.camera_state != "CAMERA_OFFLINE" and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertIsNone(camera.latest_frame())
                reconnect.set()
                deadline = time.monotonic() + 1
                while camera.camera_state != "CAMERA_ONLINE" and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertEqual(camera.camera_state, "CAMERA_ONLINE")
            finally:
                reconnect.set()
                camera.close()

    def test_monitor_rejects_bad_packets_then_updates_and_expires(self):
        settings = laptop_settings()
        with patch.object(laptop_monitor, "config", settings):
            receiver = laptop_monitor.MonitorReceiver()
            receiver.start()
            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                addr = receiver._sock.getsockname()
                for raw in (b'{bad', b'[]', b'{"vx":NaN}', b'{"vx":1e999}',
                            b'[' * 1000):
                    sender.sendto(raw, addr)
                time.sleep(0.03)
                self.assertFalse(receiver.snapshot()[1])
                sender.sendto(json.dumps({"t": 0, "state": "WATCH/STOP", "camera": "ONLINE"}).encode(), addr)
                deadline = time.monotonic() + 1
                while not receiver.snapshot()[1] and time.monotonic() < deadline:
                    time.sleep(0.01)
                status, online = receiver.snapshot()
                self.assertTrue(online)
                self.assertEqual(status["t"], 0)  # sender time is not local receive time
                self.assertEqual(status["state"], "WATCH/STOP")
                time.sleep(settings.MONITOR_STALE_S + 0.03)
                status, online = receiver.snapshot()
                self.assertFalse(online)
                self.assertEqual(status, {})  # old data must not look current
            finally:
                sender.close()
                receiver.close()

    def test_missing_fields_and_nonfinite_values_render_as_unknown(self):
        lines = laptop_monitor.format_status_lines({}, online=True, camera_state="CAMERA_OFFLINE")
        self.assertIn("CAMERA: CAMERA_OFFLINE", lines)
        self.assertIn("ESP32: UNKNOWN", lines)
        self.assertIn("THREAT: UNKNOWN", lines)
        self.assertIn("VX: N/A", lines)
        lines = laptop_monitor.format_status_lines(
            {"command": "RUN vx=nan vy=4.0 w=0.2", "escape": "BACK 30cm"},
            online=True, camera_state="CAMERA_ONLINE")
        self.assertIn("VX: N/A", lines)
        self.assertIn("VY: 4.0", lines)
        self.assertIn("ESCAPE: BACK", lines)
        self.assertIn("TARGET: 30cm", lines)
        untrusted = laptop_monitor.format_status_lines(
            {"vx": "NaN", "target_distance_cm": "Infinity"},
            online=True, camera_state="CAMERA_ONLINE")
        self.assertIn("VX: N/A", untrusted)
        self.assertIn("TARGET: N/A", untrusted)
        stale = laptop_monitor.format_status_lines({"risk": "DANGER"},
                                                    online=False, camera_state="CAMERA_ONLINE")
        self.assertIn("PI: OFFLINE", stale)
        self.assertIn("RISK: UNKNOWN", stale)

    def test_camera_and_network_settings_accept_environment_values(self):
        path = ROOT / "laptop" / "config.py"
        spec = importlib.util.spec_from_file_location("laptop_config_probe", path)
        module = importlib.util.module_from_spec(spec)
        overrides = {"GRISE_CAMERA_INDEX": "2", "GRISE_FRAME_WIDTH": "640",
                     "GRISE_FRAME_HEIGHT": "480", "GRISE_CAMERA_FPS": "15",
                     "GRISE_JPEG_QUALITY": "70", "GRISE_MJPEG_HOST": "127.0.0.1",
                     "GRISE_MJPEG_PORT": "8181", "GRISE_MONITOR_BIND_HOST": "127.0.0.1",
                     "GRISE_MONITOR_UDP_PORT": "9101"}
        with patch.dict(os.environ, overrides):
            spec.loader.exec_module(module)
        self.assertEqual((module.CAMERA_INDEX, module.FRAME_WIDTH, module.FRAME_HEIGHT,
                          module.TARGET_FPS, module.JPEG_QUALITY), (2, 640, 480, 15, 70))
        self.assertEqual((module.MJPEG_HOST, module.MJPEG_PORT,
                          module.MONITOR_BIND_HOST, module.MONITOR_UDP_PORT),
                         ("127.0.0.1", 8181, "127.0.0.1", 9101))

    def test_window_close_releases_camera_and_monitor(self):
        path = ROOT / "laptop" / "main.py"
        spec = importlib.util.spec_from_file_location("laptop_main_probe", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        camera = SimpleNamespace(start=lambda: None, latest_frame=lambda: None,
                                 camera_state="CAMERA_OFFLINE", close=lambda: closed.append("camera"))
        monitor = SimpleNamespace(start=lambda: None, snapshot=lambda: ({}, False),
                                  close=lambda: closed.append("monitor"))
        closed = []
        settings = laptop_settings(SHOW_PREVIEW=True, WINDOW_NAME="GRISE Test")
        with patch.object(module, "config", settings), patch.object(
                module, "CameraStreamer", return_value=camera), patch.object(
                module, "MonitorReceiver", return_value=monitor), patch.object(
                module, "draw_status", side_effect=lambda frame, *_: frame), patch.object(
                module.cv2, "imshow"), patch.object(module.cv2, "waitKey", return_value=0), patch.object(
                module.cv2, "getWindowProperty", return_value=-1), patch.object(
                module.cv2, "destroyAllWindows"):
            module.main()
        self.assertEqual(closed, ["monitor", "camera"])


if __name__ == "__main__":
    unittest.main()
