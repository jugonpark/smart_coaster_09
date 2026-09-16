from __future__ import annotations

import time
import cv2
import config


class NetworkCamera:
    """OpenCV reader for the laptop MJPEG stream with reconnect behavior."""

    def __init__(self, url: str | None = None) -> None:
        self.url = url or config.NETWORK_CAMERA_URL
        self.cap = None
        self.last_ok_t = 0.0
        self.state = "OFFLINE"
        self._open()

    def _open(self) -> None:
        if self.cap is not None:
            self.cap.release()
        self.cap = None
        self.state = "OFFLINE"
        try:
            self.cap = cv2.VideoCapture(self.url)
            if self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception as exc:
            print(f"[CAMERA] open failed: {exc}")
        print(f"[CAMERA] network source: {self.url}")

    def read(self):
        if self.cap is None or not self.cap.isOpened():
            self._open()
            time.sleep(0.05)
            return False, None
        try:
            ok, frame = self.cap.read()
        except Exception as exc:
            print(f"[CAMERA] read failed: {exc}")
            ok, frame = False, None
        if not ok or frame is None:
            self._open()
            return False, None
        if config.FLIP_HORIZONTAL:
            frame = cv2.flip(frame, 1)
        self.last_ok_t = time.monotonic()
        self.state = "ONLINE"
        return True, frame

    def close(self) -> None:
        self.state = "OFFLINE"
        if self.cap is not None:
            self.cap.release()
            self.cap = None
