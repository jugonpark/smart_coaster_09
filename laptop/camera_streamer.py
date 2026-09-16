from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

import config


class CameraStreamer:
    """One camera capture thread shared by local preview and MJPEG clients."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frame = None
        self._frame_at = 0.0
        self._running = False
        self._capture_thread: threading.Thread | None = None
        self._server_thread: threading.Thread | None = None
        self._server: ThreadingHTTPServer | None = None
        self.cap = None
        self._open_camera()

    def _open_camera(self) -> None:
        cap = None
        try:
            cap = cv2.VideoCapture(config.CAMERA_INDEX)
            if not cap.isOpened():
                cap.release()
                self.cap = None
                print(f"[CAMERA] offline: index={config.CAMERA_INDEX}; retrying")
                return
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.FRAME_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, config.TARGET_FPS)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.cap = cap
            print(f"[CAMERA] opened: index={config.CAMERA_INDEX}")
        except Exception as exc:
            if cap is not None:
                cap.release()
            self.cap = None
            print(f"[CAMERA] open failed: {exc}")

    @property
    def camera_state(self) -> str:
        with self._lock:
            fresh = self._frame is not None and (
                time.monotonic() - self._frame_at <= config.CAMERA_FRAME_STALE_S
            )
        return "CAMERA_ONLINE" if self._running and fresh else "CAMERA_OFFLINE"

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        streamer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                return

            def do_GET(self):
                if self.path == "/health":
                    body = (b'{"ok":true}' if streamer.camera_state == "CAMERA_ONLINE" else b'{"ok":false}')
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return

                if self.path == "/snapshot.jpg":
                    jpg = streamer._encode_latest()
                    if jpg is None:
                        self.send_error(503, "no frame")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                    return

                if self.path == "/stream.mjpg":
                    if streamer.camera_state != "CAMERA_ONLINE":
                        self.send_error(503, "camera offline")
                        return
                    self.send_response(200)
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Pragma", "no-cache")
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    try:
                        while streamer._running:
                            jpg = streamer._encode_latest()
                            if jpg is None:
                                break
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                            time.sleep(1.0 / max(config.TARGET_FPS, 1))
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    self.close_connection = True
                    return

                self.send_error(404)

        self._server = ThreadingHTTPServer((config.MJPEG_HOST, config.MJPEG_PORT), Handler)
        self._server.daemon_threads = True
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

    def _capture_loop(self) -> None:
        period = 1.0 / max(config.TARGET_FPS, 1)
        while self._running:
            if self.cap is None:
                time.sleep(config.CAMERA_RETRY_S)
                if self._running:
                    self._open_camera()
                continue
            t0 = time.perf_counter()
            try:
                ok, frame = self.cap.read()
            except Exception as exc:
                print(f"[CAMERA] read failed: {exc}")
                ok, frame = False, None
            if ok and frame is not None:
                if config.FLIP_HORIZONTAL:
                    frame = cv2.flip(frame, 1)
                with self._lock:
                    self._frame = frame
                    self._frame_at = time.monotonic()
            else:
                with self._lock:
                    self._frame = None
                    self._frame_at = 0.0
                self.cap.release()
                self.cap = None
                print("[CAMERA] frame unavailable; reconnecting")
                continue
            dt = time.perf_counter() - t0
            if dt < period:
                time.sleep(period - dt)

    def latest_frame(self):
        with self._lock:
            if (not self._running or self._frame is None or
                    time.monotonic() - self._frame_at > config.CAMERA_FRAME_STALE_S):
                return None
            return self._frame.copy()

    def _encode_latest(self) -> bytes | None:
        frame = self.latest_frame()
        if frame is None:
            return None
        ok, encoded = cv2.imencode(
            ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, config.JPEG_QUALITY]
        )
        return encoded.tobytes() if ok else None

    def close(self) -> None:
        self._running = False
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=1)
        if self._server_thread is not None:
            self._server_thread.join(timeout=1)
        if self.cap is not None:
            self.cap.release()
            self.cap = None
