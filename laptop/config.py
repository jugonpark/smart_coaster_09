"""GRISE laptop node configuration.

Role: external USB camera sender + monitoring display only.
The laptop does NOT make motion decisions.
"""
import os


def _int(name: str, default: int, minimum: int, maximum: int) -> int:
    value = int(os.getenv(name, str(default)))
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be between {minimum} and {maximum}")
    return value


def _float(name: str, default: float) -> float:
    value = float(os.getenv(name, str(default)))
    if not 0 < value < float("inf"):
        raise ValueError(f"{name} must be a finite positive number")
    return value

CAMERA_INDEX = _int("GRISE_CAMERA_INDEX", 0, 0, 32)
FRAME_WIDTH = _int("GRISE_FRAME_WIDTH", 1280, 1, 8192)
FRAME_HEIGHT = _int("GRISE_FRAME_HEIGHT", 720, 1, 8192)
TARGET_FPS = _int("GRISE_CAMERA_FPS", 30, 1, 240)
FLIP_HORIZONTAL = False
CAMERA_RETRY_S = _float("GRISE_CAMERA_RETRY_S", 1.0)
CAMERA_FRAME_STALE_S = _float("GRISE_CAMERA_FRAME_STALE_S", 0.5)

# MJPEG server used by Raspberry Pi NetworkCamera.
MJPEG_HOST = os.getenv("GRISE_MJPEG_HOST", "0.0.0.0")
MJPEG_PORT = _int("GRISE_MJPEG_PORT", 8080, 0, 65535)
JPEG_QUALITY = _int("GRISE_JPEG_QUALITY", 80, 1, 100)

# Raspberry Pi -> laptop monitoring JSON UDP.
MONITOR_BIND_HOST = os.getenv("GRISE_MONITOR_BIND_HOST", "0.0.0.0")
MONITOR_UDP_PORT = _int("GRISE_MONITOR_UDP_PORT", 9001, 0, 65535)
MONITOR_STALE_S = _float("GRISE_MONITOR_STALE_S", 1.5)

SHOW_PREVIEW = True
WINDOW_NAME = "GRISE Laptop Camera / Monitor"
