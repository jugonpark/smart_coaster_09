"""Self-contained ArUco robot pose + pixel/world conversion.
Adapted from the provided grise-master camera.py, marker_scanner.py and robot_tracker.py.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np
import config


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class WorldFrame:
    px_per_cm: float = config.DEFAULT_PX_PER_CM
    frame_height: int = config.FRAME_HEIGHT

    def update_scale(self, px_per_cm: float) -> None:
        if px_per_cm is None or not np.isfinite(px_per_cm) or px_per_cm <= 0:
            return
        if 0.5 * self.px_per_cm <= px_per_cm <= 2.0 * self.px_per_cm:
            self.px_per_cm = 0.8 * self.px_per_cm + 0.2 * px_per_cm
        else:
            self.px_per_cm = px_per_cm

    def to_world(self, px_x: float, px_y: float) -> tuple[float, float]:
        return px_x / self.px_per_cm, (self.frame_height - px_y) / self.px_per_cm

    def to_pixel(self, world_x: float, world_y: float) -> tuple[int, int]:
        return int(round(world_x * self.px_per_cm)), int(round(self.frame_height - world_y * self.px_per_cm))

    def px_len_to_cm(self, px: float) -> float:
        return px / self.px_per_cm


@dataclass
class MarkerScan:
    by_id: dict = field(default_factory=dict)

    def get(self, marker_id: int):
        return self.by_id.get(marker_id)

    @staticmethod
    def center_px(corners) -> tuple[float, float]:
        return float(np.mean(corners[:, 0])), float(np.mean(corners[:, 1]))

    @staticmethod
    def side_px(corners) -> float:
        return float(np.mean([np.linalg.norm(corners[(k + 1) % 4] - corners[k]) for k in range(4)]))

    @staticmethod
    def heading_rad(corners, offset_rad: float = 0.0) -> float:
        tl, tr = corners[0], corners[1]
        return _wrap_pi(math.atan2(-(float(tr[1]) - float(tl[1])), float(tr[0]) - float(tl[0])) + offset_rad)


class MarkerScanner:
    def __init__(self) -> None:
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, config.ARUCO_DICT_NAME))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    def scan(self, frame) -> MarkerScan:
        corners, ids, _ = self._detector.detectMarkers(frame)
        out = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                mid = int(i)
                cand = c[0]
                if mid not in out or MarkerScan.side_px(cand) > MarkerScan.side_px(out[mid]):
                    out[mid] = cand
        return MarkerScan(out)


@dataclass
class RobotPose:
    detected: bool = False
    x_cm: float = 0.0
    y_cm: float = 0.0
    heading_rad: float = 0.0
    px: tuple[float, float] = (0.0, 0.0)
    timestamp: float | None = None


class RobotTracker:
    def __init__(self) -> None:
        self._scanner = MarkerScanner()
        self._last = RobotPose()
        self._heading_offset = math.radians(config.MARKER_HEADING_OFFSET_DEG)

    def process(self, frame, world: WorldFrame) -> RobotPose:
        scan = self._scanner.scan(frame)
        target = scan.get(config.ROBOT_MARKER_ID)
        if target is None:
            self._last = RobotPose()
            return self._last

        world.update_scale(MarkerScan.side_px(target) / config.MARKER_SIZE_CM)
        cx_px, cy_px = MarkerScan.center_px(target)
        x_cm, y_cm = world.to_world(cx_px, cy_px)
        heading = MarkerScan.heading_rad(target, self._heading_offset)

        a = config.ROBOT_POSE_EMA_ALPHA
        if self._last.detected:
            x_cm = (1 - a) * self._last.x_cm + a * x_cm
            y_cm = (1 - a) * self._last.y_cm + a * y_cm
            d = _wrap_pi(heading - self._last.heading_rad)
            heading = _wrap_pi(self._last.heading_rad + a * d)

        self._last = RobotPose(True, x_cm, y_cm, heading, (cx_px, cy_px), time.monotonic())
        return self._last
