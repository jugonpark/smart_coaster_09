import math
import sys
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from core.shared_state import SharedState
from fusion.sensor_fusion import SensorFusion
from main import perception_step
from perception.network_camera import NetworkCamera
from perception.object_detector import Detection, ObjectDetector
from perception.radar import RadarState
from perception.robot_tracker import MarkerScan, RobotPose, RobotTracker, WorldFrame
from perception.vision_state import DIRECTIONS, build_vision_state, sector_for


def obj(x, y, *, timestamp=None):
    return Detection("box", 0.9, 10, 10, 4, 4, x, y, 0, timestamp)


def vision(robot, objects, *, available=True, now=100):
    return build_vision_state(camera_ok=True, captured_at=now, robot=robot,
                              detections=objects, detector_available=available, now=now)


class VisionTests(unittest.TestCase):
    def test_marker_lost_discards_pose_and_smoothing_history(self):
        tracker = RobotTracker()
        corners = np.array([[0, 0], [80, 0], [80, 80], [0, 80]], dtype=float)
        tracker._scanner.scan = lambda frame: MarkerScan({0: corners})
        first = tracker.process(None, WorldFrame())
        self.assertTrue(first.detected)
        self.assertIsNotNone(first.timestamp)
        tracker._scanner.scan = lambda frame: MarkerScan()
        lost = tracker.process(None, WorldFrame())
        self.assertFalse(lost.detected)
        self.assertIsNone(lost.timestamp)
        self.assertEqual((lost.x_cm, lost.y_cm), (0, 0))

    def test_world_to_robot_position_and_heading(self):
        state = vision(RobotPose(True, 10, 20, math.pi / 2), [obj(10, 40)])
        relative = state.objects[0]
        self.assertAlmostEqual(relative.x_cm, 20)
        self.assertAlmostEqual(relative.y_cm, 0, places=6)
        self.assertEqual(state.sectors["FRONT"], "BLOCKED")

    def test_each_sector_and_exact_angle_boundaries(self):
        robot = RobotPose(True, 0, 0, 0)
        for index, direction in enumerate(DIRECTIONS):
            angle = math.radians(index * 45)
            state = vision(robot, [obj(20 * math.cos(angle), 20 * math.sin(angle))])
            self.assertEqual(state.sectors[direction], "BLOCKED", direction)
            self.assertEqual(sum(v == "BLOCKED" for v in state.sectors.values()), 1)
        for angle, expected in ((22.5, "FRONT_LEFT"), (67.5, "LEFT"),
                                (-22.5, "FRONT"), (-67.5, "FRONT_RIGHT"),
                                (157.5, "BACK"), (-157.5, "BACK_RIGHT")):
            self.assertEqual(sector_for(math.radians(angle)), expected)

    def test_threshold_robot_radius_and_valid_empty(self):
        robot = RobotPose(True, 0, 0, 0)
        edge = config.COLLISION_CHECK_DISTANCE_CM + config.ROBOT_RADIUS_CM
        self.assertEqual(vision(robot, [obj(edge, 0)]).sectors["FRONT"], "BLOCKED")
        self.assertEqual(vision(robot, [obj(edge + 0.01, 0)]).sectors["FRONT"], "CLEAR")
        self.assertTrue(all(s == "CLEAR" for s in vision(robot, []).sectors.values()))

    def test_unavailable_lost_stale_and_nonfinite_are_unknown(self):
        robot = RobotPose(True, 0, 0, 0)
        cases = [vision(robot, [], available=False), vision(RobotPose(), []),
                 vision(RobotPose(True, 0, 0, 0, timestamp=90), []),
                 vision(RobotPose(True, math.nan, 0, 0), []),
                 vision(robot, [obj(math.inf, 0)]),
                 vision(robot, [object()]),
                 vision(robot, [obj(10, 0, timestamp=90)])]
        for state in cases:
            self.assertTrue(all(s == "UNKNOWN" for s in state.sectors.values()))
        self.assertFalse(cases[0].detector_available)
        self.assertFalse(cases[4].detector_available)

    def test_detector_failure_differs_from_empty_inference(self):
        with patch.object(config, "YOLO_BACKEND", "stub"):
            detector = ObjectDetector()
        self.assertFalse(detector.available)
        self.assertEqual(detector.detect(None, WorldFrame()), [])
        detector.backend = "ultralytics"
        detector._frame_counter = 1
        detector._detect_ultralytics = lambda frame: []
        self.assertEqual(detector.detect(None, WorldFrame()), [])
        self.assertTrue(detector.available)
        detector._detect_ultralytics = lambda frame: (_ for _ in ()).throw(RuntimeError("model"))
        detector._frame_counter = 1
        with self.assertRaises(RuntimeError):
            detector.detect(None, WorldFrame())
        self.assertFalse(detector.available)
        self.assertEqual(detector._cached, [])

    def test_camera_disconnect_and_reconnect_state(self):
        class Capture:
            def __init__(self, good): self.good = good
            def isOpened(self): return True
            def set(self, *_): pass
            def read(self):
                return (True, np.zeros((4, 4, 3), dtype=np.uint8)) if self.good else (False, None)
            def release(self): pass
        captures = iter([Capture(False), Capture(True)])
        with patch("perception.network_camera.cv2.VideoCapture", side_effect=lambda url: next(captures)):
            camera = NetworkCamera("fake")
            self.assertEqual(camera.state, "OFFLINE")
            self.assertEqual(camera.read(), (False, None))
            self.assertEqual(camera.state, "OFFLINE")
            self.assertTrue(camera.read()[0])
            self.assertEqual(camera.state, "ONLINE")
            self.assertLess(time.monotonic() - camera.last_ok_t, 1)
            camera.close()
            self.assertEqual(camera.state, "OFFLINE")

    def test_perception_publishes_unknown_when_detector_fails(self):
        shared = SharedState()
        camera = SimpleNamespace(read=lambda: (True, object()), last_ok_t=time.monotonic())
        tracker = SimpleNamespace(process=lambda frame, world: RobotPose(True, 0, 0, 0))
        detector = SimpleNamespace(backend="ultralytics", available=True,
                                   detect=lambda frame, world: (_ for _ in ()).throw(RuntimeError("inference")))
        radar = SimpleNamespace(read=lambda: RadarState())
        self.assertTrue(perception_step(camera, WorldFrame(), tracker, detector,
                                        radar, SensorFusion(), shared))
        world = shared.snapshot().world
        self.assertIsNotNone(world.vision)
        self.assertTrue(world.vision.robot.detected)
        self.assertFalse(world.vision.detector_available)
        self.assertFalse(world.sensor_valid)
        self.assertTrue(all(v == "UNKNOWN" for v in world.vision.sectors.values()))
        self.assertFalse(world.direction_clear("FRONT"))

    def test_perception_rejects_stale_frame(self):
        shared = SharedState()
        camera = SimpleNamespace(read=lambda: (True, object()),
                                 last_ok_t=time.monotonic() - config.WORLD_STALE_S - 0.1)
        self.assertFalse(perception_step(camera, None, None, None, None, None, shared))
        self.assertIsNone(shared.snapshot().world)
        self.assertEqual(shared.snapshot().failure, "camera frame stale")


if __name__ == "__main__":
    unittest.main()
