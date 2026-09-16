import math
import sys
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from core.control_loop import ControlLoop, run_monitor_loop
from core.shared_state import SharedState
from core.state_machine import ControlStateMachine
from main import perception_step
from fusion.sensor_fusion import SensorFusion
from perception.radar import RadarState, RadarTarget
from perception.robot_tracker import RobotPose


def valid_world():
    return SensorFusion().build(RobotPose(True, 0, 0, 0), [],
                                RadarState(True, RadarTarget(20, 0, 30), 0, "fake"))


class RecordingSender:
    def __init__(self):
        self.calls = []
        self.lock = threading.Lock()

    def send(self, vx, vy, w, status, force=False):
        with self.lock:
            self.calls.append((time.monotonic(), vx, vy, w, status))


class CoreControllerTests(unittest.TestCase):
    def setUp(self):
        self.shared = SharedState()
        self.sender = RecordingSender()
        self.telemetry = SimpleNamespace(latest=lambda: SimpleNamespace(received=True, age_s=0))

    def test_shared_world_is_snapshot_not_mutable_alias(self):
        world = valid_world()
        self.shared.publish_world(world, captured_at=10)
        world.robot.x_cm = 123
        read = self.shared.snapshot()
        self.assertEqual(read.world.robot.x_cm, 0)
        read.world.robot.x_cm = 456
        self.assertEqual(self.shared.snapshot().world.robot.x_cm, 0)

    def test_stale_world_sends_stop(self):
        self.shared.publish_world(valid_world(), captured_at=10)
        command = ControlLoop(self.shared, self.telemetry, self.sender).tick(now=11)
        self.assertEqual(command.status, "STOP")
        self.assertEqual(command.reason, "WorldState stale")
        self.assertEqual(self.sender.calls[-1][1:], (0, 0, 0, "STOP"))

    def test_perception_failure_revokes_previous_world(self):
        self.shared.publish_world(valid_world(), captured_at=10)
        self.shared.publish_failure("YOLO inference failed", camera_ok=True)
        command = ControlLoop(self.shared, self.telemetry, self.sender).tick(now=10.1)
        self.assertEqual(command.status, "STOP")
        self.assertIn("YOLO inference failed", command.reason)
        self.assertIsNone(self.shared.snapshot().world)

    def test_telemetry_stale_stops_motion(self):
        self.shared.publish_world(valid_world(), captured_at=10)
        receiver = SimpleNamespace(latest=lambda: SimpleNamespace(received=True, age_s=1))
        command = ControlLoop(self.shared, receiver, self.sender).tick(now=10.1)
        self.assertEqual(command.reason, "ESP32 telemetry stale")

    def test_invalid_world_stops_before_risk_evaluation(self):
        world = valid_world()
        world.robot.heading_rad = math.nan
        self.shared.publish_world(world, captured_at=10)
        command = ControlLoop(self.shared, self.telemetry, self.sender).tick(now=10.1)
        self.assertEqual(command.status, "STOP")
        self.assertIn("invalid", command.reason)

    def test_motion_default_off_even_for_danger(self):
        self.assertFalse(config.ENABLE_ESCAPE_MOTION)
        self.shared.publish_world(valid_world(), captured_at=10)
        command = ControlLoop(self.shared, self.telemetry, self.sender).tick(now=10.1)
        self.assertEqual(command.status, "STOP")
        self.assertIn("motion disabled", command.reason)

    def test_control_runs_near_30hz_while_perception_is_idle(self):
        self.shared.publish_world(valid_world(), captured_at=time.monotonic())
        loop = ControlLoop(self.shared, self.telemetry, self.sender)
        stop = threading.Event()
        thread = threading.Thread(target=loop.run, args=(stop,))
        thread.start()
        time.sleep(0.24)
        stop.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        calls = self.sender.calls
        self.assertGreaterEqual(len(calls), 5)
        self.assertLessEqual(len(calls), 10)
        self.assertLess(max(b[0] - a[0] for a, b in zip(calls, calls[1:])), 0.08)
        self.assertGreater(loop.timing.ticks, 0)
        self.assertGreater(loop.timing.last_interval_s, 0)

    def test_perception_exception_revokes_previous_world(self):
        self.shared.publish_world(valid_world(), captured_at=time.monotonic())
        camera = SimpleNamespace(read=lambda: (_ for _ in ()).throw(RuntimeError("read failed")))
        result = perception_step(camera, None, None, None, None, None, self.shared)
        self.assertFalse(result)
        self.assertIsNone(self.shared.snapshot().world)
        self.assertIn("read failed", self.shared.snapshot().failure)
        self.assertEqual(ControlLoop(self.shared, self.telemetry, self.sender).tick().status,
                         "STOP")

    def test_slow_inference_dates_world_at_capture_not_completion(self):
        camera = SimpleNamespace(read=lambda: (True, object()))
        tracker = SimpleNamespace(process=lambda frame, wf: RobotPose(True, 0, 0, 0))
        detector = SimpleNamespace(backend="ultralytics", detect=lambda frame, wf: time.sleep(0.08) or [],
                                   pick_obstacles=lambda detections: [])
        radar = SimpleNamespace(read=lambda: RadarState(True, RadarTarget(20, 0, 30)))
        self.assertTrue(perception_step(camera, None, tracker, detector, radar,
                                        SensorFusion(), self.shared))
        age = time.monotonic() - self.shared.snapshot().world_at
        self.assertGreaterEqual(age, 0.07)

    def test_failure_during_risk_calculation_cancels_old_motion(self):
        old_motion = config.ENABLE_ESCAPE_MOTION
        config.ENABLE_ESCAPE_MOTION = True
        self.shared.publish_world(valid_world(), captured_at=time.monotonic())
        entered = threading.Event()
        resume = threading.Event()
        loop = ControlLoop(self.shared, self.telemetry, self.sender)
        real_evaluate = loop.risk_eval.evaluate

        def delayed_evaluate(target):
            entered.set()
            self.assertTrue(resume.wait(1))
            return real_evaluate(target)

        loop.risk_eval.evaluate = delayed_evaluate
        result = []
        thread = threading.Thread(target=lambda: result.append(loop.tick()))
        try:
            thread.start()
            self.assertTrue(entered.wait(1))
            self.shared.publish_failure("camera offline", camera_ok=False)
        finally:
            resume.set()
            thread.join(1)
            config.ENABLE_ESCAPE_MOTION = old_motion
        self.assertEqual(result[0].status, "STOP")
        self.assertEqual(self.sender.calls[-1][1:], (0, 0, 0, "STOP"))

    def test_monitor_loop_runs_independently_of_perception(self):
        reports = []
        monitor = SimpleNamespace(send_unavailable=lambda reason, camera_ok: reports.append(
            (time.monotonic(), reason, camera_ok)))
        stopped = threading.Event()
        thread = threading.Thread(target=run_monitor_loop,
                                  args=(self.shared, self.telemetry, monitor, stopped))
        thread.start()
        time.sleep(0.23)
        stopped.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertGreaterEqual(len(reports), 2)
        self.assertLessEqual(len(reports), 3)
        self.assertTrue(all(not camera_ok for _, _, camera_ok in reports))

    def test_slow_risk_calculation_cannot_send_old_motion(self):
        old_motion = config.ENABLE_ESCAPE_MOTION
        config.ENABLE_ESCAPE_MOTION = True
        try:
            self.shared.publish_world(valid_world(), captured_at=time.monotonic())
            loop = ControlLoop(self.shared, self.telemetry, self.sender)
            real_evaluate = loop.risk_eval.evaluate

            def slow_evaluate(target):
                time.sleep(config.WORLD_STALE_S + 0.02)
                return real_evaluate(target)

            loop.risk_eval.evaluate = slow_evaluate
            command = loop.tick()
            self.assertEqual(command.status, "STOP")
            self.assertEqual(command.reason, "WorldState stale")
        finally:
            config.ENABLE_ESCAPE_MOTION = old_motion


if __name__ == "__main__":
    unittest.main()
