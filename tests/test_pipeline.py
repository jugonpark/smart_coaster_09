import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

from decision.risk_evaluator import RiskState
from fusion.sensor_fusion import SensorFusion, WorldState
from perception.object_detector import Detection
from perception.radar import RadarState, RadarTarget
from perception.robot_tracker import RobotPose
from planning.escape_planner import EscapePlan, EscapePlanner
from safety.safety_manager import SafetyManager


def obstacle(x, y):
    return Detection("box", 0.9, 0, 0, 10, 10, x_cm=x, y_cm=y, radius_cm=2)


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.robot = RobotPose(True, 0, 0, 0)
        self.radar = RadarState(True, RadarTarget(20, 0, 30), 0, "udp")

    def test_diagonal_obstacle_blocks_only_its_sector(self):
        world = SensorFusion().build(self.robot, [obstacle(20, 20)], self.radar)
        self.assertFalse(world.direction_clear("FRONT_LEFT"))
        self.assertTrue(world.direction_clear("FRONT"))
        self.assertTrue(world.direction_clear("LEFT"))

    def test_blocked_preferred_escape_selects_nearest_clear_sector(self):
        world = SensorFusion().build(self.robot, [obstacle(-20, 0)], self.radar)
        plan = EscapePlanner().plan(world, RiskState(level="DANGER"))
        self.assertEqual(plan.status, "RUN")
        self.assertIn(plan.direction, ("BACK_LEFT", "BACK_RIGHT"))

    def test_nonfinite_obstacle_fails_closed(self):
        world = SensorFusion().build(self.robot, [obstacle(math.nan, 0)], self.radar)
        plan = EscapePlanner().plan(world, RiskState(level="DANGER"))
        command = SafetyManager().validate(
            camera_ok=True, world=world, plan=plan,
            telemetry=SimpleNamespace(received=True, age_s=0),
        )
        self.assertEqual(command.status, "STOP")
        self.assertIn("sensor", command.reason.lower())

    def test_invalid_motion_is_stopped_even_if_motion_enabled(self):
        import config
        old = config.ENABLE_ESCAPE_MOTION
        config.ENABLE_ESCAPE_MOTION = True
        try:
            world = SensorFusion().build(self.robot, [], self.radar)
            plan = EscapePlan(vx=math.nan, status="RUN", direction="BACK")
            command = SafetyManager().validate(
                camera_ok=True, world=world, plan=plan,
                telemetry=SimpleNamespace(received=True, age_s=0),
            )
            self.assertEqual(command.status, "STOP")
            self.assertIn("invalid motion", command.reason)
        finally:
            config.ENABLE_ESCAPE_MOTION = old

    def test_unavailable_obstacle_detector_prevents_motion(self):
        import config
        old = config.ENABLE_ESCAPE_MOTION
        config.ENABLE_ESCAPE_MOTION = True
        try:
            world = SensorFusion().build(self.robot, [], self.radar, perception_available=False)
            plan = EscapePlanner().plan(world, RiskState(level="DANGER"))
            command = SafetyManager().validate(
                camera_ok=True, world=world, plan=plan,
                telemetry=SimpleNamespace(received=True, age_s=0),
            )
            self.assertEqual(command.status, "STOP")
            self.assertIn("sensor", command.reason.lower())
        finally:
            config.ENABLE_ESCAPE_MOTION = old

    def test_safety_and_planning_matrix(self):
        import config
        old = config.ENABLE_ESCAPE_MOTION
        config.ENABLE_ESCAPE_MOTION = True
        try:
            base = SensorFusion().build(self.robot, [], self.radar)
            danger = EscapePlanner().plan(base, RiskState(level="DANGER"))
            fresh = SimpleNamespace(received=True, age_s=0)
            safety = SafetyManager()
            self.assertEqual(safety.validate(camera_ok=True, world=base, plan=danger,
                                             telemetry=fresh).status, "RUN")
            warn = EscapePlanner().plan(base, RiskState(level="WARN"))
            self.assertEqual(safety.validate(camera_ok=True, world=base, plan=warn,
                                             telemetry=fresh).status, "SLOW")
            safe = EscapePlanner().plan(base, RiskState(level="SAFE"))
            self.assertEqual(safety.validate(camera_ok=True, world=base, plan=safe,
                                             telemetry=fresh).status, "STOP")
            self.assertEqual(safety.validate(camera_ok=False, world=base, plan=danger,
                                             telemetry=fresh).reason, "camera offline")
            lost = SensorFusion().build(RobotPose(), [], self.radar)
            self.assertEqual(safety.validate(camera_ok=True, world=lost, plan=danger,
                                             telemetry=fresh).reason, "robot ArUco lost")
            no_radar = SensorFusion().build(self.robot, [], RadarState())
            self.assertEqual(safety.validate(camera_ok=True, world=no_radar, plan=danger,
                                             telemetry=fresh).reason, "radar disconnected")
            self.assertEqual(safety.validate(camera_ok=True, world=base, plan=danger,
                                             telemetry=SimpleNamespace(received=True, age_s=1)).reason,
                             "ESP32 telemetry stale")
            blocked = SensorFusion().build(self.robot, [
                obstacle(20 * math.cos(math.radians(a)), 20 * math.sin(math.radians(a)))
                for a in range(0, 360, 45)
            ], self.radar)
            no_path = EscapePlanner().plan(blocked, RiskState(level="DANGER"))
            self.assertEqual(no_path.status, "STOP")
            self.assertEqual(safety.validate(camera_ok=True, world=blocked, plan=no_path,
                                             telemetry=fresh).status, "STOP")
        finally:
            config.ENABLE_ESCAPE_MOTION = old


if __name__ == "__main__":
    unittest.main()
