import json
import math
import sys
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from decision.risk_evaluator import RiskState
from fusion.sensor_fusion import SensorFusion
from perception.radar import parse_fake_packet
from perception.robot_tracker import RobotPose
from perception.vision_state import DIRECTIONS, build_vision_state
from planning.escape_planner import EscapePlanner
from safety.safety_manager import SafetyManager


NOW = 100.1


def scene(angle=0, sectors=None, *, camera_ok=True, detector_available=True):
    vision = build_vision_state(camera_ok=True, captured_at=100,
                                robot=RobotPose(True, 0, 0, 0), detections=[],
                                detector_available=True, now=100)
    vision.camera_ok = camera_ok
    vision.detector_available = detector_available
    if sectors:
        vision.sectors.update(sectors)
    radar = parse_fake_packet(json.dumps({"distance_cm": 20, "angle_deg": angle,
                                          "approach_speed_cm_s": 40}).encode(), received_at=100)
    return SensorFusion().build_from_states(vision, radar, now=NOW)


def risk(level):
    return RiskState(level=level, timestamp=NOW)


def safe_command(world, plan, level="DANGER", *, telemetry_age=0,
                 camera_ok=True, now=NOW):
    return SafetyManager().validate(
        camera_ok=camera_ok, world=world, plan=plan, risk=risk(level), now=now,
        telemetry=SimpleNamespace(received=True, age_s=telemetry_age))


class EscapeSafetyTests(unittest.TestCase):
    def test_eight_threat_directions_choose_exact_opposite(self):
        angles = (0, 45, 90, 135, 180, -135, -90, -45)
        for index, angle in enumerate(angles):
            world = scene(angle)
            plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
            with self.subTest(threat=DIRECTIONS[index]):
                self.assertTrue(plan.valid)
                self.assertEqual(plan.threat_direction, DIRECTIONS[index])
                self.assertEqual(plan.direction, DIRECTIONS[(index + 4) % 8])
                self.assertEqual(plan.candidate_directions[0], plan.direction)

    def test_candidate_order_blocked_and_unknown_fallback(self):
        world = scene(0, {"BACK": "BLOCKED", "BACK_LEFT": "UNKNOWN"})
        plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
        self.assertEqual(plan.candidate_directions,
                         ("BACK", "BACK_LEFT", "BACK_RIGHT", "LEFT", "RIGHT",
                          "FRONT_LEFT", "FRONT_RIGHT", "FRONT"))
        self.assertEqual(plan.direction, "BACK_RIGHT")
        self.assertEqual(plan.blocked_directions, ("BACK", "BACK_LEFT"))
        world = scene(0, {direction: "BLOCKED" for direction in
                          ("BACK", "BACK_LEFT", "BACK_RIGHT", "RIGHT")})
        self.assertEqual(EscapePlanner().plan(world, risk("DANGER"), now=NOW).direction, "LEFT")

    def test_all_blocked_or_unknown_stops(self):
        for value in ("BLOCKED", "UNKNOWN"):
            world = scene(0, dict.fromkeys(DIRECTIONS, value))
            plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
            self.assertFalse(plan.valid)
            self.assertEqual(plan.status, "STOP")
            with patch.object(config, "ENABLE_ESCAPE_MOTION", True):
                self.assertEqual(safe_command(world, plan).status, "STOP")

    def test_warn_danger_speed_distance_and_diagonal_magnitude(self):
        world = scene(0)
        warn = EscapePlanner().plan(world, risk("WARN"), now=NOW)
        danger = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
        self.assertEqual((warn.status, warn.speed_cm_s, warn.target_distance_cm),
                         ("SLOW", 15, 15))
        self.assertEqual((danger.status, danger.speed_cm_s, danger.target_distance_cm),
                         ("RUN", 25, 30))
        self.assertEqual((danger.vx, danger.vy, danger.w), (-25, 0, 0))
        diagonal = EscapePlanner().plan(scene(45), risk("DANGER"), now=NOW)
        self.assertAlmostEqual(math.hypot(diagonal.vx, diagonal.vy), 25)
        self.assertAlmostEqual(abs(diagonal.vx), 25 / math.sqrt(2))
        self.assertEqual(diagonal.w, 0)

    def test_safe_invalid_and_risk_transition_stop(self):
        world = scene()
        planner = EscapePlanner()
        self.assertEqual(planner.plan(world, risk("WARN"), now=NOW).status, "SLOW")
        self.assertEqual(planner.plan(world, risk("DANGER"), now=NOW).status, "RUN")
        for level in ("SAFE", "INVALID"):
            plan = planner.plan(world, risk(level), now=NOW)
            self.assertFalse(plan.valid)
            self.assertEqual(plan.status, "STOP")
            with patch.object(config, "ENABLE_ESCAPE_MOTION", True):
                self.assertEqual(safe_command(world, plan, level).status, "STOP")

    def test_auto_switch_and_telemetry_final_authority(self):
        world = scene()
        plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
        with patch.object(config, "ENABLE_ESCAPE_MOTION", False):
            command = safe_command(world, plan)
            self.assertEqual((command.status, command.vx, command.vy, command.w),
                             ("STOP", 0, 0, 0))
        with patch.object(config, "ENABLE_ESCAPE_MOTION", True):
            self.assertEqual(safe_command(world, plan).status, "RUN")
            for age in (config.TELEMETRY_STALE_S + 0.01, math.inf):
                self.assertEqual(safe_command(world, plan, telemetry_age=age).status, "STOP")

    def test_sensor_stale_and_plan_validation_stop(self):
        world = scene()
        plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
        with patch.object(config, "ENABLE_ESCAPE_MOTION", True):
            self.assertEqual(safe_command(world, plan, camera_ok=False).status, "STOP")
            self.assertEqual(safe_command(world, plan, now=NOW + config.WORLD_STALE_S + 0.01).status,
                             "STOP")
            for changed in (replace(plan, vx=math.nan), replace(plan, vx=100),
                            replace(plan, valid=False), replace(plan, direction="RIGHT"),
                            replace(plan, timestamp=99), replace(plan, status="SLOW")):
                self.assertEqual(safe_command(world, changed).status, "STOP")
            self.assertEqual(safe_command(world, plan, level="INVALID").status, "STOP")
        world.radar_valid = False
        self.assertFalse(EscapePlanner().plan(world, risk("DANGER"), now=NOW).valid)
        self.assertEqual(safe_command(world, plan).status, "STOP")
        world = scene()
        world.robot.detected = False
        self.assertEqual(safe_command(world, plan).status, "STOP")

    def test_unknown_direction_rechecked_after_planning(self):
        world = scene()
        plan = EscapePlanner().plan(world, risk("DANGER"), now=NOW)
        world.sectors[plan.direction] = "UNKNOWN"
        with patch.object(config, "ENABLE_ESCAPE_MOTION", True):
            self.assertEqual(safe_command(world, plan).status, "STOP")


if __name__ == "__main__":
    unittest.main()
