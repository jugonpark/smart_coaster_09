import json
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "raspberry_pi"))

import config
from core.shared_state import SharedState
from core.control_loop import ControlLoop
from decision.risk_evaluator import RiskEvaluator
from fusion.sensor_fusion import SensorFusion
from perception.radar import RadarState, RadarTarget, parse_fake_packet
from perception.robot_tracker import RobotPose
from perception.vision_state import build_vision_state


NOW = 100.0


def vision(*, sectors=None, at=NOW, available=True):
    state = build_vision_state(camera_ok=True, captured_at=at,
                               robot=RobotPose(True, 0, 0, 0), detections=[],
                               detector_available=available, now=at)
    if sectors:
        state.sectors.update(sectors)
    return state


def target(distance, angle=0, speed=0, target_id=None):
    result = {"distance_cm": distance, "angle_deg": angle,
              "approach_speed_cm_s": speed}
    if target_id is not None:
        result["target_id"] = target_id
    return result


def radar(*targets, at=NOW):
    return parse_fake_packet(json.dumps({"targets": targets}).encode(), received_at=at)


def world(v=None, r=None, now=NOW + 0.1):
    return SensorFusion().build_from_states(v or vision(), r or radar(), now=now)


class FusionRiskTests(unittest.TestCase):
    def test_valid_no_target_is_safe_and_preserves_environment(self):
        result = world(vision(sectors={"RIGHT": "BLOCKED"}))
        risk = RiskEvaluator().evaluate(result, now=NOW + 0.1)
        self.assertTrue(result.vision_valid and result.radar_valid and result.sensor_valid)
        self.assertEqual(result.radar_status, "NO_TARGET")
        self.assertFalse(result.threat.detected)
        self.assertIsNone(result.radar_target)
        self.assertEqual(result.sectors["RIGHT"], "BLOCKED")
        self.assertFalse(result.direction_clear("RIGHT"))
        self.assertEqual(risk.level, "SAFE")

    def test_offline_stale_parse_error_and_vision_failure_are_invalid(self):
        offline = RadarState()
        stale = radar(target(20), at=NOW - 1)
        error = RadarState(True, None, 0, "udp_fake", False, NOW, status="PARSE_ERROR")
        cases = [world(r=offline), world(r=stale), world(r=error),
                 world(v=vision(available=False))]
        for result in cases:
            self.assertFalse(result.sensor_valid)
            self.assertEqual(RiskEvaluator().evaluate(result).level, "INVALID")
        self.assertFalse(cases[0].radar_valid)
        self.assertFalse(cases[1].radar_valid)
        self.assertFalse(cases[2].radar_valid)
        self.assertFalse(cases[3].vision_valid)
        self.assertFalse(cases[3].direction_clear("FRONT"))

    def test_stationary_or_receding_target_is_not_approaching(self):
        far = world(r=radar(target(70, speed=-10)))
        self.assertFalse(far.threat.detected)
        self.assertIsNone(far.threat.ttc_s)
        self.assertEqual(RiskEvaluator().evaluate(far).level, "SAFE")
        close = world(r=radar(target(10, speed=-10)))
        risk = RiskEvaluator().evaluate(close)
        self.assertFalse(close.threat.detected)
        self.assertEqual(risk.level, "DANGER")
        self.assertEqual(risk.reason, "DISTANCE")

    def test_warn_ttc_danger_and_far_safe(self):
        cases = [(target(30, speed=5), "WARN", "DISTANCE"),
                 (target(40, speed=100), "DANGER", "TTC"),
                 (target(100, speed=10), "SAFE", "normal")]
        for item, level, reason in cases:
            with self.subTest(item=item):
                result = world(r=radar(item))
                risk = RiskEvaluator().evaluate(result)
                self.assertEqual((risk.level, risk.reason), (level, reason))
                self.assertTrue(result.threat.detected)
        self.assertAlmostEqual(world(r=radar(target(40, speed=100))).threat.ttc_s, 0.4)

    def test_angle_mount_and_sector_boundaries(self):
        for angle, expected in ((0, "FRONT"), (90, "LEFT"), (-90, "RIGHT"),
                                (22.5, "FRONT_LEFT"), (-22.5, "FRONT")):
            with self.subTest(angle=angle):
                self.assertEqual(world(r=radar(target(50, angle, 30))).threat.direction, expected)
        with patch.object(config, "RADAR_MOUNT_YAW_DEG", 90):
            self.assertEqual(world(r=radar(target(50, 0, 30))).threat.direction, "LEFT")
            self.assertEqual(world(r=radar(target(50, -90, 30))).threat.direction, "FRONT")

    def test_multi_target_selects_risk_then_ttc_then_distance(self):
        result = world(r=radar(target(40, speed=0), target(100, speed=100)))
        self.assertEqual(result.radar_target.distance_cm, 100)
        self.assertEqual(len(result.radar_targets), 2)
        self.assertEqual(RiskEvaluator().evaluate(result).level, "WARN")
        tied = world(r=radar(target(30, speed=10), target(20, speed=10)))
        self.assertEqual(tied.radar_target.distance_cm, 20)
        ttc_tie = world(r=radar(target(30, speed=30), target(20, speed=30)))
        self.assertEqual(ttc_tie.radar_target.distance_cm, 20)
        close_static = world(r=radar(target(10, speed=0), target(100, speed=10)))
        self.assertEqual(close_static.radar_target.distance_cm, 10)
        self.assertTrue(close_static.approaching_target_present)
        self.assertFalse(close_static.threat.approaching)

    def test_hold_on_valid_empty_frame_but_invalid_clears_hold(self):
        evaluator = RiskEvaluator()
        danger = world(r=radar(target(10, speed=40)))
        self.assertEqual(evaluator.evaluate(danger, now=100.1).level, "DANGER")
        empty = world(r=radar(at=100.2), now=100.2)
        held = evaluator.evaluate(empty, now=100.2)
        self.assertEqual(held.level, "DANGER")
        self.assertIn("hold", held.reason)
        invalid = world(r=RadarState(), now=100.3)
        self.assertEqual(evaluator.evaluate(invalid, now=100.3).level, "INVALID")
        fresh_empty = world(v=vision(at=100.4), r=radar(at=100.4), now=100.4)
        self.assertEqual(evaluator.evaluate(fresh_empty, now=100.4).level, "SAFE")

    def test_target_id_change_resets_ema_and_hold(self):
        evaluator = RiskEvaluator()
        first = world(r=radar(target(10, speed=40, target_id=1)))
        self.assertEqual(evaluator.evaluate(first, now=100.1).level, "DANGER")
        second = world(r=radar(target(100, speed=0, target_id=2), at=100.2), now=100.2)
        self.assertEqual(evaluator.evaluate(second, now=100.2).level, "SAFE")

    def test_receding_target_does_not_reuse_approach_ema(self):
        evaluator = RiskEvaluator()
        approaching = world(r=radar(target(40, speed=100, target_id=1)))
        self.assertEqual(evaluator.evaluate(approaching, now=100.1).level, "DANGER")
        receding = world(v=vision(at=101),
                         r=radar(target(30, speed=-10, target_id=1), at=101), now=101)
        result = evaluator.evaluate(receding, now=101)
        self.assertEqual(result.level, "WARN")
        self.assertEqual(result.approach_speed_cm_s, 0)
        self.assertIsNone(result.ttc_s)

    def test_nonfinite_target_and_timestamp_skew_fail_closed(self):
        bad = RadarTarget(math.nan, 0, 10, timestamp=NOW)
        state = RadarState(True, bad, 0, "fake", True, NOW, None, (bad,), "TARGET_DETECTED")
        self.assertFalse(world(r=state).radar_valid)
        with patch.object(config, "VISION_RADAR_MAX_SKEW_S", 0.1):
            skewed = world(v=vision(at=100.1), r=radar(at=100.4), now=100.4)
        self.assertFalse(skewed.radar_valid)

    def test_invalid_world_stops_existing_control_boundary(self):
        shared = SharedState()
        shared.publish_world(world(r=RadarState()), captured_at=NOW)
        sender = SimpleNamespace(send=lambda *args, **kwargs: None)
        telemetry = SimpleNamespace(latest=lambda: SimpleNamespace(received=True, age_s=0))
        command = ControlLoop(shared, telemetry, sender).tick(now=NOW + 0.1)
        self.assertEqual(command.status, "STOP")
        self.assertEqual(shared.snapshot().risk.level, "INVALID")


if __name__ == "__main__":
    unittest.main()
