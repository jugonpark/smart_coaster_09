import importlib.util
import csv
import json
import math
import socket
import sys
import time
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch


TOOLS = Path(__file__).resolve().parents[1] / "tools"
SCRIPT = TOOLS / "laptop_central_controller.py"
FIRMWARE_DIR = (Path(__file__).resolve().parents[1] / "arduino" /
                "esp32_omni_controller")
FIRMWARE = FIRMWARE_DIR / "esp32_omni_controller.ino"
FIRMWARE_CONFIG = FIRMWARE_DIR / "robot_config.h"
FIRMWARE_TELEMETRY = FIRMWARE_DIR / "safety_telemetry.cpp"
sys.path.insert(0, str(TOOLS))


def load_controller():
    spec = importlib.util.spec_from_file_location("laptop_central_controller", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def telemetry(seq=1, pwm=None):
    payload = {"type": "telemetry", "seq": seq, "ms": 100, "status": "STOP",
               "mode": "NETWORK", "rssi": -42, "counts": [1, 2, 3],
               "rpm": [0.1, 0.2, 0.3], "wheel_speed": [0.0, 0.0, 0.0],
               "target_speed": [0.0, 0.0, 0.0]}
    if pwm is not None:
        payload["pwm"] = pwm
    return payload


class LaptopCentralControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_controller()

    def connected_control(self):
        control = self.module.ManualControl(telemetry_timeout_s=1.0)
        control.start_connection(now=9.0)
        control.on_telemetry(telemetry(), now=10.0)
        return control

    def test_connection_requires_stop_sync_period_and_telemetry(self):
        control = self.module.ManualControl(connect_stop_s=0.6)
        control.start_connection(now=10.0)
        control.on_telemetry(telemetry(), now=10.2)
        control.press("W")
        self.assertEqual(control.state, self.module.ControlState.DISCONNECTED)
        self.assertEqual(control.command(10, 0.3)[3], "STOP")
        control.on_telemetry(telemetry(2), now=10.61)
        self.assertEqual(control.state, self.module.ControlState.CONNECTED_STOPPED)

    def test_command_json_schema_and_sequence_increment(self):
        sequence = self.module.CommandSequence()
        first = sequence.packet(1.5, 10, 0, 0, "RUN")
        second = sequence.packet(1.6, 0, 0, 0, "STOP")
        self.assertEqual(set(first), {"seq", "t", "vx", "vy", "w", "status"})
        self.assertEqual((first["seq"], second["seq"]), (1, 2))
        self.assertNotIn("motion_id", json.dumps(first))

    def test_stop_packet_and_disconnected_run_block(self):
        control = self.module.ManualControl()
        control.press("W")
        self.assertEqual(control.command(10, 0.3), (0.0, 0.0, 0.0, "STOP"))
        self.assertEqual(control.state, self.module.ControlState.DISCONNECTED)

    def test_estop_latches_blocks_run_and_releases_to_stop(self):
        control = self.connected_control()
        control.press("W")
        self.assertEqual(control.command(10, 0.3)[3], "RUN")
        control.activate_estop()
        control.press("W")
        self.assertEqual(control.state, self.module.ControlState.ESTOP)
        self.assertEqual(control.command(10, 0.3), (0.0, 0.0, 0.0, "STOP"))
        control.release_estop(now=10.1)
        self.assertEqual(control.state, self.module.ControlState.CONNECTED_STOPPED)
        self.assertEqual(control.command(10, 0.3)[3], "STOP")

    def test_estop_remains_latched_across_disconnect_and_reconnect(self):
        control = self.connected_control()
        control.activate_estop()
        control.disconnect()
        control.start_connection(now=10.0)
        control.on_telemetry(telemetry(2), now=11.0)
        self.assertEqual(control.state, self.module.ControlState.ESTOP)
        control.press("W")
        self.assertEqual(control.command(10, 0.3)[3], "STOP")
        control.release_estop(now=11.1)
        self.assertEqual(control.state, self.module.ControlState.CONNECTED_STOPPED)

    def test_timeout_stops_and_restore_does_not_resume_old_input(self):
        control = self.connected_control()
        control.press("W")
        self.assertEqual(control.command(10, 0.3)[3], "RUN")
        self.assertTrue(control.check_timeout(now=11.01))
        self.assertEqual(control.state, self.module.ControlState.TELEMETRY_LOST)
        self.assertEqual(control.command(10, 0.3)[3], "STOP")
        control.on_telemetry(telemetry(2), now=11.1)
        self.assertEqual(control.state, self.module.ControlState.CONNECTED_STOPPED)
        self.assertEqual(control.command(10, 0.3)[3], "STOP")
        control.press("W")
        self.assertEqual(control.command(10, 0.3)[3], "RUN")

    def test_expected_ik_matches_firmware(self):
        targets = self.module.expected_wheel_targets(10, 0, 0)
        self.assertAlmostEqual(targets[0], 0.0, places=6)
        self.assertAlmostEqual(targets[1], -5 * math.sqrt(3), places=5)
        self.assertAlmostEqual(targets[2], 5 * math.sqrt(3), places=5)

    def test_diagonal_is_clamped_to_linear_speed(self):
        control = self.connected_control()
        control.press("W")
        control.press("A")
        vx, vy, w, status = control.command(10, 0.3)
        self.assertEqual((w, status), (0.0, "RUN"))
        self.assertAlmostEqual(math.hypot(vx, vy), 10.0)
        self.assertAlmostEqual(vx, vy)

    def test_invalid_telemetry_is_rejected(self):
        good = telemetry()
        self.assertEqual(self.module.validate_telemetry(good).seq, 1)
        for bad in ({**good, "counts": [1, 2]},
                    {**good, "rpm": [0, float("nan"), 0]},
                    {**good, "mode": None}):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    self.module.validate_telemetry(bad)

    def test_pwm_telemetry_and_backward_compatibility(self):
        current = self.module.validate_telemetry(telemetry(pwm=[0, -100, 120]))
        legacy = self.module.validate_telemetry(telemetry())
        self.assertEqual(current.pwm, (0, -100, 120))
        self.assertEqual(legacy.pwm, (None, None, None))
        with self.assertRaises(ValueError):
            self.module.validate_telemetry(telemetry(pwm=[0, 300, 0]))

    def test_wheel_speed_error_and_tracking_status(self):
        self.assertAlmostEqual(8.66 - 8.10, 0.56)
        self.assertAlmostEqual(-8.66 - -8.10, -0.56)
        self.assertEqual(self.module.wheel_tracking_status(8.66, -8.10), "방향 이상")
        self.assertEqual(self.module.wheel_tracking_status(0.1, 0.0), "정지")
        self.assertEqual(self.module.wheel_tracking_status(8.66, 7.20), "느림")
        self.assertEqual(self.module.wheel_tracking_status(8.66, 9.80), "빠름")

    def test_motor_snapshot_uses_received_values(self):
        payload = telemetry(pwm=[0, -105, 123])
        payload.update({"counts": [123, -3210, 3022],
                        "rpm": [0.1, -28.4, 26.2],
                        "wheel_speed": [0.02, -8.51, 7.92],
                        "target_speed": [0.0, -8.66, 8.66]})
        state = self.module.validate_telemetry(payload)
        snapshot = self.module.format_motor_snapshot(state)
        self.assertIn("M2 Target=-8.66 Measured=-8.51 RPM=-28.4 PWM=-105 Count=-3210", snapshot)
        self.assertIn("활성 바퀴 measured magnitude 최대 편차 = 0.59 cm/s", snapshot)

    def test_session_logger_interval_three_rows_and_excel_encoding(self):
        logger = self.module.SessionLogger(interval_s=1.0)
        path = logger.start(now=10.0)
        try:
            self.assertTrue(path.exists())
            self.assertTrue(logger.due(now=10.0))
            for now in (10.0, 11.0, 12.0):
                logger.append_snapshot({"status": "STOP", "m1_pwm": -100}, now=now)
            self.assertEqual(logger.row_count, 3)
            raw = path.read_bytes()
            self.assertTrue(raw.startswith(b"\xef\xbb\xbf"))
            with path.open("r", newline="", encoding="utf-8-sig") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 3)
            self.assertEqual(rows[0]["status"], "STOP")
            self.assertEqual(rows[0]["m1_pwm"], "-100")
            self.assertFalse(logger.due(now=12.5))
            self.assertTrue(logger.due(now=13.0))
        finally:
            logger.cleanup()
        self.assertFalse(path.exists())

    def test_log_snapshot_missing_telemetry_keeps_motor_fields_blank(self):
        measurement = self.module.MotionMeasurement().snapshot(now=1.0)
        row = self.module.build_log_snapshot(
            esp32_ip="10.232.69.103", connected=True,
            command=(0.0, 0.0, 0.0, "STOP"), motion=measurement,
            telemetry=None, telemetry_age_ms=1500.0,
            diagnostics={"sent": 4, "received": 0, "valid": 0, "invalid": 0})
        self.assertEqual(row["telemetry_valid"], "false")
        self.assertEqual(row["status"], "STOP")
        self.assertNotIn("m1_measured_cm_s", row)
        self.assertEqual(row["udp_sent_count"], 4)

    def test_log_snapshot_records_all_motor_fields(self):
        state = self.module.validate_telemetry(telemetry(pwm=[0, -100, 120]))
        measurement = self.module.MotionMeasurement().snapshot(now=1.0)
        row = self.module.build_log_snapshot(
            esp32_ip="10.232.69.103", connected=True,
            command=(10.0, 0.0, 0.0, "RUN"), motion=measurement,
            telemetry=state, telemetry_age_ms=10.0,
            diagnostics={"sent": 5, "received": 3, "valid": 3, "invalid": 0})
        self.assertEqual(row["telemetry_valid"], "true")
        self.assertEqual(row["m2_target_cm_s"], "0.000")
        self.assertEqual(row["m2_error_cm_s"], "0.000")
        self.assertEqual(row["m2_pwm"], -100)
        self.assertEqual(row["telemetry_accepted_count"], 3)

    def test_session_export_survives_temp_cleanup_and_reset_deletes_temp(self):
        logger = self.module.SessionLogger()
        temp_path = logger.start(now=1.0)
        logger.append_snapshot({"status": "STOP"}, now=1.0)
        repo_root = Path(__file__).resolve().parents[1]
        with tempfile.NamedTemporaryFile(
                suffix=".csv", delete=False, dir=repo_root) as destination:
            permanent = Path(destination.name)
        permanent.unlink()
        try:
            logger.export_csv(permanent)
            self.assertTrue(permanent.exists())
            logger.cleanup()
            self.assertFalse(temp_path.exists())
            self.assertTrue(permanent.exists())
            self.assertIn("status", permanent.read_text(encoding="utf-8-sig"))
        finally:
            permanent.unlink(missing_ok=True)

        reset_path = logger.start(now=2.0)
        logger.append_snapshot({"status": "STOP"}, now=2.0)
        logger.reset()
        self.assertFalse(reset_path.exists())
        self.assertEqual(logger.row_count, 0)

    def test_unsaved_session_confirmation_can_cancel_discard(self):
        logger = self.module.SessionLogger()
        logger.start(now=1.0)
        logger.append_snapshot({"status": "STOP"}, now=1.0)
        app = object.__new__(self.module.CentralControllerApp)
        app.session_logger = logger
        app.root = None
        try:
            with patch("tkinter.messagebox.askokcancel", return_value=False) as confirm:
                self.assertFalse(app._confirm_discard_log())
            confirm.assert_called_once()
            self.assertTrue(logger.temp_path.exists())
        finally:
            logger.cleanup()

    def test_korean_operator_labels_keep_wire_values_unchanged(self):
        self.assertEqual(self.module.STATE_LABEL_KO["TELEMETRY_LOST"], "텔레메트리 끊김")
        self.assertEqual(self.module.STATUS_LABEL_KO["STOP"], "정지 (STOP)")
        self.assertEqual(self.module.MODE_LABEL_KO["NETWORK"], "네트워크 (NETWORK)")
        packet = self.module.CommandSequence().packet(1.0, 0, 0, 0, "STOP")
        self.assertEqual(packet["status"], "STOP")

    def test_firmware_uses_dhcp_without_changing_udp_safety_contract(self):
        source = FIRMWARE.read_text(encoding="utf-8")
        config = FIRMWARE_CONFIG.read_text(encoding="utf-8")
        telemetry_source = FIRMWARE_TELEMETRY.read_text(encoding="utf-8")
        self.assertIn("WiFi.begin(WIFI_SSID, WIFI_PASSWORD)", source)
        self.assertNotIn("WiFi.config(", source)
        self.assertIn("constexpr uint16_t UDP_COMMAND_PORT = 8888U;", config)
        self.assertIn("constexpr uint16_t UDP_TELEMETRY_PORT = 8889U;", config)
        self.assertIn("constexpr uint32_t COMMAND_WATCHDOG_MS = 300U;", config)
        self.assertIn("controllerIp = remoteIp;", source)
        self.assertIn('addIntArray(root, "pwm", motorEncoderState.currentPwm);',
                      telemetry_source)

    def test_motion_measurement_linear_distance(self):
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=10.0, vy=0.0, w=0.0, label=("W",), now=1.0)
        result = measurement.finish(now=3.0)
        self.assertAlmostEqual(result.elapsed_s, 2.0)
        self.assertAlmostEqual(result.expected_distance_cm, 20.0)
        self.assertAlmostEqual(result.expected_angle_deg, 0.0)

    def test_motion_measurement_slower_linear_distance(self):
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=5.0, vy=0.0, w=0.0, label=("W",), now=1.0)
        result = measurement.finish(now=4.0)
        self.assertAlmostEqual(result.expected_distance_cm, 15.0)

    def test_motion_measurement_diagonal_uses_final_command_vector(self):
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=6.0, vy=8.0, w=0.0,
                                    label=("W", "A"), now=1.0)
        result = measurement.finish(now=3.0)
        self.assertAlmostEqual(result.linear_speed_cm_s, 10.0)
        self.assertAlmostEqual(result.expected_distance_cm, 20.0)

    def test_motion_measurement_rotation_angle(self):
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=0.0, vy=0.0, w=0.5, label=("Q",), now=1.0)
        result = measurement.finish(now=3.0)
        self.assertAlmostEqual(result.expected_angle_rad, 1.0)
        self.assertAlmostEqual(result.expected_angle_deg, math.degrees(1.0))

    def test_motion_measurement_stop_release_and_new_run(self):
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=10.0, vy=0.0, w=0.0, label=("W",), now=1.0)
        first = measurement.finish(now=2.0)
        self.assertAlmostEqual(first.expected_distance_cm, 10.0)
        measurement.begin_or_update(vx=5.0, vy=0.0, w=0.0, label=("S",), now=5.0)
        second = measurement.finish(now=8.0)
        self.assertAlmostEqual(second.elapsed_s, 3.0)
        self.assertAlmostEqual(second.expected_distance_cm, 15.0)

    def test_motion_measurement_reset_does_not_change_control_command(self):
        control = self.connected_control()
        control.press("W")
        command_before = control.command(10.0, 0.3)
        measurement = self.module.MotionMeasurement()
        measurement.begin_or_update(vx=command_before[0], vy=command_before[1],
                                    w=command_before[2], label=("W",), now=1.0)
        measurement.reset(now=2.0)
        self.assertEqual(control.command(10.0, 0.3), command_before)
        self.assertAlmostEqual(measurement.snapshot(now=2.0).expected_distance_cm, 0.0)

    def test_udp_diagnostics_distinguish_datagram_from_valid_telemetry(self):
        command_receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        command_receiver.bind(("127.0.0.1", 0))
        events = []
        control = self.module.ManualControl()
        client = self.module.Esp32UdpClient(
            "127.0.0.1", command_receiver.getsockname()[1], 0, control,
            event_sink=lambda kind, value: events.append((kind, value)))
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            client.start()
            telemetry_port = client.sock.getsockname()[1]
            sender.sendto(b"not-json", ("127.0.0.1", telemetry_port))
            deadline = time.monotonic() + 1.0
            while client.invalid_telemetry_packets == 0 and time.monotonic() < deadline:
                time.sleep(0.01)
            diagnostics = client.diagnostics()
            self.assertGreaterEqual(diagnostics["sent"], 1)
            self.assertEqual(diagnostics["received"], 1)
            self.assertEqual(diagnostics["valid"], 0)
            self.assertEqual(diagnostics["invalid"], 1)
            self.assertTrue(any(kind == "transport_started" for kind, _ in events))
            self.assertTrue(any(kind == "invalid_telemetry" for kind, _ in events))
        finally:
            client.disconnect()
            sender.close()
            command_receiver.close()

    def test_disconnect_sends_stop_burst(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(0.1)
        control = self.module.ManualControl()
        client = self.module.Esp32UdpClient(
            "127.0.0.1", receiver.getsockname()[1], 0, control,
            rate_hz=20, stop_duration_s=0.15)
        try:
            client.start()
            time.sleep(0.06)
            client.disconnect()
            packets = []
            while True:
                try:
                    packets.append(json.loads(receiver.recv(4096)))
                except socket.timeout:
                    break
            self.assertGreaterEqual(len(packets), 3)
            self.assertTrue(all(packet["status"] == "STOP" for packet in packets))
            self.assertEqual([packet["seq"] for packet in packets],
                             sorted({packet["seq"] for packet in packets}))
        finally:
            client.disconnect()
            receiver.close()


if __name__ == "__main__":
    unittest.main()
