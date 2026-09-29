import importlib.util
import json
import math
import socket
import sys
import time
import unittest
from pathlib import Path


TOOLS = Path(__file__).resolve().parents[1] / "tools"
SCRIPT = TOOLS / "laptop_central_controller.py"
FIRMWARE = (Path(__file__).resolve().parents[1] / "arduino" /
            "esp32_omni_controller" / "esp32_omni_controller.ino")
sys.path.insert(0, str(TOOLS))


def load_controller():
    spec = importlib.util.spec_from_file_location("laptop_central_controller", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def telemetry(seq=1):
    return {"type": "telemetry", "seq": seq, "ms": 100, "status": "STOP",
            "mode": "NETWORK", "rssi": -42, "counts": [1, 2, 3],
            "rpm": [0.1, 0.2, 0.3], "wheel_speed": [0.0, 0.0, 0.0],
            "target_speed": [0.0, 0.0, 0.0]}


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

    def test_korean_operator_labels_keep_wire_values_unchanged(self):
        self.assertEqual(self.module.STATE_LABEL_KO["TELEMETRY_LOST"], "텔레메트리 끊김")
        self.assertEqual(self.module.STATUS_LABEL_KO["STOP"], "정지 (STOP)")
        self.assertEqual(self.module.MODE_LABEL_KO["NETWORK"], "네트워크 (NETWORK)")
        packet = self.module.CommandSequence().packet(1.0, 0, 0, 0, "STOP")
        self.assertEqual(packet["status"], "STOP")

    def test_firmware_uses_dhcp_without_changing_udp_safety_contract(self):
        source = FIRMWARE.read_text(encoding="utf-8")
        self.assertIn("constexpr bool USE_STATIC_IP = false;", source)
        self.assertIn("if (USE_STATIC_IP) {", source)
        self.assertIn("WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS1)", source)
        self.assertIn("constexpr uint16_t UDP_PORT = 8888;", source)
        self.assertIn("constexpr uint16_t TELEMETRY_PORT = 8889;", source)
        self.assertIn("constexpr uint32_t CMD_TIMEOUT_MS = 300;", source)
        self.assertIn("controllerIp = remoteIp;", source)

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
