from __future__ import annotations

import time
import threading

import config
from comm import MonitorSender, TelemetryReceiver, UdpSender
from core.control_loop import ControlLoop, run_monitor_loop
from core.shared_state import SharedState
from fusion import SensorFusion
from perception import NetworkCamera, ObjectDetector, RadarReceiver, RobotTracker, WorldFrame
from perception.vision_state import build_vision_state


def _build_detector():
    try:
        return ObjectDetector()
    except Exception as e:
        print(f"[YOLO] init failed -> detector unavailable: {e}")
        return None


def perception_step(camera, world_frame, robot_tracker, detector, radar, fusion,
                    shared: SharedState) -> bool:
    """Acquire a frame and publish only a complete WorldState from that frame."""
    camera_ok = False
    try:
        camera_ok, frame = camera.read()
        now = time.monotonic()
        captured_at = getattr(camera, "last_ok_t", 0.0) or now
        if not camera_ok or frame is None:
            shared.publish_failure("camera offline", camera_ok=False)
            return False
        if captured_at > now or now - captured_at > config.WORLD_STALE_S:
            shared.publish_failure("camera frame stale", camera_ok=False)
            return False
        robot = robot_tracker.process(frame, world_frame)
        detector_available = bool(detector and getattr(detector, "available", detector.backend != "stub"))
        try:
            detections = detector.detect(frame, world_frame) if detector else []
        except Exception as exc:
            print(f"[YOLO] inference failed: {exc}")
            detections = []
            detector_available = False
        if detector:
            detector_available = detector_available and getattr(detector, "available", True)
        vision = build_vision_state(
            camera_ok=True, captured_at=captured_at, robot=robot,
            detections=detections,
            detector_available=detector_available,
            now=time.monotonic(),
        )
        radar_state = radar.read()
        world = fusion.build_from_states(vision, radar_state, now=time.monotonic())
        shared.publish_world(world, captured_at=captured_at)
        return True
    except Exception as exc:
        reason = f"perception exception: {exc}"
        print(f"[PERCEPTION] {reason}")
        shared.publish_failure(reason, camera_ok=camera_ok)
        return False


def main() -> None:
    camera = NetworkCamera()
    world_frame = WorldFrame()
    robot_tracker = RobotTracker()
    detector = _build_detector()
    radar = RadarReceiver()
    fusion = SensorFusion()
    shared = SharedState()
    sender = UdpSender()
    telemetry_rx = TelemetryReceiver()
    monitor = MonitorSender()
    controller = ControlLoop(shared, telemetry_rx, sender)
    stopped = threading.Event()
    control_thread = threading.Thread(target=controller.run, args=(stopped,), daemon=True)
    monitor_thread = threading.Thread(
        target=run_monitor_loop, args=(shared, telemetry_rx, monitor, stopped), daemon=True
    )

    last_log = 0.0
    print("[PI] central controller started")
    print(f"[PI] camera={config.NETWORK_CAMERA_URL}")
    print(f"[PI] ESP32={config.ESP32_IP}:{config.ESP32_PORT}")
    print(f"[PI] automatic motion enabled={config.ENABLE_ESCAPE_MOTION}")

    try:
        control_thread.start()
        monitor_thread.start()
        while True:
            ok = perception_step(camera, world_frame, robot_tracker, detector,
                                 radar, fusion, shared)
            if not ok:
                time.sleep(0.03)
            now = time.monotonic()
            if now - last_log >= 1.0 / config.PRINT_HZ:
                state = shared.snapshot()
                print(f"[PI] state={state.health.state} cmd={state.command.status} "
                      f"reason={state.command.reason} ticks={state.timing.ticks} "
                      f"control_ms={state.timing.last_duration_s * 1000:.1f} "
                      f"missed={state.timing.missed_deadlines}")
                last_log = now
    except KeyboardInterrupt:
        pass
    finally:
        stopped.set()
        control_thread.join(timeout=1)
        monitor_thread.join(timeout=1)
        sender.close()
        monitor.close()
        telemetry_rx.close()
        radar.close()
        camera.close()
        print("[PI] stopped")


if __name__ == "__main__":
    main()
