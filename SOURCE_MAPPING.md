# Source mapping

사용자 제공 `grise-master.zip`에서 직접 유지/이식한 핵심:

- `comm/udp_sender.py` -> `raspberry_pi/comm/udp_sender.py` (그대로)
- `planning/transform.py` -> `raspberry_pi/planning/transform.py` (그대로)
- `perception/object_detector.py` -> `raspberry_pi/perception/object_detector.py` (그대로)
- `perception/camera.py` + `marker_scanner.py` + `robot_tracker.py` -> `raspberry_pi/perception/robot_tracker.py`로 자립형 통합
- `decision/risk_evaluator.py`의 거리/접근속도/TTC/EMA/hold 개념과 수치 -> radar 기반 evaluator로 이전
- `esp32/esp32_omni_controller.ino` -> 기존 제어를 보존하고 telemetry 송신만 추가

현재 설계 변경 때문에 의도적으로 제외한 기존 중앙 기능:

- MediaPipe Pose / Hand / Gaze 중심의 컵 접근 위험판정
- 기존 cup-attraction Potential Field 중심의 주행

이들은 필요 시 향후 기능으로 되살릴 수 있지만, 현재 목표인 "전방 radar 접근 감지 + 외부 camera 공간 확인 + 회피" MVP의 필수 경로에서는 제외한다.
