# GRISE Raspberry Pi Central Controller

이 폴더가 기존 노트북 `grise-master`의 **중앙 관제 역할을 Raspberry Pi로 이전하기 위한 기준 구조**입니다.

```text
main.py
config.py
perception/
├─ object_detector.py      # 기존 grise 기반
├─ robot_tracker.py        # 기존 camera/marker_scanner/robot_tracker 통합
├─ network_camera.py       # 노트북 MJPEG 수신
└─ radar.py                # 현재는 인터페이스/UDP placeholder
fusion/
└─ sensor_fusion.py
decision/
└─ risk_evaluator.py       # 기존 위험 기준을 radar 입력 형태로 이전
planning/
├─ escape_planner.py       # 8방향 후보 기반 MVP
└─ transform.py            # 기존 grise 그대로
safety/
└─ safety_manager.py
comm/
├─ udp_sender.py           # 기존 grise 그대로
├─ telemetry_receiver.py
└─ monitor_sender.py
```

## 설계 원칙

- Pi: 인식, 센서융합, 위험판단, 회피방향/거리 결정, 최종 안전검사
- ESP32: 3WD 역기구학, Encoder, PID, PWM, watchdog
- Laptop: 카메라 스트리밍 + 모니터링만

## 최초 실행 순서

1. 노트북 `main.py` 실행 후 MJPEG 주소 확인.
2. Pi에서 `GRISE_LAPTOP_IP`를 실제 노트북 IP로 설정.
3. 처음에는 `GRISE_ENABLE_ESCAPE_MOTION=0` 상태로 관제 로그만 검증.
4. ESP32 telemetry 수신까지 확인한 뒤 실제 모션을 단계적으로 활성화.
5. radar.py는 이후 IWR6843AOP 실제 parser/transport로 교체.

### 중요한 값

기존 grise에서 유지한 초기값: 1280x720@30fps, ArUco ID 0/8cm, YOLO conf 0.45,
ESP32 10.182.7.50:8888, command 30Hz, risk 15/35cm, approach 25cm/s, TTC 0.8/1.6s,
Pi max speed 35cm/s / 1.8rad/s.

새 MVP 초기값: WARN 15cm 회피, DANGER 30cm 회피. 이 값들은 실제 실험 후 반드시 튜닝합니다.
