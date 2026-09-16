# GRISE 3-Node Codex Handoff

이 패키지는 사용자가 제공한 `grise-master`를 바탕으로 **노트북 / Raspberry Pi / ESP32** 역할을 분리한 1차 구현 골격입니다.

```text
Laptop
  USB Camera -> MJPEG -> Raspberry Pi
  Raspberry Pi status <- UDP -> Monitor

Raspberry Pi
  NetworkCamera -> YOLO + ArUco
                    + Radar(interface placeholder)
        -> Sensor Fusion
        -> Risk Evaluator
        -> Escape Planner
        -> Safety Manager
        -> UDP {vx,vy,w,status} -> ESP32
        <- UDP Telemetry <- ESP32
        -> Monitor JSON -> Laptop

ESP32
  UDP command -> 3WD inverse kinematics -> Encoder PID -> TB6612 -> Motor x3
  300ms watchdog / Wi-Fi loss / invalid command -> STOP
  encoder telemetry -> Raspberry Pi
```

## 네트워크 포트

- Laptop MJPEG HTTP: `8080`
- Pi -> ESP32 command UDP: `8888`
- ESP32 -> Pi telemetry UDP: `8889`
- Pi -> Laptop monitor UDP: `9001`
- Future radar placeholder UDP: `8890`

## 기존 grise에서 유지한 값

- Camera: 1280x720, 30FPS, no horizontal flip
- YOLO: conf 0.45, imgsz 640, every 2 frames
- ArUco: DICT_4X4_50, robot ID 0, marker 8cm, pose EMA 0.5
- Risk initial thresholds: DANGER 15cm, WARN 35cm, approach 25cm/s, TTC 0.8/1.6s
- Pi motion limit: 35cm/s, 1.8rad/s, accel 90cm/s^2
- ESP32: 10.182.7.50, UDP 8888, command watchdog 300ms
- ESP32 geometry: wheel radius 2.9cm, robot radius 9.0cm, wheel angles 0/120/240 deg
- Encoder: 11 PPR x 74.83 reduction, A rising edge
- PID initial: Kp 3.0, Ki 8.0, Kd 0.05, control loop 100Hz
- Motor pins and TB6612 STBY are preserved from the provided sketch.

## 새로 추가한 값/인터페이스

- ESP32 telemetry UDP 8889 @ ~5Hz (200ms)
- Pi monitor UDP 9001 @ 5Hz
- Radar placeholder UDP 8890
- MVP escape distance: WARN 15cm / DANGER 30cm

PROJECT H는 자동 회피 목표의 거리 종료를 ESP32에서 수행한다. 엔코더 배율과 실제 15/30cm 이동 오차는 실기 보정이 필요하다. 세부 계약은 [Odometry & Distance Control](docs/ODOMETRY_DISTANCE_CONTROL.md)에 있다.
- Automatic escape motion is **disabled by default** until integration tests pass.

## 중요

Wi-Fi SSID/password는 보안상 산출물에서 `CHANGE_ME`로 바꿨습니다. 실제 환경 값은 직접 입력하세요.
`10.182.7.110`은 기존 ESP32 코드의 gateway 값이며 노트북 IP라고 확정할 수 없으므로 실제 네트워크에서 확인해야 합니다.
