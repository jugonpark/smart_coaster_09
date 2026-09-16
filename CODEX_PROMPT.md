# Codex implementation prompt

이 저장소는 GRISE 3-node skeleton이다. 기존 동작을 최대한 보존하면서 단계적으로 구현한다.

목표 아키텍처:
1. `laptop/`: USB 외부카메라를 MJPEG로 Pi에 전달하고 Pi 상태를 모니터링한다. 판단 로직은 넣지 않는다.
2. `raspberry_pi/`: 중앙 관제. NetworkCamera, ArUco robot pose, YOLO obstacle, future IWR6843AOP radar, sensor fusion, risk, escape planner, safety, ESP32 command, telemetry, laptop monitor를 담당한다.
3. `arduino/esp32_omni_controller.ino`: 실시간 모터 제어. UDP body command, 3WD inverse kinematics, encoder PID, fail-safe를 담당한다.

우선순위:
- 기존 `grise-master`에서 이미 검증된 UDP packet `{seq,t,vx,vy,w,status}`와 ESP32 kinematics/PID/watchdog을 깨지 않는다.
- 첫 단계는 Laptop -> Pi 영상, Pi -> ESP32 STOP/RUN command, ESP32 -> Pi telemetry, Pi -> Laptop monitoring 통신을 각각 독립 검증한다.
- Radar는 인터페이스만 유지하고 실제 IWR6843AOP parser는 별도 단계로 구현한다.
- 자동 motion은 `GRISE_ENABLE_ESCAPE_MOTION=0`이 기본이며 테스트가 끝나기 전 임의로 활성화하지 않는다.
- 위치/거리 기반 종료는 encoder odometry가 검증된 뒤 구현한다. 현재 `EscapePlan.target_distance_cm`는 planner 메타데이터다.
- 충돌 방지는 camera obstacle sector + 향후 근거리 센서 입력을 확장할 수 있게 유지한다.

테스트 시 각 단계에서 실패 원인을 네트워크/카메라/인식/판단/제어로 분리할 수 있게 로그를 남긴다.
