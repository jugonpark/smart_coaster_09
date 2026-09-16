# GRISE 구조와 개발 경계

## 3개 노드

Laptop은 USB 카메라를 HTTP MJPEG 8080으로 송출하고 Pi의 UDP 9001 상태를 표시한다. 위험 판단과 모터 제어는 하지 않는다.

Raspberry Pi 5는 MJPEG 수신, ArUco/YOLO, radar 입력, 융합, 위험도, 회피 방향, 최종 안전 판단을 수행한다. ESP32에 UDP 8888로 `{seq,t,vx,vy,w,status}`를 보내고 UDP 8889 telemetry를 받는다. `vx/vy`는 cm/s, `w`는 rad/s, 상태는 `RUN/SLOW/STOP`이다. Radar UDP 8890은 현재 가짜 입력 경계다.

ESP32는 수신 속도를 3WD 역기구학·encoder PID·TB6612 출력으로 변환하고 telemetry를 보낸다. 명령 300 ms 단절, Wi-Fi 상실, 유효하지 않은 패킷, STOP은 모터 정지 조건이다. 위험도와 회피 방향은 계산하지 않는다.

## 하위 프로젝트

| 프로젝트 | 독립 결과 |
|---|---|
| A Pi Core Controller | 공유 스냅샷, 독립 인식/30 Hz 제어/5 Hz 모니터 루프, stale/예외 STOP |
| B Laptop Camera & Monitoring | 카메라 송출과 상태 표시 |
| C ESP32 Motion Controller | 3WD 실시간 제어, watchdog, telemetry |
| D Vision & Environment Perception | ArUco/YOLO와 8방향 장애물 |
| E Radar Perception | 실제 IWR6843AOP target 추출 |
| F Sensor Fusion & Risk | WorldState와 SAFE/WARN/DANGER |
| G Escape Planning & Safety | 8방향 회피 및 최종 명령 차단 |
| H Odometry & Distance Control | encoder 이동량과 목표 거리 도달 STOP |

## PROJECT A 실행 경계

Perception은 영상 획득 직후 monotonic 시각을 기록하고 WorldState를 만들어 공유 스냅샷에 게시한다. 30 Hz 제어 루프는 최신 스냅샷을 읽고 stale/실패를 먼저 판정한 뒤 기존 RiskEvaluator → EscapePlanner → SafetyManager → UdpSender 순서로 실행한다. 기존 telemetry receiver는 별도 스레드다. 5 Hz 모니터 루프는 최신 제어 결과를 Laptop으로 보낸다.

공유 상태는 lock을 통해 복사한 스냅샷으로 읽고 쓴다. 오래되거나 유효하지 않은 WorldState, perception 예외, telemetry stale, camera 단절은 STOP이다. `GRISE_ENABLE_ESCAPE_MOTION=0`이 기본이며 실제 장치 확인 전 활성화하지 않는다. PROJECT H의 `target_distance_cm`는 ESP32의 encoder 기반 로컬 목표 종료에 사용되며 실거리 정확도는 하드웨어 검증 전이다.

SLAM, ROS2, 새 YOLO/PID, 실 radar parser, odometry, UI 확대는 PROJECT A 범위 밖이다.
