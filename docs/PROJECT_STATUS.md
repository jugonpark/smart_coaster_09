# GRISE 하위 프로젝트 상태

상태는 `NOT_STARTED`, `IN_PROGRESS`, `SOFTWARE_VERIFIED`, `HARDWARE_VERIFIED` 중 하나다. 로컬 가짜 입력 테스트는 장치 검증이 아니다.

| 프로젝트 | 상태 | 근거 및 남은 확인 |
|---|---|---|
| A Pi Core Controller | SOFTWARE_VERIFIED | 독립 30 Hz/5 Hz 루프, snapshot, stale/예외/telemetry/자동 이동 OFF를 로컬 테스트로 확인. 실제 Pi 지연과 ESP32 watchdog 미검증 |
| B Laptop Camera & Monitoring | SOFTWARE_VERIFIED | 카메라 실패·재연결/MJPEG/UDP 오류·만료/UI 종료를 가짜 장치와 루프백으로 검증. USB 카메라와 실제 네트워크 미검증 |
| C ESP32 Motion Controller | SOFTWARE_VERIFIED | Modular typed controller, safety/protocol/motion model tests, ESP32 Core 3.3.12 compile 통과. 실제 board upload·motor·watchdog 실기 미검증 |
| D Vision & Environment Perception | SOFTWARE_VERIFIED | VisionState·8방향·실패/신선도·좌표계 가짜 입력 테스트. 실제 카메라·ArUco·YOLO 실영상 미검증 |
| E Radar Perception | IN_PROGRESS | UDP fake/replay RadarState 경계 검증. 실제 firmware packet 명세·serial sample·parser가 없어 SAMPLE SERIAL FRAME REQUIRED |
| F Sensor Fusion & Risk | SOFTWARE_VERIFIED | D/E snapshot의 유효성·신선도, 다중 target·sector·TTC·EMA/hold를 fake 입력으로 검증. 실제 radar 거리·부호·지연 교정 없음 |
| G Escape Planning & Safety | SOFTWARE_VERIFIED | 8방향·UNKNOWN fallback·속도·Safety STOP 경계를 합성 입력으로 검증. 자동 모션 기본 OFF, 실기 미검증 |
| H Odometry & Distance Control | SOFTWARE_VERIFIED | IK/FK, derived count limit, braking, motion ID, completion gate, typed host protocol 및 ESP32 compile 검증. 실제 count/rev·15/30cm 정확도 미검증 |

기존 기준선: `python -B -m unittest discover -s tests -v` 11개 통과 (2026-09-16, PROJECT A 수정 전). 변경별 테스트와 하드웨어 잔여 항목은 이 문서에 갱신한다.

## PROJECT A 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| YOLO 속도에서 명령 주기를 분리 | `raspberry_pi/core/control_loop.py`, `main.py` | 인식 정지 상태에서 약 30 Hz 명령 및 5 Hz monitor 루프 테스트 | Pi 실제 부하에서 주기·jitter 측정 |
| 오래된 상태와 예외에서 STOP | `core/shared_state.py`, `core/state_machine.py`, `config.py`, `main.py` | stale, 인식 예외, 계산 중 실패, 계산 지연, invalid 상태 테스트 | 카메라 단절 및 추론 지연 실기 주입 |
| 실패 상태를 노트북에 표시 | `comm/monitor_sender.py` | 루프백 UDP camera/YOLO 실패 상태 테스트 | 실제 Laptop 화면 및 UDP 손실 확인 |

변경 후 전체 결과: `python -B -m unittest discover -s tests -v` 24개 통과 (2026-09-16). ESP32 펌웨어, 모터 및 radar 하드웨어 검증은 수행하지 않았다.

## PROJECT B 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| USB 카메라 초기 실패와 실행 중 손실 복구 | `laptop/camera_streamer.py`, `config.py` | fake 카메라 실패/재연결, 이전 프레임 폐기, HTTP 503→MJPEG 복구 | USB 장치 인덱스, 1280×720@30fps, 장치 분리/재연결 |
| 잘못된 Pi UDP와 오래된 표시 차단 | `laptop/monitor.py` | malformed JSON/NaN/INF/누락 필드, 정상 수신, 로컬 timeout 표시 | Pi와 Wi-Fi에서 5 Hz, 손실·지연 확인 |
| 창 닫기·예외 시 자원 정리 | `laptop/main.py` | 가짜 창 닫기에서 monitor/camera 정리 | 실제 OpenCV 창과 Ctrl+C 동작 |
| 설치·설정 설명 | `laptop/README.md` | 기본값과 환경변수 문서화 | 현장 Laptop IP·방화벽 확인 |

PROJECT B의 Python 로직만 SOFTWARE_VERIFIED이다. 실제 Laptop USB 카메라, Raspberry Pi와 Wi-Fi, JPEG 지연, 장시간 연결 안정성은 HARDWARE_VERIFIED 이전에 측정해야 한다. PROJECT A의 상태는 SOFTWARE_VERIFIED로 유지한다.

## PROJECT C 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| 필수 필드·속도·중복 seq 검증과 재시작 STOP 재동기화 | `arduino/esp32_omni_controller.ino` | ArduinoJson v7/ESP32 generic board 컴파일. Pi sender JSON 인터페이스 테스트 | 실제 UDP 역순·재시작·invalid packet 시험 |
| STOP/Wi-Fi 손실의 모터·PID 출력 정지 명확화 | `arduino/esp32_omni_controller.ino` | 소스 검토 및 컴파일 | 실제 PWM=0, 방향 핀 LOW, 300 ms watchdog 측정 |
| encoder 가정·제어 지연 계수·telemetry 상태 정리 | `arduino/esp32_omni_controller.ino`, `arduino/README.md` | 3휠 telemetry Pi 수신 및 malformed/NaN/INF/stale 테스트 | count 부호, PPR, RPM, 100 Hz jitter, 5 Hz telemetry 실측 |

PROJECT C 빌드: 설치된 Arduino CLI 1.5.1, Arduino-ESP32 3.3.11, ArduinoJson 7.4.3에서 `esp32:esp32:esp32`로 컴파일 성공 (2026-09-16). 이 FQBN은 generic ESP32 Dev Module이며 실제 보드 모델 확인, flash, 모터 주행은 수행하지 않았다. A와 B의 상태는 SOFTWARE_VERIFIED로 유지한다.

## PROJECT D 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| MJPEG 연결 상태와 수신 시각 명시 | `perception/network_camera.py` | fake 단절·재연결, 기존 MJPEG 루프백 | 실제 Laptop→Pi 지연·손실 측정 |
| 마커 유실과 검출기 실패를 빈 검출과 구분 | `perception/robot_tracker.py`, `object_detector.py`, `main.py` | 유실·빈 추론·예외 테스트 | ID 0, 8cm, 실제 마커 배치·가림 확인 |
| 로봇 상대좌표와 8방향 CLEAR/BLOCKED/UNKNOWN | `perception/vision_state.py`, `config.py`, `fusion/sensor_fusion.py`, `main.py` | 8방향·경계각·거리·stale·NaN/INF 테스트 | 로봇 반경, 원근 오차, 실제 물체 sector 확인 |
| 좌표·scale 가정 고정 | `docs/VISION_COORDINATES.md` | 코드·문서 대조 | 카메라/마커 현장 보정 |

PROJECT D는 카메라 없는 합성 입력과 전체 Python 테스트 및 구문 검사만으로 SOFTWARE_VERIFIED한다. 실장치 MJPEG, ArUco 및 모델 성능을 확인하기 전 HARDWARE_VERIFIED로 기록하지 않는다. A·B·C 상태는 SOFTWARE_VERIFIED로 유지한다.

## PROJECT E 변경 기록

| 이유 | 변경 파일 | 검증된 범위 | 미완료 입력 |
|---|---|---|---|
| fake UDP의 잘못된·오래된 데이터를 명시적으로 구분 | `perception/radar.py`, `config.py` | 정상/빈/다중 target, PARSE_ERROR, DATA_STALE, 수신 지속 테스트 | 실제 TI serial stream |
| offline fixture로 동일 RadarState 재생 | `perception/radar.py`, `tests/test_radar.py` | JSON Lines replay 및 오류 후 복구 테스트 | UART binary frame fixture와 명세 |
| 부호·각도·장착 가정 및 parser 경계 설명 | `docs/RADAR_INTERFACE.md` | fake 입력 계약 고정 | firmware/demo, CLI 설정, baud, raw 부호 측정 |

E는 **IN_PROGRESS / HARDWARE INPUT REQUIRED**다. `SerialFrameParser`는 의도적으로 구현되지 않았다. A·B·C·D는 SOFTWARE_VERIFIED 상태를 유지한다.

## PROJECT F 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| D/E snapshot을 개별 유효성·수신 시각과 함께 결합 | `fusion/sensor_fusion.py`, `main.py`, `config.py` | 정상 빈 target, offline/stale/parse error, vision UNKNOWN, skew·NaN 검사 | 실제 레이더 frame 시각·camera 지연·mount yaw 측정 |
| 최근접과 위험 우선순위를 구분, TTC·EMA·hold 경계 명시 | `decision/risk_evaluator.py`, `core/control_loop.py` | 다중 target·거리/TTC·ID 교체·hold·invalid STOP 검사 | 거리/속도/hold 임계값 현장 보정 |
| 선택 monitor 진단 및 계약 문서화 | `comm/monitor_sender.py`, `docs/FUSION_RISK.md`, `tests/test_fusion_risk.py` | 기존 packet 유지 및 새 진단 필드 | 실제 Laptop 표시·네트워크 확인 |

F는 fake/replay RadarState와 합성 VisionState를 사용한 SOFTWARE_VERIFIED다. 이는 실제 IWR6843AOP 기반 위험 검증을 뜻하지 않는다. E는 IN_PROGRESS / HARDWARE INPUT REQUIRED, A·B·C·D는 SOFTWARE_VERIFIED로 유지한다.

## PROJECT G 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| 정반대 우선의 결정적 8방향 fallback, UNKNOWN 제외, 계획 metadata | `planning/escape_planner.py`, `fusion/sensor_fusion.py` | 8방향·후보 순서·BLOCKED/UNKNOWN·대각선 속도 테스트 | 실제 이동 방향·장애물 sector·방향 oscillation |
| 최신 위험·계획·센서·telemetry 및 벡터 재검사 | `safety/safety_manager.py`, `core/control_loop.py` | auto OFF/ON, stale·invalid·NaN·속도 한계·STOP 테스트 | 실기 STOP 지연과 watchdog |
| 선택 계획 진단과 계약 | `comm/monitor_sender.py`, `docs/ESCAPE_SAFETY.md`, `tests/test_escape_safety.py` | 기존 packet 유지 | Laptop 표시·실기 주행 점검 |

G는 합성 WorldState/RiskState로 SOFTWARE_VERIFIED한다. G에서는 15/30cm가 계획 메타데이터였고, 현재 H가 encoder 기반 로컬 종료를 담당한다. E는 IN_PROGRESS / HARDWARE INPUT REQUIRED, A·B·C·D·F는 SOFTWARE_VERIFIED를 유지한다.

## PROJECT H 변경 기록

| 이유 | 변경 파일 | 소프트웨어 검증 | 하드웨어 잔여 |
|---|---|---|---|
| ESP32 100 Hz 로컬 거리 종료, 3WD forward kinematics 및 목표 투영 | `arduino/esp32_omni_controller.ino`, `arduino/odometry_math.h` | 방향·회전·count 변환·29/30cm 경계 compile-time assert, generic ESP32 컴파일 | 실제 count/rev, 부호, wheel radius, slip, overshoot |
| Pi 반복 goal ID, 완료 후 새 인식 스냅샷, boot/취소 재동기화 | `raspberry_pi/core/motion_goal.py`, `core/control_loop.py` | 목표 ID·완료·STOP·재부팅·취소 단위 테스트 | 실제 30 Hz UDP 손실·재부팅·watchdog 실기 |
| 기존 UDP 6개 필드 유지 및 선택 telemetry 수신 | `comm/udp_sender.py`, `comm/telemetry_receiver.py` | 기존 패킷 필드, 선택 필드, malformed/NaN 거부 테스트 | 실제 ESP32↔Pi 왕복 |
| 수식·보정·실기 측정 절차 | `docs/ODOMETRY_DISTANCE_CONTROL.md`, 관련 README/architecture 문서 | 코드·문서 대조 | 15/30cm 전 방향 반복 측정 |

H는 **SOFTWARE_VERIFIED**다. `python -B -m unittest discover -s tests -q`: 72개 통과 (2026-09-16). Arduino-ESP32 3.3.11, ArduinoJson 7.4.3, `esp32:esp32:esp32` 컴파일 성공: 947123 bytes flash (72%), 48636 bytes global RAM (14%). 실제 보드 flash, 모터 구동, 위치 정확도 측정은 수행하지 않았으며 HARDWARE_VERIFIED가 아니다. A·B·C·D·F·G 상태는 유지하고 E는 IN_PROGRESS / HARDWARE INPUT REQUIRED다.

## 2026-09-30 ESP32 controller redesign

| 변경 | 소프트웨어 근거 | 하드웨어 잔여 |
|---|---|---|
| 516-line composition sketch와 config/motor/protocol/motion/safety 모듈 | Arduino-ESP32 3.3.12 + ArduinoJson 7.4.3 compile, 949911 bytes flash, 49004 bytes global RAM | board upload, STBY/direction/PWM electrical behavior |
| typed session protocol, latest-wins mailbox, bounded UDP | duplicate/stale/session restart/heartbeat/latest motion/flood budget 계약 테스트 | 실제 UDP flood와 100Hz jitter 측정 |
| velocity/distance 상태 분리, braking, completion gates | pure model과 constexpr IK/FK/odometry/braking assertions | slip, count/rev, wheel radius, 15/30cm accuracy |
| immediate STOP/FAULT와 reset handshake | watchdog/wraparound/recovery/source-order 테스트 | 실제 STOP latency, Wi-Fi loss, controller restart |
| Windows/Pi typed migration와 legacy telemetry fallback | 전체 `pytest`: 152 passed + 33 subtests, py_compile, hidden Tk smoke | 실제 Laptop/Pi/ESP32 왕복과 장시간 안정성 |

이 결과는 C와 H의 `SOFTWARE_VERIFIED`를 갱신한다. 실제 모터는 실행하지 않았고
`HARDWARE_VERIFIED` 주장은 하지 않는다. Physical test 순서는
[ESP32 Motion Controller Reference](ESP32_MOTION_CONTROLLER.md)에 고정했다.
