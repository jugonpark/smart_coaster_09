# GRISE 구현 현황과 2주 계획 (2026-09-16)

## 현재 확인한 범위

`README.md`, `SOURCE_MAPPING.md`, `CODEX_PROMPT.md`, `laptop/`, `raspberry_pi/`, `arduino/esp32_omni_controller.ino`를 읽었다. 이 작업공간은 Git 저장소가 아니며 실제 장치, YOLO 가중치, IWR6843AOP 입력을 포함하지 않는다. 아래의 통과 여부는 로컬 가짜 입력 및 루프백 통신만 뜻한다.

### 구현되어 있는 기능

- Laptop: USB 카메라 캡처, HTTP MJPEG, 로컬 미리보기, Pi UDP 상태 표시.
- Pi: MJPEG 수신, ArUco ID 0 추적, YOLO/Roboflow/stub 선택, UDP radar placeholder, 위험도와 hold, 8방향 회피 후보, 최종 안전판단, ESP32 명령·telemetry, Laptop 모니터 송신.
- ESP32 소스: 3WD 역기구학, encoder, PID, TB6612 출력, 300 ms command watchdog, Wi-Fi 손실·STOP 시 정지, telemetry 송신. 펌웨어는 이번 환경에서 빌드하거나 장치에서 실행하지 않았다.
- Pi 명령 형식 `{seq,t,vx,vy,w,status}`와 `RUN/SLOW/STOP`, cm/s 및 rad/s 단위 유지.

### Skeleton 또는 미완료

- 실제 IWR6843AOP parser와 센서 시간 동기화. UDP 8890은 가짜 입력 경계만 있다.
- Encoder 기반 3WD odometry 및 목표 거리 도달 정지. `target_distance_cm`는 현재 메타데이터다.
- ToF 등 근거리 센서 융합, camera calibration 및 obstacle 거리 정확도.
- PROJECT A에서 독립 30 Hz 제어 루프와 지연 계측을 추가했다. 실제 Raspberry Pi 부하에서 목표 주기와 지연 상한은 아직 측정하지 않았다.
- YOLO 모델 성능 검증. 가중치가 없으면 검출이 비활성화되며 실제 이동은 허용하지 않는다.

### 장치에서 확인할 사항

1. Laptop 카메라 인덱스, 1280x720@30 fps, Pi에서 MJPEG 디코딩과 네트워크 단절 후 재접속.
2. Laptop/Pi/ESP32 IP·서브넷·gateway. `10.182.7.110`을 Laptop IP로 가정하지 않는다.
3. ESP32 펌웨어 빌드, pin mapping, TB6612 STBY, encoder 방향·11 PPR×74.83, 휠 반경 2.9 cm, 로봇 반경 9.0 cm.
4. 바퀴를 띄운 상태의 STOP, RUN/SLOW, Wi-Fi 손실과 Pi 중단 후 300 ms 정지. 지면 주행은 그 뒤 제한된 공간에서 진행한다.
5. ESP32 telemetry 5 Hz와 Pi stale 판정, Laptop monitor 5 Hz. 실제 패킷 유실 및 시계 차이를 기록한다.
6. ArUco 8 cm 축·heading, 픽셀/cm 환산, 장애물 위치와 45 cm collision 경계.
7. 실제 radar 장착 후 거리, 접근속도 부호, TTC 및 15/35 cm·0.8/1.6 s 임계값 재조정.

### 발견한 위험과 이번 변경

- 4방향 장애물 상태를 8방향 회피에 사용해 대각선 장애물이 인접 방향까지 막았다. 8방향 상태로 분리했다.
- Camera 실패 시 STOP만 보내고 Laptop monitor를 갱신하지 않았다. `OFFLINE`/`STOP` 상태를 전송한다.
- 비정상 센서 좌표 또는 NaN 속도에 대한 최종 거부가 없었다. Pi SafetyManager에서 STOP한다.
- 모델 미탑재/stub이 빈 장애물 목록으로 해석될 수 있었다. 검출이 비활성화되면 모션을 금지한다.
- 비정상 telemetry JSON이 수신 스레드를 죽일 수 있었다. 형식·3개 휠 배열·유한값을 검사하고 다음 패킷을 계속 수신한다.
- telemetry 나이 0초를 monitor에서 OFFLINE으로 표시했다. 정상 ONLINE으로 표시한다.
- 남은 위험: ESP32 수신측은 `seq`/`t`의 신선도를 검사하지 않으며 송신자 인증도 없다. 신뢰된 격리 네트워크에서만 시험하고, 모션을 켜기 전에 재전송·역순 패킷 정책을 정한다.

## 2주 우선순위와 단계별 종료 기준

| 기간 | 단계 | 종료 기준 | 하드웨어 확인 |
|---|---|---|---|
| 1~2일 | Phase 1 Laptop→Pi 영상 | MJPEG loopback 통과; 실제 Pi에서 해상도·지연·재접속 계측 | USB 카메라와 실제 Wi-Fi |
| 2~3일 | Phase 2 Pi→ESP32 STOP | 패킷 형식 확인; 펌웨어 빌드; 정지 유지 | 모터를 띄워 STOP 및 Pi 종료 후 watchdog 확인 |
| 3~4일 | Phase 3 RUN/SLOW | 통신 형식 및 제한값 확인 | 제한된 공간에서 방향·속도·정지 확인 |
| 4~5일 | Phase 4 telemetry | malformed packet 후 수신 유지, stale→STOP | encoder count·RPM·wheel speed·RSSI 수신 |
| 5~6일 | Phase 5 monitoring | UDP 상태와 camera failure 표시 | Pi→Laptop 5 Hz 및 연결 단절 표시 |
| 6~8일 | Phase 6~8 ArUco/YOLO/sector | marker 손실→STOP, 장애물 8방향 테스트 | marker heading·거리 교정, 모델 정확도 |
| 8~10일 | Phase 9~10 fake radar/full simulation | SAFE/WARN/DANGER, blocked, stale, invalid 전부 STOP/RUN/SLOW 기대치 확인; 자동 모션 기본 OFF | 위험 시나리오 안전 시험 |
| 10~14일 | Phase 11 odometry 설계 | 기하·encoder 부호 실측 후 테스트와 위치 오차 기록; 검증 전 거리 종료 명령 금지 | 휠별 회전과 직선/횡/회전 주행 |

Phase 12 실제 IWR6843AOP 연결은 장치와 parser 사양이 준비된 뒤 별도 통합 단계로 둔다. 2주 안에 실제 radar까지 완성됐다고 간주하지 않는다.

## 현재 로컬 검증

`python -m unittest discover -s tests -v`는 가짜 카메라 MJPEG, UDP 명령 형식, telemetry, monitor, 안전 판단을 검사한다. 펌웨어 watchdog과 실제 주행은 이 테스트로 검증되지 않는다. `GRISE_ENABLE_ESCAPE_MOTION=0` 기본값은 유지한다.
