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
- ESP32: 개발/테스트 시 DHCP, UDP 8888, command watchdog 300ms
- ESP32 geometry: wheel radius 2.9cm, robot radius 9.0cm, wheel angles 0/120/240 deg
- Encoder: 11 PPR x 74.83 reduction, A rising edge
- PID initial: Kp 2.6, Ki 1.3, Kd 0.0, control loop 100Hz
- Motor pins and TB6612 STBY are preserved from the provided sketch.

## 새로 추가한 값/인터페이스

- ESP32 telemetry UDP 8889 @ ~5Hz (200ms)
- Pi monitor UDP 9001 @ 5Hz
- Radar placeholder UDP 8890
- MVP escape distance: WARN 15cm / DANGER 30cm

PROJECT H는 자동 회피 목표의 거리 종료를 ESP32에서 수행한다. 엔코더 배율과 실제 15/30cm 이동 오차는 실기 보정이 필요하다. 세부 계약은 [Odometry & Distance Control](docs/ODOMETRY_DISTANCE_CONTROL.md)에 있다.

## Windows Laptop Central Controller

Raspberry Pi 없이 Windows 노트북에서 ESP32의 기존 velocity-control 경로를 수동 시험할 수 있다. 바퀴를 공중에 띄우고 firmware를 업로드한 다음 Arduino Serial Monitor에서 다음 명령을 실행한다.

```text
AUTO
STREAM ON
```

Serial 상태에서 `MODE = NETWORK`, Wi-Fi 연결 및 UDP listener 활성화를 확인한다. 이후 저장소 루트에서 실행한다.

```powershell
python tools/laptop_central_controller.py
```

Serial Monitor에 출력된 현재 DHCP IP를 지정할 수도 있다.

```powershell
python tools/laptop_central_controller.py --ip 10.232.69.180
```

위 주소는 예시이며 실제로는 Serial Monitor에 표시된 DHCP IP를 사용한다.

GUI의 `연결`은 먼저 STOP을 20Hz로 계속 보내 sequence epoch를 재동기화한다. 정상 telemetry를 받은 뒤에만 W/S/A/D 이동과 Q/E 회전을 허용한다. 키를 놓으면 STOP하며, Space는 일반 정지, ESC는 latch되는 비상 정지다. 비상 정지는 `비상 정지 해제` 버튼으로만 해제되고 항상 정지 상태로 돌아온다. Telemetry가 1초간 없으면 `텔레메트리 끊김`으로 전환해 계속 STOP을 보내며, telemetry가 복구돼도 이전 키 입력을 자동 재개하지 않는다.

Windows Firewall에서 Python의 UDP telemetry port 8889 수신을 허용해야 할 수 있다. `tools/esp32_udp_test.py`와 central controller는 모두 UDP 8889를 사용하므로 동시에 실행하지 않는다.

연결 직후 이벤트 로그의 `UDP 준비`에서 노트북 송신 IP와 ESP32 대상 IP를 확인한다. 현재 ESP32 설정은 `/24`(`255.255.255.0`)이므로 두 주소의 앞 세 옥텟이 다르면 같은 Wi-Fi 대역에 연결하고 IP/gateway 설정을 다시 확인한다. `ping` 성공만으로 대상이 ESP32라고 판단하지 않는다. 다른 망의 동일 IP 장비가 응답할 수 있고 ICMP 성공은 UDP 8888/8889 도달을 보장하지 않는다.

텔레메트리가 없으면 GUI 로그의 STOP 송신·UDP 수신·정상·거부 건수와 Arduino Serial의 `UDP rx/accepted/rejected`를 함께 본다. ESP32 수신 누계가 0이면 네트워크 경로 문제이고, 수신은 증가하지만 거부가 증가하면 바로 위의 `[UDP RX] rejected reason=...`에서 JSON/필드/sequence 사유를 확인한다.

### 개발/테스트 네트워크

- ESP32는 기본적으로 DHCP를 사용한다.
- firmware 업로드 후 Serial Monitor에서 `[WiFi] IP=...` 또는 `STATUS`의 `WiFi = CONNECTED IP=...`를 확인한다.
- 확인한 IP를 Windows 중앙 관제 GUI의 `ESP32 IP`에 입력한다.
- 노트북과 ESP32가 같은 `/24` 대역인지 확인한 뒤 `연결` 버튼만 눌러 STOP 및 telemetry 경로를 먼저 검증한다.

최종 시연용 전용 공유기에서 고정 주소가 필요하면 firmware의 `USE_STATIC_IP`를 `true`로 바꾸고 `LOCAL_IP`, `GATEWAY`, `SUBNET`을 해당 공유기 대역에 맞춘다. 기존 `10.182.7.50` 설정을 다른 네트워크에서 그대로 사용하면 안 된다.
- Automatic escape motion is **disabled by default** until integration tests pass.

## 중요

Wi-Fi SSID/password는 보안상 산출물에서 `CHANGE_ME`로 바꿨습니다. 실제 환경 값은 직접 입력하세요.
`10.182.7.110`은 기존 ESP32 코드의 gateway 값이며 노트북 IP라고 확정할 수 없으므로 실제 네트워크에서 확인해야 합니다.
