# PROJECT C: ESP32 Motion Controller

## 기존 경로와 타이밍

Pi는 UDP 8888에 `{seq,t,vx,vy,w,status}`를 약 30 Hz로 보낸다. `vx/vy`는 robot frame cm/s, `w`는 rad/s이며 `status`는 `RUN`, `SLOW`, `STOP`이다. ESP32는 기존 3WD 역기구학 → 100 Hz encoder/PID → TB6612 PWM 경로를 유지한다. 약 200 ms마다 UDP 8889로 telemetry를 보낸다. `STATUS` serial 출력의 `control overruns (>20ms)`는 제어 루프 지연을 관찰하기 위한 누적 계수다.

필수 필드가 없거나 숫자형이 아니거나 NaN/INF인 패킷, 잘못된 status, linear speed 50 cm/s 또는 angular speed 3 rad/s 초과, 지나치게 긴 패킷은 즉시 safe STOP으로 보낸다. 정상 `SLOW`는 기존 15 cm/s·1 rad/s 추가 상한을 적용한다. 동일하거나 작은 `seq`는 거부하며 STOP한다. Pi가 재시작하여 seq가 1로 돌아온 경우에는 마지막 유효 명령 이후 300 ms watchdog이 경과한 다음 새 `STOP` 패킷으로만 seq 기준을 재설정한다. Pi의 epoch `t`는 숫자·유한값 여부만 검증한다. ESP32의 `millis()`와 절대시각 비교는 하지 않는다. 인증이나 신뢰할 수 없는 네트워크에 대한 보호는 이번 범위 밖이다.

STOP 명령, invalid packet, Wi-Fi 손실, 300 ms watchdog, 비정상 wheel 수치는 `stopAllMotors()` 경로로 모이며 wheel target·PID 상태·PWM을 0으로 한다. PWM 0일 때 두 방향 핀은 LOW인 coast 정지다. Wi-Fi 재연결 후 이전 속도 명령을 재사용하지 않는다. Serial `MANUAL_PWM`은 기존 벤치 시험 기능으로 남아 있지만 Wi-Fi 손실과 잘못된 UDP 명령은 이 모드도 정지시킨다.

## Telemetry 계약

Pi의 기존 receiver와 같은 JSON을 유지한다: `type="telemetry"`, `seq`, `ms`(ESP32 uptime), `status`, `rssi`, 길이 3의 `counts`, `rpm`, `wheel_speed`, `target_speed` 배열. 속도 단위는 cm/s다. 추가 `mode`는 `NETWORK` 또는 `MANUAL_PWM`으로 기존 receiver가 무시한다. 비유한 wheel 숫자는 0으로 대체하고 safe STOP한다. `status`는 watchdog 정지 시 STOP을 보고한다.

## 핀과 측정 전 가정

| 항목 | GPIO 또는 값 |
|---|---|
| TB6612 공통 STBY | 27 |
| M1 IN1/IN2/PWM, ENC A/B | 19/18/25, 34/35 |
| M2 IN1/IN2/PWM, ENC A/B | 21/22/26, 16/17 |
| M3 IN1/IN2/PWM, ENC A/B | 23/13/14, 32/33 |
| Wheel normal angle (robot +X 기준 CCW) | M1 0°, M2 120°, M3 240° |
| Wheel radius / robot radius | 2.9 cm / 9.0 cm 초기값 |
| Encoder count 초기 가정 | `RAW_ENCODER_PPR=11`, `GEAR_RATIO=74.83`, `QUADRATURE_MULTIPLIER=1` (A rising만 계수) |
| PID 초기값 | Kp 3.0, Ki 8.0, Kd 0.05 |

`motorReversed`와 `encoderReversed`는 각 M1~M3에 개별 설정 가능하다. 현재 모두 false이며 실제 정방향 명령과 count 증가 방향은 미검증이다. M1~M3의 chassis 실제 위치·wheel 접선 방향, PPR/감속비, GPIO34/35의 외부 pull-up 유무를 측정해야 한다. TB6612 모듈 2개의 네 채널 중 쓰지 않는 한 채널의 IN/PWM을 LOW에 고정하는 배선도 확인한다. 소스는 그 채널의 핀을 할당하지 않는다. Wi-Fi SSID/password는 `CHANGE_ME` placeholder이며 실값을 저장소에 넣지 않는다. 고정 IP/gateway는 현장 네트워크와 Pi `GRISE_ESP32_IP`에 맞춰야 한다.

## 실제 장치 확인 순서

1. 실제 보드 FQBN을 확인하고 firmware를 빌드·flash한다. 빌드 성공은 flash나 주행 검증이 아니다.
2. 모터를 바닥에서 띄우고 STOP, 제한된 RUN/SLOW, 각 wheel 방향과 encoder count 부호를 확인한다.
3. wheel RPM/실측 속도/target 속도, 세 wheel 동시 제어, PID 잔류 출력과 STBY를 확인한다.
4. Pi 종료, UDP 손실, Wi-Fi 단절, 잘못된 패킷에서 정지 시간과 재연결 후 새 명령 필요 조건을 측정한다.
5. telemetry가 약 5 Hz로 Pi에 도착하고 Pi stale 판정이 작동하는지 확인한다.

PROJECT C는 odometry, 목표 거리 도달 STOP, PID 재튜닝을 포함하지 않는다.
# PROJECT H distance goals

Optional `motion_id` and `target_distance_cm` on RUN/SLOW activate encoder based distance control. The six original command fields remain required; packets without both optionals retain velocity mode. A valid STOP is required after ESP32 boot before a distance goal. See [Odometry & Distance Control](../docs/ODOMETRY_DISTANCE_CONTROL.md) for geometry, cancellation, telemetry, and calibration.
