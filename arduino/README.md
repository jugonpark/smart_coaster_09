# ESP32 Motion Controller

펌웨어는 `esp32_omni_controller/` 아래의 고정 구조체와 함수 기반 모듈로 구성된다.

| 파일 | 책임 |
|---|---|
| `esp32_omni_controller.ino` | 안전한 setup, cooperative scheduler, bounded Serial/Wi-Fi/UDP service |
| `robot_config.h` | 핀, 단위, geometry, limit, timing, PID/FF |
| `motor_encoder.*` | TB6612 출력, 최소 ISR, count snapshot, 속도 계산 |
| `network_protocol.*` | typed/legacy decode, normalized acceptance, latest-wins mailbox |
| `kinematics_motion.*` | 상태, IK/FK, slew, odometry, distance braking, PID |
| `safety_telemetry.*` | watchdog, fault/reset, 즉시 정지, typed telemetry와 migration aliases |
| `odometry_math.h` | hardware-independent constexpr 수학 |

상세 계약은 [ESP32 Motion Controller](../docs/ESP32_MOTION_CONTROLLER.md)와
[Odometry & Distance Control](../docs/ODOMETRY_DISTANCE_CONTROL.md)에 있다.

## 빌드와 로컬 시크릿

Arduino-ESP32 Core 3.3.12와 ArduinoJson 7.4.3 기준:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 arduino/esp32_omni_controller
```

`secrets.example.h`를 `secrets.h`로 복사하고 로컬 Wi-Fi 값을 입력한다.
`secrets.h`는 Git에서 제외된다. 현재 연결은 DHCP만 사용한다.

## 고정 하드웨어 계약

| 장치 | IN1 | IN2 | PWM | ENC_A | ENC_B |
|---|---:|---:|---:|---:|---:|
| M1 | 19 | 18 | 25 | 34 | 35 |
| M2 | 21 | 22 | 26 | 16 | 17 |
| M3 | 23 | 13 | 14 | 32 | 33 |
| STBY | 27 | | | | |

M1 GPIO34/35에는 internal pull-up이 없다. 외부 pull-up과 실제 배선을 모터 시험 전에 확인한다.
`motorReversed[]`와 `encoderReversed[]`는 독립 보정값이며 현재 모두 false다.

초기 software 설정:

- wheel radius 2.9cm, robot radius 9.0cm, wheel angle 0/120/240°
- A-phase RISING x1, measured starting value 898 counts/output-rev
- body 15cm/s, wheel 20cm/s, angular 1.0rad/s, SLOW 5cm/s
- PID Kp 3.0, Ki 0.6, Kd 0.0
- control 100Hz, telemetry 5Hz, command watchdog 300ms
- UDP command 8888, telemetry 8889

count/rev, motor/encoder polarity, wheel radius, PID/FF는 실물 확인 전 보정 확정값이 아니다.

## Serial 명령

`HELP`, `STATUS`, `PIN`, `ENC`, `ZERO`, `AUTO`, `MANUAL`, `STOP`,
`M1/M2/M3`, `ALL`, `VEL`, `MOVE`를 지원한다. MANUAL_TEST는 Wi-Fi와 독립적이지만
3초 동안 새 수동 명령이 없으면 즉시 정지한다. Serial `STOP`은 항상 작동한다.

실물 연결 직후에는 `STATUS`로 DHCP IP와 UDP 상태만 확인한다. 모터 명령은 바퀴를 띄우고
물리 시험 절차를 준비한 뒤 수행한다.

## 안전 경계

STOP, fault, Wi-Fi loss, watchdog, invalid packet은 mailbox와 slew limiter를 건너뛰고
가장 먼저 PWM 0을 쓴다. 방향 전환은 PWM 0, deadtime, 방향 핀, duty 순이다.
펌웨어 빌드와 host 테스트는 `SOFTWARE_VERIFIED` 근거이며 flash, 배선, 회전, 거리 정확도를
검증하지 않는다.
