# PROJECT E Radar interface

## 근거와 미구현 경계

대상은 TI IWR6843AOP-EVM, xWR68xx, mmWave SDK 03.06 계열이다. 현재 작업공간에는 사용 중인 firmware/demo의 정확한 이름·설정, UART packet 명세 또는 실제 serial frame이 없다. `SerialFrameParser.feed()`는 **SAMPLE SERIAL FRAME REQUIRED** 경계로 남긴다. 현재 `serial` backend는 OFFLINE을 반환하며 TI 바이너리를 해석하지 않는다. 실시간 serial transport도 아직 구현하지 않았다. 이 조건에서 E 전체를 SOFTWARE_VERIFIED로 간주하지 않는다.

## Backend와 입력

`GRISE_RADAR_BACKEND`는 `disabled`, `udp_fake`, `replay`, `serial`을 선택한다. 기본 빈 값은 기존 `GRISE_RADAR_ENABLED=1`일 때 `udp_fake`, 그 외 `disabled`와 같다. `udp_fake`는 기존 UDP 8890을 유지하며 nonblocking 수신이다. `replay`는 `GRISE_RADAR_REPLAY_PATH`의 JSON Lines를 한 줄씩 읽어 같은 정규화 경로를 거친다. 파일을 저장해 회귀 fixture로 사용할 수 있다. 이 JSON은 **이미 GRISE 부호·각도로 정규화한 가짜 입력**이며 TI raw packet이 아니다.

기존 단일 target 입력은 그대로 쓸 수 있다:

```json
{"distance_cm":62.4,"angle_deg":-8.3,"approach_speed_cm_s":41.2}
```

정상적인 빈 frame은 `{"targets":[]}`이고 다중 target은 `{"frame_id":7,"targets":[{...},{...}]}`이다. `frame_id`와 각 target의 `target_id`는 선택 사항이다. `targets`는 입력 순서대로 보존하며 `nearest_target`/기존 `target`은 단순 최단 거리다. 위험도 순위가 아니다.

## 데이터 계약

`RadarTarget`의 `distance_cm`는 0 이상, `angle_deg`는 레이더 정면 0°, 왼쪽 양수·오른쪽 음수이며 `[-180,180)`으로 정규화한다. 이는 **가짜 입력의 GRISE 규약**이다. 실제 TI 출력의 azimuth 부호는 확인 전이다. `approach_speed_cm_s > 0`은 로봇 쪽으로 접근한다. `radial_velocity_cm_s`는 GRISE 정규화 값으로 양수일 때 멀어지므로 fake 입력에서는 `-approach_speed_cm_s`다. `approaching`은 approach speed가 양수일 때 참이다. 실제 raw Doppler 부호 변환은 firmware·샘플 확인 전까지 정의하지 않는다. `timestamp`는 Pi monotonic 수신 시각이며 외부 packet 시각을 로컬 시각으로 오인하지 않는다.

`RadarState`는 기존 `connected`, `target`, `age_s`, `source` 순서를 유지하며 `valid`, `timestamp`, `frame_id`, 전체 `targets`, `status`, `error`를 제공한다. 상태는 `OFFLINE`, `NO_TARGET`, `TARGET_DETECTED`, `DATA_STALE`, `PARSE_ERROR`다. `NO_TARGET`은 연결·데이터가 유효한 정상 빈 frame이다. 오류·만료 시 이전 target을 현재 target으로 반환하지 않는다. fake UDP 만료는 `RADAR_STALE_S=0.35`초 후 `DATA_STALE`, `connected=false`, `valid=false`가 된다. 파싱 실패는 `connected=true`, `valid=false`, `PARSE_ERROR`다. 수신 오류 시 소켓 재연결을 시도한다. `RadarReceiver.read()`는 한 번에 최대 32 packet을 nonblocking으로 읽는다.

`RADAR_MAX_DISTANCE_CM=1000`, `RADAR_MAX_RADIAL_SPEED_CM_S=500`, target 최대 128개는 **임시 fake 입력 보호 한계**이며 센서 성능 명세가 아니다. NaN/INF, 음수 거리, 범위 밖 각도·속도, 잘못된 ID는 frame 전체를 거부한다. `RADAR_MOUNT_YAW_DEG=0`은 장착 가정으로만 보관하고 E parser에는 적용하지 않는다. 레이더와 로봇/카메라 좌표 결합은 F의 후속 작업이다.

## 실제 연결을 위한 자료와 확인

실제 E 완료에는 사용 firmware/demo와 SDK build, control/data UART 경로 및 baud, CLI 설정 파일, 공식 출력 packet 명세, 원본 serial byte frame fixture가 필요하다. 정면/좌측/우측 표적과 접근·이탈 동작을 함께 기록해 거리·각도·raw radial velocity의 부호를 판별해야 한다. Pi에서 장치 인식, frame rate, 빈·다중 target, malformed/partial frame, 케이블 분리·재연결, stale 시간, CPU 부하와 장시간 안정성을 확인한다. 이 증거로 parser와 serial transport를 구현·회귀 검증하기 전에는 HARDWARE_VERIFIED도 아니다.
