# PROJECT D 영상 좌표와 상태

## 좌표계

- 영상 원점은 왼쪽 위, `x`는 오른쪽, `y`는 아래쪽이다. Laptop 송출은 좌우 반전하지 않는다.
- 월드 원점은 영상 왼쪽 아래로 환산한다. `+X`는 오른쪽, `+Y`는 위쪽이며 단위는 cm이다. `WorldFrame.to_world()`가 `(px_x / px_per_cm, (frame_height - px_y) / px_per_cm)`를 적용한다.
- 로봇 원점은 마커 중심이다. 로봇 `+x`는 전방, `+y`는 왼쪽이다. `world_to_robot()`는 월드 위치 차이를 heading의 음의 각도만큼 회전한다.
- Heading 0은 월드 `+X`를 향하고 양의 회전은 반시계 방향이다. 현재 ArUco heading은 검출된 `top-left → top-right` edge를 전방으로 간주한다. `MARKER_HEADING_OFFSET_DEG=0`이다. 실제 마커 부착 방향과 로봇 전방 일치는 하드웨어 검증 대상이다.

## 거리 보정의 전제

초기 `DEFAULT_PX_PER_CM=8.0`이다. ID 0 마커의 물리적 한 변 8cm와 영상 측정 길이로 `WorldFrame`의 단일 `px_per_cm`을 갱신한다. EMA alpha 0.5는 포즈에 적용된다. 이 비율은 같은 영상 평면에 있는 물체와 마커에만 근사적으로 유효하다. 카메라 기울기, 물체 높이, 렌즈 왜곡, 원근 변형에 대한 보정은 없다. 따라서 계산된 cm와 45cm sector 경계는 현장 측정값으로 확정할 수 없다. `WorldFrame`이 향후 보정 결과를 받는 경계이며, 이번 단계에 보정 도구는 포함하지 않았다.

## 8방향과 유효성

Bearing은 로봇 `+x`에서 반시계 양수이다. 각 sector는 45° 폭이고 정확히 ±22.5°, ±67.5° 같은 경계는 반시계 쪽 sector에 포함한다. 순서는 `FRONT, FRONT_LEFT, LEFT, BACK_LEFT, BACK, BACK_RIGHT, RIGHT, FRONT_RIGHT`다. 따라서 `FRONT_RIGHT`는 음의 bearing이다.

장애물 후보는 `obstacle, box, bottle, chair`이며 `person`을 포함한 다른 label은 Vision object로 남되 D의 sector를 차단하지 않는다. 초기 판정은 `(중심 거리 - 검출 bbox 반경 - ROBOT_RADIUS_CM) <= COLLISION_CHECK_DISTANCE_CM`이다. 반경은 각각 bbox 긴 변의 절반, 초기 로봇 반경 9.0cm, threshold 45cm이다. 이 원형 근사는 실제 chassis 외곽이나 물체 footprint를 대체하지 않는다.

`VisionState`는 `camera_ok`, frame 수신 `timestamp`, `robot`(detected, x/y, heading, timestamp), `detector_available`, 상대좌표와 검출 시각이 있는 `objects`, 8개 `sectors`를 제공한다. 정상적인 빈 검출은 `CLEAR`이고 카메라·마커·검출기 실패, 오래된 영상·포즈·검출, 잘못된 좌표는 `UNKNOWN`이다. 마커 유실은 현재 포즈를 비우며 이전 포즈를 새 포즈로 게시하지 않는다. 판정은 monotonic clock과 `WORLD_STALE_S=0.35`를 사용한다. 정확한 다중 센서 시각 동기화는 여기서 수행하지 않는다.

Pi `WorldState.vision`은 선택 필드이며 기존 명령·모니터 패킷 형식을 바꾸지 않는다. `UNKNOWN`은 기존 차단 플래그에 차단으로 사상된다. 일부 sector만 UNKNOWN이면 G는 그 방향을 제외하고 다른 CLEAR 방향을 검토한다. 카메라·포즈·검출기 자체가 무효이면 `sensor_valid=false`이고 상위 안전 게이트가 정지한다. D는 위험도·회피 방향을 계산하지 않는다.

## 실기 검증

외부 카메라→Pi MJPEG 수신, ID 0/8cm 인식, 전방 edge와 좌표·회전 방향, 실제 박스·의자·사람 검출, 45cm 거리 오차, 원근 왜곡, 영상 지연과 YOLO 처리량, 마커 가림·복구를 실제 장치에서 확인해야 한다.
