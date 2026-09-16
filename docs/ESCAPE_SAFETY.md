# PROJECT G: Escape Planning & Safety

## 입력과 경계

Planner는 F의 `WorldState`와 `RiskState`를 소비한다. SAFE·INVALID와 센서 또는 WorldState가 무효·만료된 경우 실행 가능한 계획을 만들지 않는다. 회피 중 방향은 매 control tick의 최신 snapshot에서 다시 계산한다. 경로 smoothing이나 방향 hold는 이번 단계에 없다. 실제 주행 시 oscillation을 확인해야 한다.

## 8방향 정책

로봇 `+x`는 FRONT, `+y`는 LEFT이다. 순서는 `FRONT, FRONT_LEFT, LEFT, BACK_LEFT, BACK, BACK_RIGHT, RIGHT, FRONT_RIGHT`이며 threat의 정반대 index를 우선한다. 후보 offset은 정반대 기준 `0, -1, +1, -2, +2, -3, +3, 4`이다. 따라서 FRONT threat의 순서는 `BACK, BACK_LEFT, BACK_RIGHT, LEFT, RIGHT, FRONT_LEFT, FRONT_RIGHT, FRONT`다. 동률에서 왼쪽 방향을 먼저 보는 고정 정책이며 무작위 선택하지 않는다.

`CLEAR`만 이동 후보가 된다. `BLOCKED`와 `UNKNOWN`은 해당 방향에서 제외한다. 일부 sector가 UNKNOWN이어도 다른 CLEAR 방향을 찾을 수 있다. 모든 방향이 막혔거나 UNKNOWN이면 계획은 무효이며 STOP이다. 카메라·검출기·로봇 포즈·Radar 자체가 무효인 경우는 전체 계획을 무효로 한다.

## 계획과 속도

`EscapePlan`은 기존 `vx, vy, w, status, direction, target_distance_cm, reason`을 유지하며 `valid`, 생성 시각, 위험도, threat 방향, 목표 속도, 후보·제외 방향을 추가한다. WARN은 SLOW·15cm/s·15cm, DANGER는 RUN·25cm/s·30cm이다. 8방향은 단위 벡터에 목표 속도를 곱해 변환한다. 대각선의 합성 속도도 목표 속도와 같다. 기본 회전속도 `w=0`으로 heading을 유지한다.

`target_distance_cm`는 PROJECT H에서 검증된 자동 회피 명령에 한해 ESP32 로컬 거리 목표로 전달된다. 제어 방식과 검증 경계는 [Odometry & Distance Control](ODOMETRY_DISTANCE_CONTROL.md)에 있다.

## 최종 안전 게이트

SafetyManager가 최종 권한을 가진다. 카메라·마커·Vision·Radar 무효, 만료된 WorldState·계획, INVALID 위험, ESP32 telemetry 만료, 무효 계획, 선택 sector의 BLOCKED/UNKNOWN, 비정상 수치·속도·상태·벡터 불일치 시 STOP을 반환한다. `GRISE_ENABLE_ESCAPE_MOTION=0`이 기본이므로 계획이 정상이어도 전송 명령은 STOP이다. 이 스위치를 켜더라도 모든 검사가 통과해야 RUN/SLOW가 허용된다. Pi→ESP32 JSON 형식은 유지한다.

Laptop monitor에는 계획 유효성·방향·속도·거리 메타데이터·후보·제외 방향·정지 이유를 선택 필드로 추가한다. 기존 필수 필드는 유지한다.

## 실기 검증

실제 로봇에서 8방향과 대각선 이동, WARN/DANGER 속도, STOP, 장애물 fallback, 자동 이동 스위치, 센서·telemetry·Wi-Fi 손실에 따른 정지, 회피 중 방향 oscillation을 확인해야 한다. 정확한 15/30cm 이동은 H 이후 검증한다.
