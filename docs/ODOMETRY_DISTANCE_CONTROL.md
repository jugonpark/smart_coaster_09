# Odometry and Distance Control

## 책임과 명령 의미

`cmd_vel`은 최신 명령이 이전 desired body velocity를 교체한다. `cmd_move`는 ESP32 motion
state가 완료, STOP, fault, 취소 또는 다른 motion request까지 소유하는 지속 목표다. 두 명령은
같은 `NormalizedCommand` acceptance를 거치지만 실행 의미를 공유하지 않는다.

MOVE 예시:

```json
{"type":"cmd_move","session_id":38192014,"seq":102,
 "motion_id":20,"status":"RUN","vx":10.0,"vy":0.0,"w":0.0,
 "target_distance_cm":30.0}
```

같은 `motion_id`와 같은 방향·거리는 watchdog liveness만 갱신하고 goal origin을 초기화하지
않는다. 같은 ID에서 방향 또는 거리가 바뀌면 protocol fault와 즉시 정지다. 완료된 ID는
GOAL_REACHED에 남고 다른 ID만 새 목표를 시작한다.

## Encoder와 좌표계

Robot +X는 전방, +Y는 좌측, positive heading은 CCW다. Wheel angle은 M1/M2/M3
0/120/240°다. ISR은 A-phase RISING에서 B를 읽는 x1 count다. 초기 설정 898
counts/output-rev는 측정 출발값이며 실제 조립 상태별 확인이 필요하다.

Wheel displacement:

```text
distance_cm = delta_count * 2*pi*wheel_radius_cm / counts_per_rev
```

Forward kinematics:

```text
dx     = (d3 - d2) / sqrt(3)
dy     = (2*d1 - d2 - d3) / 3
dtheta = (d1 + d2 + d3) / (3*robot_radius)
```

각 tick은 midpoint heading으로 goal-start frame의 `odom_dx_cm`, `odom_dy_cm`,
`odom_dtheta_rad`를 적분한다.

```text
progress = odom_dx*ux + odom_dy*uy
lateral  = -odom_dx*uy + odom_dy*ux
```

Encoder jump 한계는 고정 2000 count가 아니다. 20cm/s wheel limit, circumference,
counts/rev, 실제 dt로 계산한 이론 count에 3배 margin을 적용한다.

## Braking과 완료

남은 거리에 따른 translation 상한:

```text
v_allowed = sqrt(2 * BODY_DECEL_CM_S2 * max(remaining_cm, 0))
```

requested cruise보다 braking bound가 작아지면 `DISTANCE_BRAKING`으로 전환한다. Overshoot가
발생하면 reverse correction을 하지 않고 즉시 PWM 0을 쓰며 `overshoot_cm`을 기록한다.

GOAL_REACHED에는 다음 조건이 모두 필요하다.

- `abs(remaining_cm) <= 0.5cm`
- 최대 measured wheel speed `<= 0.8cm/s`
- `abs(lateral_error_cm) <= 2.0cm`
- encoder와 odometry가 valid

5cm를 넘는 lateral deviation은 `PATH_DEVIATION` fault다.

## Software 검증과 실물 검증

constexpr math assertions와 Python model tests는 pure X/Y/rotation, IK/FK round trip,
progress/lateral 분리, derived encoder threshold, slew, braking, motion ID lifecycle,
completion gate, overshoot no-reverse를 확인한다. 이는 실제 slip, wheel diameter, encoder
polarity, chassis alignment를 확인하지 않는다.

실물 시험에서는 count/rev와 부호를 먼저 측정한 후 10cm/s wheel, +X, +Y, rotation,
15cm, 30cm 순으로 진행한다. 각 시험에서 target/measured/PWM/count, physical X/Y,
heading drift, lateral drift, overshoot를 기록한다.
