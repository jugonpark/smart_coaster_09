# PROJECT H: Odometry & Distance Control

## Contract and ownership

The Pi remains the risk, planning, and safety authority. For a validated autonomous escape it repeats the same `motion_id` and `target_distance_cm` alongside the six existing UDP fields `{seq,t,vx,vy,w,status}`. `seq` identifies packet freshness; `motion_id` identifies one bounded action. A packet without both optional fields continues in velocity mode. The ESP32 uses its 100 Hz loop and encoder counts to stop locally. No Pi timing assumption is needed at the distance boundary.

The ESP32 boots stopped and requires a fresh valid STOP before any distance goal. STOP, invalid command, watchdog, Wi-Fi loss, and encoder fault cancel the goal and command PWM zero. A completed or canceled ID is retained and cannot restart from another RUN packet; a new ID is required. The Pi chooses a random positive starting ID each process, observes the ESP32 `boot_id`, sends STOP on a changed boot ID, and only then starts a new goal. This does not change the existing `seq` resynchronization policy. A completed goal causes Pi STOP until a later perception revision; risk and escape are then recomputed before a new ID is issued.

## Encoder assumption and wheel distance

The firmware ISR counts **A phase RISING** edges and reads B for direction, with optional per-motor sign inversion. Current nominal settings are raw encoder PPR 11, gear ratio 74.83, quadrature multiplier 1, so `COUNTS_PER_OUTPUT_REV = 823.13`. This is a software assumption, not a measured wheel value. The sign, PPR definition, effective quadrature factor, and gear ratio must be checked on each physical motor. Wheel displacement is `delta_count * 2π * WHEEL_RADIUS_CM / COUNTS_PER_OUTPUT_REV`, with nominal radius 2.9 cm. Absolute encoder snapshots are not reset by a new goal; successive deltas are checked for implausible jumps.

## Coordinate convention and forward kinematics

Robot +X is FRONT, +Y is LEFT, positive heading is counterclockwise. The existing inverse maps body `(vx,vy,w)` to wheels at 0°, 120°, 240° with `wheel_i = -sin(angle_i) vx + cos(angle_i) vy + ROBOT_RADIUS_CM*w`. Its exact inverse for wheel displacements `(d1,d2,d3)` is:

```text
dx = (d3 - d2) / sqrt(3)
dy = (2*d1 - d2 - d3) / 3
dtheta = (d1 + d2 + d3) / (3*ROBOT_RADIUS_CM)
```

The ESP32 integrates each body delta in the goal-start frame using midpoint heading. The initial goal direction is unit `(ux,uy)` from the first command. `goal_progress_cm = action_dx_cm*ux + action_dy_cm*uy`; `lateral_error_cm = -action_dx_cm*uy + action_dy_cm*ux`. Thus side drift and total wheel or Euclidean travel do not count toward the target. Project H supports translation goals with `w≈0`; it reports rotation drift but does not correct heading. Initial stop threshold is `target_distance_cm - 0.5 cm`; physical overshoot is separate and unmeasured.

## State and telemetry

Local modes are `IDLE`, `VELOCITY_CONTROL`, `DISTANCE_CONTROL`, `GOAL_REACHED`, `STOPPED`. The optional telemetry extension contains `boot_id`, `motion_id`, `goal_active`, `goal_reached`, `target_distance_cm`, `goal_progress_cm`, `remaining_distance_cm`, `action_dx_cm`, `action_dy_cm`, `action_dtheta_rad`, and `lateral_error_cm`. Existing telemetry fields remain. The Pi rejects nonfinite or malformed optional values. The monitor and other old consumers may ignore them.

## Hardware verification procedure

With wheels raised, measure one output revolution on each motor and record count magnitude and sign in both directions. Confirm ISR edge mode, actual counts/rev, and wheel circumference before ground tests. Then place the robot in a clear, guarded area and measure physical displacement for FRONT/BACK/LEFT/RIGHT/diagonal 30 cm, WARN 15 cm, DANGER 30 cm. Record target, projected encoder progress, physical X/Y, heading drift, overshoot, lateral drift, floor material, and repeated trials. Test repeated packets for a completed ID, new ID, STOP mid-goal, Wi-Fi loss, Pi crash, and ESP32 restart. Calibrate `COUNTS_PER_OUTPUT_REV` or effective wheel radius only from those measurements. Until then PROJECT H is software verified only, not hardware verified.
