# ESP32 Motion Controller Reference

## Runtime architecture

```text
UDP / Serial
  -> bounded decode
  -> NormalizedCommand
  -> session + seq + absolute-limit acceptance
     -> STOP/fault: immediate motor PWM zero
     -> heartbeat: liveness only
     -> cmd_vel/cmd_move: latest-wins mailbox
  -> 100Hz control tick
     -> encoder snapshot/validation
     -> watchdog and motion state
     -> slew/braking -> IK -> wheel normalization
     -> preserved wheel PID -> TB6612
  -> 5Hz immutable telemetry snapshot
```

`loop()` checks the control deadline before and after UDP. One UDP service call handles at most
4 packets and 1000 microseconds. Packet size is at most 512 bytes. JSON, Serial output, telemetry
serialization, and Wi-Fi reconnect are outside the control tick.

## Command protocol

Command port is UDP 8888. Every controller process creates one random nonzero 32-bit
`session_id`; `seq` strictly increases within it. Decode completes before acceptance classifies
duplicate/stale packets. Duplicate/stale commands are ignored. A session change during movement
immediately stops and needs a later STOP or heartbeat before retrying motion.

```json
{"type":"cmd_vel","session_id":11,"seq":1,
 "status":"RUN","vx":10.0,"vy":0.0,"w":0.0}
```

```json
{"type":"cmd_move","session_id":11,"seq":2,
 "motion_id":5,"status":"SLOW","vx":5.0,"vy":0.0,"w":0.0,
 "target_distance_cm":15.0}
```

```json
{"type":"stop","session_id":11,"seq":3}
{"type":"heartbeat","session_id":11,"seq":4}
{"type":"reset_fault","session_id":11,"seq":5}
```

`ENABLE_LEGACY_PROTOCOL=true` is a temporary migration adapter. Legacy
`{seq,t,vx,vy,w,status}` becomes the same `NormalizedCommand`; it has no motor path. Partially
supplied legacy distance fields are invalid.

## Mailbox and states

The internal command transport is one latest-wins slot plus a local revision, with no FIFO.
Heartbeats never occupy it. MOVE leaves the mailbox when consumed and becomes persistent motion
state; VEL remains replaceable desired velocity.

States:

```text
STOPPED
VELOCITY
DISTANCE_ACTIVE
DISTANCE_BRAKING
GOAL_REACHED
FAULT
```

Ordinary reversal uses body and wheel slew through zero. STOP and FAULT bypass both mailbox and
slew and call the physical PWM-zero write first.

## Faults and recovery

| Fault | Recovery |
|---|---|
| WIFI_LOSS, CMD_TIMEOUT, BAD_PACKET | condition clear plus fresh STOP/heartbeat handshake |
| SPEED_LIMIT, ENCODER_JUMP, ENCODER_INVALID | `reset_fault`, then fresh STOP/heartbeat |
| ODOMETRY_INVALID, PATH_DEVIATION, CONTROL_TIMING | `reset_fault`, then fresh STOP/heartbeat |

NETWORK moving states use a 300ms accepted command/heartbeat watchdog. MANUAL_TEST ignores Wi-Fi
and uses a 3000ms serial command watchdog. Wi-Fi loss in NETWORK stops. Serial STOP always works.

## Telemetry

Telemetry port is UDP 8889 at 5Hz and targets the accepted controller address. Typed fields:

- identity: `type`, `boot_id`, `session_id`, `last_seq`, `uptime_ms`
- controller: `mode`, `state`, `fault`, `command_age_ms`
- command: `cmd_vx`, `cmd_vy`, `cmd_w`
- wheel: `wheel_target`, `wheel_speed`, `wheel_pwm`, `encoder_count`
- goal: `motion_id`, `goal_target_cm`, `goal_progress_cm`, `remaining_cm`,
  `lateral_error_cm`, `overshoot_cm`
- odometry: `odom_dx_cm`, `odom_dy_cm`, `odom_dtheta_rad`
- diagnostics: `wifi_rssi`, `control_overruns`

Migration aliases `seq`, `status`, `counts`, `rpm`, `target_speed`, `pwm` remain. Inapplicable
goal values are null. Hosts prefer typed fields and fall back to aliases. Missing legacy PWM is
shown as `--` and written as an empty CSV field.

## Configuration and secrets

`robot_config.h` is the only pin and controller configuration source. `secrets.h` is local and
ignored; copy `secrets.example.h`. DHCP is used. Do not reuse an address from another subnet.

Initial software limits are body 15cm/s, wheel 20cm/s, angular 1.0rad/s, SLOW 5cm/s. The wheel
PID core preserves Kp 3.0, Ki 0.6, Kd 0.0, array state, P/I/D order, output saturation, and
anti-windup. Tuning is not part of this redesign.

## Verification boundary

Current automated gate:

- Python regression and protocol/motion/safety models
- Python syntax compilation
- hidden Tk GUI smoke
- Arduino CLI `esp32:esp32:esp32`, Core 3.3.12, ArduinoJson 7.4.3
- `git diff --check`

These establish `SOFTWARE_VERIFIED`. They do not establish board upload, wiring, motor direction,
encoder polarity/counts, 100Hz timing under real Wi-Fi load, watchdog latency, trajectory, or
15/30cm accuracy.

## Physical test order

1. Upload and open Serial Monitor; do not send RUN.
2. Confirm boot stopped, DHCP IP, `UDP=ON`, and `STATUS`.
3. Confirm laptop and ESP32 share the intended subnet; connect GUI and verify typed STOP/telemetry.
4. Raise wheels. Test M1, M2, M3 positive/negative and encoder sign independently.
5. Measure one output revolution count for every wheel and direction.
6. Test each wheel at 10cm/s, then all wheels.
7. Test robot +X, +Y, and rotation.
8. Test 15cm goal, then 30cm goal.
9. Record target, measured speed, PWM, count, physical distance, lateral/heading drift, overshoot,
   network loss, controller restart, and STOP latency.

Only after these pass should the corresponding item move to `HARDWARE_VERIFIED`.
