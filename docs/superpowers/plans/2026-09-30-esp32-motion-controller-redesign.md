# ESP32 Motion Controller Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the monolithic ESP32 firmware with a safe, typed, testable low-level motion controller while preserving the verified wheel-speed PID core.

**Architecture:** A thin Arduino sketch runs a cooperative scheduler around focused function-based modules. UDP and Serial commands normalize into one acceptance layer and latest-wins mailbox; STOP and faults bypass the mailbox, while the 100 Hz control path owns encoder validation, motion state, IK, PID, odometry, and motor output.

**Tech Stack:** Arduino-ESP32 Core 3.3.12, ArduinoJson 7.4.3, C++17-compatible Arduino code, Python 3 standard library, pytest/unittest.

**Spec:** `docs/superpowers/specs/2026-09-30-esp32-motion-controller-redesign.md`

## Global Constraints

- Preserve the current PID gains, P/I/D calculation order, `pidIntegral[]`, `pidPreviousError[]`, `targetWheelSpeed[]`, `measuredWheelSpeed[]`, saturation, and anti-windup concept.
- Use the GitHub `06693c1` pin map: M1 `19/18/25/34/35`, M2 `21/22/26/16/17`, M3 `23/13/14/32/33`, STBY `27`.
- Keep command UDP `8888`, telemetry UDP `8889`, command watchdog `300 ms`, control period `10 ms`, and telemetry period `200 ms`.
- Initial limits are body linear `15 cm/s`, wheel `20 cm/s`, angular `1.0 rad/s`, and SLOW linear `5 cm/s`.
- Use centimeters, cm/s, rad/s, radians, counts, seconds for control math, and explicitly named ms/us scheduler values.
- Use A-phase RISING with B-phase direction and document output-shaft counts/rev as measured but physically unverified.
- No Wi-Fi credentials may remain in tracked source, tests, or documentation.
- STOP and FAULT always bypass the mailbox and slew limit and immediately write motor PWM zero.
- Do not claim hardware verification; no physical motor operation is part of implementation.
- Preserve current dirty user changes outside files listed by each task.

## Review Focus

- Wraparound of `millis()`/`micros()` must retain correct due/age comparisons; Task 5 adds boundary assertions for unsigned subtraction.
- A heartbeat arriving after a motion packet but before the control tick must not erase the motion mailbox; Task 3 tests this exact ordering.
- A controller session change during movement must stop before any new-session motion executes; Task 3 tests stop/handshake/retry ordering.
- A same `motion_id` packet with one changed float field must fault rather than reset or continue the goal; Task 4 tests direction and distance changes.
- Packet flood and repeated malformed datagrams must remain bounded and leave the 100 Hz control path runnable; Task 6 tests per-loop packet and time-budget enforcement through injectable limits.

---

### Task 1: Configuration, secrets, and pure motion math

**Files:**
- Create: `arduino/esp32_omni_controller/robot_config.h`
- Create: `arduino/esp32_omni_controller/secrets.example.h`
- Modify: `.gitignore`
- Modify: `arduino/esp32_omni_controller/odometry_math.h`
- Create: `arduino/esp32_omni_controller/firmware_self_test.h`
- Test: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Produces: `robot_config` constants; `BodyDelta`; `inverseKinematicsBody`, `forwardKinematics`, `wheelDistanceCm`, `goalProgressCm`, `lateralErrorCm`, `encoderDeltaLimitCounts`, `brakingSpeedLimit`, and `slewTowards` pure functions.
- Consumes: none.

- [ ] **Step 1: Write failing configuration and math contract tests**

Add tests asserting the exact GitHub pin map, no pin duplication, 15/20/1.0 speed limits, 10 ms control period, 300 ms watchdog, 200 ms telemetry period, the physical-wiring warning, ignored `secrets.h`, and absence of tracked SSID/password literals. Include source checks that the removed `MAX_ENCODER_DELTA_PER_TICK = 2000` no longer exists.

- [ ] **Step 2: Run Task 1 tests and verify failure**

Run: `python -m pytest tests/test_esp32_firmware_contract.py -q`

Expected: FAIL because the new config and self-test headers do not exist.

- [ ] **Step 3: Add centralized configuration and secrets template**

Define all pins, geometry, measured counts/rev, direction arrays, limits, acceleration/deceleration, deadzone arrays, timing, UDP ports, PID gains, feed-forward flags, packet size/count/time budgets, and tolerance constants in `robot_config.h`. Add `secrets.example.h`, ignore `secrets.h`, and use compile-time placeholder macros when local secrets are absent.

- [ ] **Step 4: Replace odometry helpers with coordinate-consistent pure functions**

Implement the named pure functions in `odometry_math.h`. `encoderDeltaLimitCounts` derives the per-tick limit from wheel speed, circumference, counts/rev, dt, and safety margin. `brakingSpeedLimit` returns `sqrt(2*a*remaining)` for positive finite inputs and zero at/below the target. `slewTowards` limits signed change by rate times dt.

- [ ] **Step 5: Add compile-time self-tests**

In `firmware_self_test.h`, add static assertions for zero, pure X/Y/rotation, IK/FK round trip tolerances, progress/lateral separation, encoder threshold order of magnitude, slew limiting, and braking monotonicity.

- [ ] **Step 6: Run Task 1 tests**

Run: `python -m pytest tests/test_esp32_firmware_contract.py -q`

Expected: PASS.

- [ ] **Step 7: Commit Task 1**

```powershell
git add .gitignore arduino/esp32_omni_controller/robot_config.h arduino/esp32_omni_controller/secrets.example.h arduino/esp32_omni_controller/odometry_math.h arduino/esp32_omni_controller/firmware_self_test.h tests/test_esp32_firmware_contract.py
git commit -m "Build ESP32 configuration and motion math core"
```

### Task 2: Safe motor and encoder subsystem

**Files:**
- Create: `arduino/esp32_omni_controller/motor_encoder.h`
- Create: `arduino/esp32_omni_controller/motor_encoder.cpp`
- Modify: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Consumes: `robot_config.h`, `wheelDistanceCm`, derived encoder limit.
- Produces: `MotorEncoderState`; `initializeMotorOutputsSafe`, `enableMotorDriver`, `disableMotorDriver`, `writeMotorPwm`, `stopAllMotorsImmediate`, `attachEncoderInterrupts`, `snapshotEncoderCounts`, `updateMeasuredWheelSpeed`, and `zeroEncoderReference`.

- [ ] **Step 1: Add failing source-contract tests for motor boot and ISR behavior**

Assert STBY LOW precedes PWM attachment and STBY HIGH, direction and PWM outputs initialize LOW/zero, Core 3.x `ledcAttach`/`ledcWrite` are used, direction changes write zero before deadtime, and each ISR only reads B, applies sign, and updates count.

- [ ] **Step 2: Run the focused tests and verify failure**

Run: `python -m pytest tests/test_esp32_firmware_contract.py -q`

- [ ] **Step 3: Implement safe motor output**

Implement the public functions with one fixed three-motor state. `writeMotorPwm` constrains `-255..255`, applies `motorReversed`, writes zero before a sign change, waits configured deadtime, changes direction, writes duty, and records signed PWM. Zero PWM leaves both direction pins LOW.

- [ ] **Step 4: Implement minimal encoder ISR and speed sampling**

Use A RISING and read B for direction. Snapshot all counts in one short interrupt-disabled section. Compute wheel cm/s at the control tick, reject non-finite dt and derived-threshold jumps, then apply the configured low-pass filter.

- [ ] **Step 5: Run Task 2 tests and Arduino compile**

Run the focused pytest file, then Arduino CLI compile for `esp32:esp32:esp32`.

Expected: tests PASS and compile succeeds with no upload.

- [ ] **Step 6: Commit Task 2**

```powershell
git add arduino/esp32_omni_controller/motor_encoder.h arduino/esp32_omni_controller/motor_encoder.cpp tests/test_esp32_firmware_contract.py
git commit -m "Add safe motor and encoder subsystem"
```

### Task 3: Typed protocol, legacy adapter, acceptance, and latest-wins mailbox

**Files:**
- Create: `arduino/esp32_omni_controller/network_protocol.h`
- Create: `arduino/esp32_omni_controller/network_protocol.cpp`
- Create: `tests/test_esp32_protocol_model.py`
- Modify: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Consumes: config absolute limits and packet limits.
- Produces: `CommandType`, `CommandStatus`, `NormalizedCommand`, `DecodeResult`, `AcceptanceResult`, `CommandMailbox`, `decodePacket`, `acceptCommand`, `takeLatestMotion`, and `clearMotionMailbox`.

- [ ] **Step 1: Write failing protocol model tests**

Test typed velocity/move/stop/heartbeat/reset decoding; legacy normalization; missing, non-integer, zero, and changed `session_id`; duplicate/stale ignore; new-session STOP handshake; heartbeat not replacing motion; latest velocity replacement; latest cross-type motion revision; partial legacy distance rejection; non-finite and absolute-limit rejection.

- [ ] **Step 2: Run protocol tests and verify failure**

Run: `python -m pytest tests/test_esp32_protocol_model.py -q`

- [ ] **Step 3: Implement fixed normalized command types and packet decoder**

Use one 512-byte input buffer and ArduinoJson parsing outside the control tick. Decode validates complete shape and finite numeric fields but makes no sequence decision. The legacy adapter creates the same `NormalizedCommand` and never calls motion code.

- [ ] **Step 4: Implement acceptance and mailbox**

Acceptance owns session, last sequence, handshake, and local revision. Duplicate/stale results are IGNORE. New session while moving returns IMMEDIATE_STOP and marks handshake pending. Heartbeat updates liveness without touching the mailbox. Accepted motion replaces the single mailbox slot.

- [ ] **Step 5: Add implementation contract checks**

Assert there is no FIFO/container queue, duplicate/stale classification occurs in acceptance rather than decode, and legacy calls the normalized acceptance interface.

- [ ] **Step 6: Run Task 3 tests and compile**

Run both new test files and Arduino CLI compile.

- [ ] **Step 7: Commit Task 3**

```powershell
git add arduino/esp32_omni_controller/network_protocol.h arduino/esp32_omni_controller/network_protocol.cpp tests/test_esp32_protocol_model.py tests/test_esp32_firmware_contract.py
git commit -m "Add typed ESP32 protocol and latest command mailbox"
```

### Task 4: Motion state, acceleration, braking, odometry, and preserved PID

**Files:**
- Create: `arduino/esp32_omni_controller/kinematics_motion.h`
- Create: `arduino/esp32_omni_controller/kinematics_motion.cpp`
- Create: `tests/test_esp32_motion_model.py`
- Modify: `arduino/esp32_omni_controller/firmware_self_test.h`
- Modify: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Consumes: `NormalizedCommand`, motor encoder snapshot, config, odometry helpers.
- Produces: `MotionState`, `MotionControllerState`, `applyMotionCommand`, `updateMotionState`, `computeLimitedWheelTargets`, `runWheelPid`, `cancelMotionImmediate`, `resetMotionAfterFault`, and goal/odometry telemetry snapshot fields.

- [ ] **Step 1: Write failing motion-model tests**

Test STOPPED/VELOCITY/DISTANCE_ACTIVE/DISTANCE_BRAKING/GOAL_REACHED transitions, body and wheel slew, SLOW cap, braking speed, no reverse after overshoot, completion requiring low speed and lateral tolerance, new/same/changed/completed motion IDs, progress/lateral integration, and PID reset at stop/reached.

- [ ] **Step 2: Run motion tests and verify failure**

Run: `python -m pytest tests/test_esp32_motion_model.py -q`

- [ ] **Step 3: Implement command-state semantics**

Velocity replaces desired body motion. MOVE stores immutable motion ID, unit direction, requested cruise speed, distance, and zero local pose. Same-ID identical retransmission only refreshes liveness. Changed content returns protocol fault. Completed ID remains reached until a different ID.

- [ ] **Step 4: Implement acceleration, IK, normalization, and braking**

Apply body caps, body slew, IK, wheel normalization, then wheel slew. Distance mode limits translation by braking speed and moves to braking state when active. STOP/FAULT never calls these limiters.

- [ ] **Step 5: Implement odometry and completion**

Integrate valid wheel deltas using FK and midpoint heading. Calculate progress, lateral error, remaining, and overshoot. Complete only inside distance tolerance with low wheel speed and valid lateral/encoder state. Overshoot stops without reverse.

- [ ] **Step 6: Port the preserved PID core**

Move the current P/I/D and anti-windup calculation into `runWheelPid` without changing gains, ordering, state arrays, saturation, or zero-target reset. Keep optional LUT feed-forward and per-direction deadzone compensation outside the P/I/D calculation.

- [ ] **Step 7: Run Task 4 tests and compile**

Expected: motion/model tests PASS, static assertions compile, Arduino compile succeeds.

- [ ] **Step 8: Commit Task 4**

```powershell
git add arduino/esp32_omni_controller/kinematics_motion.h arduino/esp32_omni_controller/kinematics_motion.cpp arduino/esp32_omni_controller/firmware_self_test.h tests/test_esp32_motion_model.py tests/test_esp32_firmware_contract.py
git commit -m "Build ESP32 motion state and distance braking"
```

### Task 5: Safety, fault recovery, watchdogs, and telemetry

**Files:**
- Create: `arduino/esp32_omni_controller/safety_telemetry.h`
- Create: `arduino/esp32_omni_controller/safety_telemetry.cpp`
- Create: `tests/test_esp32_safety_model.py`
- Modify: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Consumes: protocol acceptance state, motion state, motor state, Wi-Fi status, timing, telemetry destination.
- Produces: `FaultCode`, `ControllerMode`, `SafetyState`, `immediateStopNow`, `raiseFault`, `tryResetFault`, `updateWatchdogs`, `buildTelemetry`, and `sendTelemetry`.

- [ ] **Step 1: Write failing safety-model tests**

Test immediate stop for explicit STOP, invalid packet, NaN/Inf, hard speed violation, NETWORK Wi-Fi loss, command timeout during velocity and distance, encoder/odometry/path faults, manual timeout independent of Wi-Fi, recoverable handshake, latched explicit reset, and unsigned timer wraparound.

- [ ] **Step 2: Run safety tests and verify failure**

Run: `python -m pytest tests/test_esp32_safety_model.py -q`

- [ ] **Step 3: Implement one immediate stop path and fault policy**

`immediateStopNow` writes zero PWM first, zeros targets, resets PID, clears the mailbox, cancels active motion, and records STOPPED or FAULT. Recoverable and latched fault sets follow the spec exactly.

- [ ] **Step 4: Implement network and manual watchdogs**

NETWORK valid accepted command/heartbeat age is capped at 300 ms in every moving state. MANUAL_TEST ignores Wi-Fi and stops after 3,000 ms without a serial manual command.

- [ ] **Step 5: Implement 5 Hz telemetry and compatibility aliases**

Serialize the typed fields and legacy aliases from one immutable snapshot. Omit or null inapplicable goal values. Send only to the accepted controller address; never serialize inside the control tick.

- [ ] **Step 6: Run Task 5 tests and compile**

Expected: focused tests PASS and Arduino compile succeeds.

- [ ] **Step 7: Commit Task 5**

```powershell
git add arduino/esp32_omni_controller/safety_telemetry.h arduino/esp32_omni_controller/safety_telemetry.cpp tests/test_esp32_safety_model.py tests/test_esp32_firmware_contract.py
git commit -m "Add ESP32 safety faults and telemetry"
```

### Task 6: Thin sketch integration and bounded services

**Files:**
- Replace: `arduino/esp32_omni_controller/esp32_omni_controller.ino`
- Modify: `tests/test_esp32_firmware_contract.py`

**Interfaces:**
- Consumes: every firmware module from Tasks 1-5.
- Produces: Arduino `setup()` and `loop()`, `serviceSerial`, `serviceWiFi`, `serviceUdpRxBudgeted`, `runControlTickIfDue`, and `runTelemetryIfDue`.

- [ ] **Step 1: Add failing sketch integration tests**

Assert setup ordering, no automatic motion, thin loop service ordering, control check before and after UDP, packet-count and microsecond budget checks, no Serial/JSON in PID/control functions, and full manual command inventory.

- [ ] **Step 2: Run integration tests and verify failure**

Run: `python -m pytest tests/test_esp32_firmware_contract.py -q`

- [ ] **Step 3: Replace the monolithic sketch with module composition**

Implement setup in the exact safe order from the spec. Implement a short loop that services due control before UDP, bounded Serial/Wi-Fi/UDP work, control again, then due telemetry. Use unsigned subtraction for wrap-safe scheduling.

- [ ] **Step 4: Implement bounded UDP and Serial services**

UDP stops after four packets or 1,000 microseconds. Each packet is decoded and accepted before any mailbox action. Serial HELP/STATUS/PIN/ENC/ZERO/AUTO/MANUAL/STOP/M1/M2/M3/ALL/VEL/MOVE reuse the same motor or normalized motion paths and never print per control tick.

- [ ] **Step 5: Verify packet-flood review focus**

Expose packet/time-budget decisions as pure small helpers or constants so the contract test proves both bounds and the two control checks. Confirm malformed packets do not create an unbounded loop.

- [ ] **Step 6: Run all firmware-focused tests and compile**

Run all `test_esp32_*` files and Arduino CLI compile.

Expected: PASS and successful compile without upload.

- [ ] **Step 7: Commit Task 6**

```powershell
git add arduino/esp32_omni_controller tests/test_esp32_firmware_contract.py
git commit -m "Integrate modular ESP32 motion controller firmware"
```

### Task 7: Migrate Laptop and Raspberry Pi hosts

**Files:**
- Modify: `tools/esp32_udp_test.py`
- Modify: `tools/laptop_central_controller.py`
- Modify: `raspberry_pi/comm/udp_sender.py`
- Modify: `raspberry_pi/comm/telemetry_receiver.py`
- Modify: `tests/test_esp32_udp_tool.py`
- Modify: `tests/test_laptop_central_controller.py`
- Modify: `tests/test_esp32_interface.py`
- Modify: `tests/test_motion_goal.py`
- Modify: relevant telemetry/transport tests

**Interfaces:**
- Consumes: typed command and telemetry contract from Tasks 3 and 5.
- Produces: nonzero per-process `session_id`; typed velocity/move/stop/heartbeat packets; typed telemetry parsing with temporary legacy aliases.

- [ ] **Step 1: Change host tests to require typed protocol**

Assert each process keeps one nonzero session ID, sequence increases, velocity/move/stop/heartbeat use explicit type, distance fields appear only on MOVE, STOP remains repeated on shutdown, and typed telemetry maps to existing GUI/Pi state without inventing absent values.

- [ ] **Step 2: Run host-focused tests and verify failure**

Run the modified UDP, GUI, interface, goal, telemetry, and transport tests.

- [ ] **Step 3: Implement shared host command construction in each existing boundary**

Generate a random nonzero 32-bit session ID once per sender instance. Emit typed packets. Continue periodic command or heartbeat transmission below 300 ms. Preserve fail-safe STOP bursts and do not add a new dependency.

- [ ] **Step 4: Migrate telemetry readers and GUI fields**

Prefer typed names, fall back to migration aliases, validate finite three-element arrays, expose state/fault/session/goal diagnostics, and keep legacy missing PWM as blank/`--`.

- [ ] **Step 5: Run host tests, Tk smoke, and full regression**

Run: `python -m pytest tests -p no:cacheprovider -q`, Python syntax compile, and a hidden Tk smoke test. Do not connect to ESP32 or send RUN during automated checks.

- [ ] **Step 6: Commit Task 7**

```powershell
git add tools/esp32_udp_test.py tools/laptop_central_controller.py raspberry_pi/comm/udp_sender.py raspberry_pi/comm/telemetry_receiver.py tests/test_esp32_udp_tool.py tests/test_laptop_central_controller.py tests/test_esp32_interface.py tests/test_motion_goal.py
git commit -m "Migrate GRISE hosts to typed ESP32 protocol"
```

### Task 8: Documentation and final verification

**Files:**
- Modify: `README.md`
- Replace: `arduino/README.md`
- Modify: `docs/ODOMETRY_DISTANCE_CONTROL.md`
- Create: `docs/ESP32_MOTION_CONTROLLER.md`
- Modify: `docs/PROJECT_STATUS.md`

**Interfaces:**
- Consumes: completed firmware and host behavior.
- Produces: operator protocol reference, state/fault diagram, configuration guide, and physical hardware test procedure.

- [ ] **Step 1: Update documentation to match implemented names and limits**

Document pin source of truth, coordinate frame, typed and legacy migration packets, state/fault recovery, watchdogs, latest-wins mailbox, telemetry, secrets setup, and the SOFTWARE/HARDWARE verification boundary.

- [ ] **Step 2: Document the physical test sequence**

Include individual M1/M2/M3 direction tests, encoder sign invariant, count/rev measurement, 10 cm/s wheel tests, all wheels, +X, +Y, rotation, 15 cm goal, and 30 cm goal with recorded target/measured/PWM/drift/overshoot data.

- [ ] **Step 3: Run complete verification**

Run full pytest, Python syntax compile, Tk smoke, Arduino CLI compile for Core 3.3.12, and `git diff --check`. Inspect that no credentials or build/cache artifacts are tracked.

- [ ] **Step 4: Review the complete diff against the spec**

Confirm PID core preservation, exact pin map, immediate stop ordering, typed/legacy convergence, mailbox semantics, packet budget, state/fault recovery, telemetry, and all documented limits. Record any hardware-only unknowns without claiming validation.

- [ ] **Step 5: Commit Task 8**

```powershell
git add README.md arduino/README.md docs/ODOMETRY_DISTANCE_CONTROL.md docs/ESP32_MOTION_CONTROLLER.md docs/PROJECT_STATUS.md
git commit -m "Document redesigned ESP32 motion controller"
```
