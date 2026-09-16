# PROJECT F: Sensor Fusion and Risk

## Input and output

`SensorFusion.build_from_states(VisionState, RadarState, now=monotonic_time)` consumes the existing D/E snapshots. It does not perform detection, decode TI serial data, associate camera objects with radar targets, or choose a motion direction. It returns `WorldState` with the existing `robot`, `radar_connected`, `radar_target`, `obstacles`, `blocked`, `sensor_valid`, `vision` fields and new diagnostics: `vision_valid`, `radar_valid`, `radar_status`, both sensor ages, environment `sectors`, all `radar_targets`, and `threat`. `radar_target` remains the selected target for existing consumers. `RiskEvaluator.evaluate(world)` returns `RiskState` with level, reason, distance, approach speed, TTC, threat direction, and timestamp.

## Coordinates and selection

Vision sectors are copied without redetection. Unknown or invalid vision maps to blocked legacy direction flags. Fake Radar 0° is radar front; positive angle is left. `RADAR_MOUNT_YAW_DEG` is added to normalized radar angle before selecting a robot sector with the D convention. Actual TI sign must be normalized in E once measured. F does not rotate raw TI values directly.

All valid targets are retained. For each target, the existing distance/TTC thresholds yield a raw SAFE/WARN/DANGER rank. The primary target has the highest rank; ties choose shorter TTC, then shorter distance, then earlier input order. The nearest target is therefore not always the primary threat. `approaching_target_present` answers whether any target approaches; `threat.detected` means the selected target approaches. A stationary/receding target can still trigger a very-close distance rule. `threat.direction` is a measured sector, not an escape decision.

## Risk policy

The initial, uncalibrated thresholds remain: distance DANGER `<15cm`, WARN `<35cm`; approach-speed threshold `>25cm/s`; TTC DANGER `<0.8s`, WARN `<1.6s`. TTC is `distance_cm / approach_speed_cm_s` only for positive approach speed. It is a straight radial approximation, not a collision trajectory prediction. Distance can cause WARN/DANGER even without approach. The evaluator retains distance/speed EMA (`0.4/0.3`) and DANGER/WARN hold (`0.7/0.4s`). A raw high-risk observation is not delayed by EMA. A target ID change or a large ID-less distance/angle jump resets EMA and hold. Non-approaching observations clear approach-speed EMA. A valid empty frame can retain a held level briefly, but an invalid sensor clears hold and produces `RiskState.level="INVALID"`.

## Freshness and safety boundary

Vision and Radar receive timestamps use Pi monotonic time. Vision must have a detected, finite, fresh pose and eight sectors containing CLEAR, BLOCKED, or UNKNOWN; Radar must be connected, valid, fresh, structurally consistent, and contain finite targets. A partial UNKNOWN sector does not invalidate the camera; G excludes that direction. The existing `WORLD_STALE_S=0.35` and `RADAR_STALE_S=0.35` are used. `VISION_RADAR_MAX_SKEW_S=0.35` limits receive-time difference; it does not synchronize hardware clocks. Invalid or stale sensor input makes `WorldState.sensor_valid=false`; the existing control gate then issues STOP. It is never labeled SAFE. F only produces data and risk; the final STOP authority remains with the existing safety path.

Pi monitor adds optional validity, threat, TTC and risk-reason fields while preserving existing fields. The ESP32 command JSON, telemetry, and Laptop camera interface are unchanged.

## Hardware tuning still required

The actual Radar serial parser remains unavailable in E. With an IWR6843AOP stream, verify raw velocity and azimuth signs, mount yaw, range/velocity error, frame timing, dropped-frame and skew limits, TTC approximation, risk distance and hold values. Camera scale, chassis size, obstacle sectors, and end-to-end latency also require measurement before hardware verification.
