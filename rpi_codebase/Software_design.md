# RPi Node Implementation Notes
**Supplement to `README.md` — focuses on implementation internals, control logic, design decisions, and tuning. Refer to `README.md` for setup instructions, launch commands, parameter listings, and node API references.**

---

## Table of Contents

1. [Inter-Node Data Contract](#1-inter-node-data-contract)
2. [AprilTag Detector — Implementation Detail](#2-apriltag-detector--implementation-detail)
3. [Dock Controller — FSM Logic & Control Math](#3-dock-controller--fsm-logic--control-math)
4. [Launcher Controller — Timing & Concurrency](#4-launcher-controller--timing--concurrency)
5. [Parameter Tuning Guide](#5-parameter-tuning-guide)
6. [Failure Mode Analysis](#6-failure-mode-analysis)

---

## 1. Inter-Node Data Contract

The three nodes are coupled only through the `/apriltag/detections` JSON schema. Any change to this schema — field names, units, or coordinate conventions — must be propagated to all consumers simultaneously. The schema is reproduced here as the authoritative reference:

```json
{
  "tags": [
    {
      "id":         25,
      "type":       "dynamic_dock",
      "cx":         318.4,
      "cy":         241.2,
      "dist":       0.412,
      "yaw_offset": -0.043,
      "tx":         -0.018,
      "ty":          0.031,
      "tz":          0.411,
      "corners":    [[x0,y0], [x1,y1], [x2,y2], [x3,y3]]
    }
  ],
  "stamp": 1718023456.123
}
```

**Coordinate conventions (camera frame):**
- `tx` — lateral displacement (positive = tag is to the right of the optical axis)
- `ty` — vertical displacement (positive = tag is below the optical axis)
- `tz` — depth (positive = tag is in front of the camera)
- `yaw_offset` = `atan2(tx, tz)` in radians — positive when tag is to the right
- `dist` = `‖[tx, ty, tz]‖₂` — Euclidean distance, not depth

The dock controller uses **`tz`** (depth) as its longitudinal control input, not `dist`. This is intentional: at oblique approach angles, `dist > tz`, and using `dist` would cause the robot to stop further away than intended. `tz` directly measures the forward standoff from the tag face regardless of lateral offset.

**`type` string values and their source IDs:**

| `type` string | Source IDs |
|---|---|
| `"static"` | `[0]` |
| `"dynamic_dock"` | `[25]` |
| `"dynamic_receptacle"` | `[15]` |
| `"unknown"` | any other ID |

Downstream nodes filter on `type`, not on numeric `id`. Reassigning a physical tag to a different role requires only updating the ID lists in `apriltag_detector_final.py` — the controller and launcher require no changes.

---

## 2. AprilTag Detector — Implementation Detail

### 2.1 Grayscale Conversion

The `pupil-apriltags` detector requires a single-channel `uint8` image. The conversion from RGB applies ITU-R BT.601 luminance weights rather than a naive channel average, as the latter over-weights blue and produces lower contrast for tag black-white patterns under typical indoor lighting:

```python
gray = (
    0.299 * frame[:, :, 0] +   # R
    0.587 * frame[:, :, 1] +   # G
    0.114 * frame[:, :, 2]     # B
).astype(np.uint8)
```

The 0.587 weight on green reflects its dominant contribution to human-perceived luminance, and correspondingly to the high-contrast edge features the AprilTag quad detector relies on.

### 2.2 Why `quad_sigma = 0.8`

The IMX219 sensor at 640×480 produces visible fixed-pattern noise at typical indoor exposure levels. The `quad_sigma` parameter applies a Gaussian blur (σ = 0.8 px) prior to gradient computation in the quad detector stage. Without this, noise edges produce spurious quad candidates that the decoder must subsequently reject, increasing both false-positive rate and CPU load. σ = 0.8 was selected as the minimum value that eliminated the observed noise artefacts without measurably reducing detection range in testing.

### 2.3 Why `quad_decimate = 1.0`

`quad_decimate` sub-samples the input image before quad detection. A value of 2.0 halves both dimensions (quarter the pixels), reducing CPU load but also reducing the minimum detectable tag size — specifically, small angular subtensions at range. Given the 80 mm tag and the 0.08 m `DOCK_DISTANCE` target, the relevant detection range spans approximately 0.08–1.5 m. At 1.5 m, the tag subtends roughly 30 px at 640×480 with the given intrinsics. Sub-sampling at 2.0 would reduce this to 15 px, near the minimum the `tag36h11` decoder can reliably decode. `quad_decimate = 1.0` was retained to preserve detection reliability at range, accepting the higher per-frame CPU cost.

### 2.4 `decision_margin` and the 25.0 Threshold

`decision_margin` is produced by the bit-decoding stage of `pupil-apriltags`. After projecting the detected quad into a canonical form, the decoder computes a soft margin between the best-matching tag ID and the second-best match. A margin near zero indicates that two tag IDs are nearly equally plausible — the pose estimate from such a detection is unreliable because the homography was computed from an ambiguously classified quad.

The threshold of 25.0 was established empirically: at margins below 25, pose measurements exhibited jumps exceeding 5 cm in `tz` between consecutive frames under static conditions. Above 25, frame-to-frame variation in `tz` was below 3 mm. The threshold is intentionally conservative; reducing it below ~15 re-introduces the high-variance pose behaviour.

### 2.5 Detection Rate Log as a Diagnostic Tool

The `[RATE]` log line emitted every 5 seconds provides a structured split between two distinct failure modes that are otherwise indistinguishable from the controller's perspective (no tags published):

| Observation | Root Cause |
|---|---|
| FPS = 0 | Camera pipeline fault — `camera_ros` not running, topic mismatch, or encoding unsupported |
| FPS > 0, all counts = 0 | Detection failure — calibration error, lighting, tag obscured, or margin filter too aggressive |
| FPS > 0, count for target type = 0 only | Tag ID misconfiguration, or wrong physical tag in scene |

---

## 3. Dock Controller — FSM Logic & Control Math

### 3.1 Why Two Separate FSM Layers

The outer state (`idle / docking / docked / backing_up`) is the only state the mission coordinator observes. Keeping the inner docking phases (`COARSE_YAW / APPROACH / HOLD`) invisible to the coordinator serves two purposes: (1) the coordinator's logic is not complicated by docking internals it cannot influence, and (2) the inner FSM can be redesigned without altering the coordinator's state transition table.

### 3.2 COARSE_YAW: Pure Rotation Before Translation

The deliberate sequencing of yaw alignment before longitudinal approach is motivated by the pose geometry. When the robot is laterally offset from the tag's optical axis, `yaw_offset = atan2(tx, tz)` is non-zero. Driving forward in this state moves the robot along a path that does not converge on the tag face — it approaches a point displaced by `tx` laterally. Zeroing yaw first ensures the subsequent approach is approximately collinear with the tag's surface normal.

The angular velocity command is a simple proportional controller:

```
wz = clip( -KP_YAW × yaw_offset,  -MAX_WZ,  MAX_WZ )
```

The negative sign is required by the coordinate convention: positive `yaw_offset` (tag to the right) demands negative angular velocity (rotate left, i.e. counterclockwise viewed from above) to reduce the error.

**FIX-9 — overshoot with yaw misalignment:** If the robot is past the dock point (`tz < DOCK_DISTANCE - DIST_TOLERANCE`) but heading is also misaligned, the correct action is to correct yaw *before* entering APPROACH. Dispatching immediately to APPROACH in this condition caused a feedback loop in rev11: the reverse leg produced yaw drift past `YAW_DRIFT_LIMIT`, aborting back to COARSE_YAW, which re-detected the overshoot and re-dispatched to APPROACH indefinitely. The fix gates the overshoot transition on `abs(yaw_off) < YAW_COARSE_THRESH`.

### 3.3 APPROACH: Coupled Longitudinal and Angular Control

The longitudinal velocity command is a bidirectional P-controller with a deadband floor:

```
dist_err = tz - DOCK_DISTANCE

raw_vx = KP_DIST × dist_err

if dist_err > 0:   vx = clip( raw_vx,  MIN_VX,  MAX_VX )   # forward
else:              vx = clip( raw_vx, -MAX_VX, -MIN_VX )   # reverse
```

The minimum speed floor `MIN_VX = 0.02 m/s` exists because the TurtleBot3 Burger's motor driver exhibits a deadband: command voltages below the friction threshold produce no wheel torque. A pure P-controller with small residual `dist_err` naturally produces sub-deadband commands, causing the robot to stall short of `DOCK_DISTANCE`. The floor guarantees actuation for any `|dist_err| > DIST_TOLERANCE`.

**FIX-8 — secondary yaw correction during APPROACH:** Rev11 commanded zero angular velocity throughout APPROACH. Over a 1.0 m approach at `MAX_VX = 0.06 m/s`, 16+ seconds of purely translational motion allowed heading to drift past `YAW_HOLD_THRESH` before reaching HOLD, immediately triggering the HOLD escape path and restarting from COARSE_YAW. A secondary proportional yaw term is now applied simultaneously:

```
wz = clip( -KP_YAW_APPROACH × yaw_offset,  -MAX_WZ_APPROACH,  MAX_WZ_APPROACH )
```

`KP_YAW_APPROACH = 0.20` and `MAX_WZ_APPROACH = 0.15 rad/s` are deliberately half the COARSE_YAW values. At full `KP_YAW = 0.45`, the angular term dominates at moderate yaw errors, producing a crab-walk motion that extends approach time. The reduced gain corrects heading gradually while the linear drive remains the primary axis of control. The `YAW_DRIFT_LIMIT = 0.15 rad` abort threshold is retained as a hard backstop for cases where the secondary correction is insufficient.

### 3.4 HOLD: Confirmation Counting with Decay

HOLD accumulates a count of consecutive in-tolerance ticks rather than measuring time, because the control loop runs at a nominal 50 Hz that may not be perfectly maintained under CPU load. The confirmation criterion is therefore expressed in ticks (`DOCKED_CONFIRM_TICKS = 15`) rather than seconds.

Out-of-tolerance ticks subtract 1 from the count (REV-9E decay) rather than resetting to zero. The rationale is that a single bad detection frame — caused by a momentary lighting change or a dropped packet — should not nullify a sequence of 14 good consecutive ticks. The decay rate of 1 per tick means a single bad frame delays confirmation by 2 ticks (one to subtract, one to re-earn) rather than resetting the entire 15-tick window.

**FIX-7 — yaw-drift escape from HOLD:** When the count decays to zero, two exit conditions are evaluated in priority order:

1. `abs(dist_err) > DIST_TOLERANCE × 2` → APPROACH (distance is the primary axis; APPROACH will also correct yaw via FIX-8)
2. `abs(yaw_off) > YAW_HOLD_THRESH` → COARSE_YAW (distance acceptable but heading degraded)

Without condition 2, a robot with valid distance but excessive yaw had no valid HOLD exit — `_docked_count` would decay to zero and no transition would fire, leaving the FSM permanently wedged in HOLD. The `DIST_TOLERANCE × 2` threshold for condition 1 (rather than `DIST_TOLERANCE`) prevents APPROACH re-entry on minor distance fluctuations that the HOLD controller cannot correct anyway since it commands zero velocity.

### 3.5 Tag Loss Tier Rationale

The 6.0 s full-loss timeout (`LOST_TIMEOUT`) is considerably longer than the 0.3 s brief-loss threshold. This asymmetry is intentional: the Pi 4 under concurrent ROS2 load can drop several consecutive camera frames, causing detection gaps of 0.5–1.5 s that are not genuine tag losses. A 3.0 s timeout (as used in earlier revisions) triggered spurious FSM resets during a nearly complete approach under CPU contention. At `MAX_VX = 0.06 m/s`, the robot travels a maximum of 360 mm in 6 s — well within recoverable range for a COARSE_YAW re-acquisition.

The 0.3 s brief-loss threshold is bounded by the kinematic constraint: at `MAX_VX`, 0.3 s of stale pose propagation corresponds to at most 18 mm of positional change, which is within the `DIST_TOLERANCE = 30 mm` band. Beyond 0.3 s the stale pose can no longer be trusted for control.

---

## 4. Launcher Controller — Timing & Concurrency

### 4.1 Overall-Delay vs. Fixed Inter-Shot Sleep

A fixed inter-shot sleep of the form `time.sleep(INTERVAL)` appended after each `_fire_one_ball()` call would produce wall-clock gaps of `FIRE_PULSE_DURATION + sleep_overhead + INTERVAL`. Since `FIRE_PULSE_DURATION = 0.25 s` and Python `time.sleep` can incur OS scheduling jitter of ±20 ms, the cumulative timing error across a 3-shot sequence would be on the order of 0.5–0.7 s.

REV-5A eliminates this by computing the residual sleep from the shot start timestamp:

```python
t_shot    = time.monotonic()
_fire_one_ball()                                     # ~0.25 s + overhead
remaining = STATIC_INTER_SHOT_DELAYS[shot_num-1] - (time.monotonic() - t_shot)
if remaining > 0:
    time.sleep(remaining)
```

The wall-clock gap between shot starts is then `STATIC_INTER_SHOT_DELAYS[i] ± scheduling_jitter`, where jitter is bounded by the OS scheduler resolution (~10 ms on Linux with `SCHED_OTHER`). This is the approach used in real-time audio and motion control systems where per-event timing must be stable regardless of per-iteration execution time.

### 4.2 Daemon Thread and the `firing` Flag

Fire sequences are dispatched to daemon threads rather than called directly from `_cmd_cb`, because `_cmd_cb` executes on the ROS2 executor's spin thread. A blocking `time.sleep` inside a callback would prevent the executor from processing any other topic or timer callbacks for the duration of the sleep — the node would become unresponsive to `/mission/launch_command: stop` commands, which is a safety concern.

The `firing` flag is set to `True` *synchronously on the callback thread* before `threading.Thread.start()` is called. This ordering is critical: if the flag were set inside the thread function, a second callback invocation arriving before the thread executes its first line would observe `firing = False` and spawn a second concurrent thread, potentially firing two solenoid pulses simultaneously. The synchronous pre-set eliminates this race regardless of thread scheduling latency.

---

## 5. Parameter Tuning Guide

This section describes the sensitivity and interaction of the key tunable constants. Cross-reference the parameter definitions in `README.md`.

### Dock Controller

**`DOCK_DISTANCE`** — sets the target `tz` at which the robot is considered docked. Reduce if the launcher requires closer standoff. Note that reducing below ~0.05 m risks the robot contacting the tag surface before pose estimation degrades, as the tag may partially exit the camera FOV.

**`KP_DIST` / `KP_YAW`** — increasing either gain improves convergence speed but risks oscillation. The system is not formally characterised, but empirically: `KP_DIST > 0.40` produces overshoot oscillation at the `DOCK_DISTANCE` threshold; `KP_YAW > 0.60` causes angular overshoot that repeatedly triggers `YAW_DRIFT_LIMIT` aborts.

**`YAW_COARSE_THRESH` vs. `YAW_DRIFT_LIMIT`** — these two parameters must satisfy `YAW_COARSE_THRESH < YAW_DRIFT_LIMIT`. If COARSE_YAW exits at a yaw error greater than `YAW_DRIFT_LIMIT`, APPROACH will abort on its first tick. A margin of at least 0.05 rad between the two thresholds is recommended.

**`YAW_HOLD_THRESH` vs. `YAW_COARSE_THRESH`** — HOLD uses a looser yaw threshold (0.10 rad) than the COARSE_YAW exit (0.08 rad). This prevents HOLD from immediately escaping to COARSE_YAW due to the small heading drift that occurs during the final approach deceleration. If `YAW_HOLD_THRESH ≤ YAW_COARSE_THRESH`, the robot will frequently oscillate between HOLD and COARSE_YAW.

**`DOCKED_CONFIRM_TICKS`** — increasing this value reduces false dock confirmations at the cost of additional time in HOLD. At 50 Hz, each tick is 20 ms; 15 ticks corresponds to 300 ms of continuous in-tolerance pose. Reduce only if the launcher timing is sensitive to dock latency.

### Detector

**`MIN_DECISION_MARGIN`** — reducing this value increases recall (fewer valid detections rejected) at the cost of precision (more noisy poses admitted). The recommended operating range is 15–35. Below 15, spurious `tz` jumps of >5 cm per frame have been observed. Above 35, valid detections at range (>1 m) may be rejected due to reduced image contrast at small angular subtension.

**`TAG_SIZE_M`** — this is a scale factor on the entire pose estimate, not a detection parameter. Incorrect values produce proportional depth error: `tz_actual = tz_measured × (TAG_SIZE_M_actual / TAG_SIZE_M_configured)`.

---

## 6. Failure Mode Analysis

The following table catalogues observed failure modes, their observable signatures, and the corrective action. These were encountered during iterative hardware testing and motivated the revision history documented in each source file's docstring.

| Failure | Observable Signature | Root Cause | Corrective Action |
|---|---|---|---|
| Systematic undershoot at `DOCK_DISTANCE` | Robot stops 2–5 cm short and does not converge | `MIN_VX` below motor deadband | Measure deadband empirically; raise `MIN_VX` |
| HOLD → COARSE_YAW oscillation | Status alternates `docking/docking/...` without reaching `docked` | `YAW_HOLD_THRESH ≤ YAW_COARSE_THRESH` | Ensure `YAW_HOLD_THRESH > YAW_COARSE_THRESH` by ≥0.02 rad |
| Permanent HOLD with no `docked` | FSM stuck, count at 0, no phase transition | Yaw bad, distance OK — FIX-7 missing (pre-rev12) | Upgrade to rev12; verify `YAW_HOLD_THRESH` is not too tight |
| APPROACH → COARSE_YAW → APPROACH loop | Robot oscillates without net forward progress during approach | Overshot robot dispatched to APPROACH with bad yaw (pre-rev12) | Upgrade to rev12; FIX-9 gates overshoot transition on yaw threshold |
| No tags detected despite tag in frame | `[RATE]` shows FPS > 0, all counts = 0 | `MIN_DECISION_MARGIN` too high, poor lighting, or tag substrate causing specular reflection | Lower `MIN_DECISION_MARGIN` cautiously; improve illumination; use matte substrate |
| Wrong depth estimate | Robot stops at clearly wrong distance | `TAG_SIZE_M` misconfigured | Measure printed tag precisely; recalibrate |
| DDS topic discovery fails silently | `ros2 topic echo` shows no messages despite node running | FastDDS multicast blocked on hotspot network | Configure unicast peer list; set `FASTRTPS_DEFAULT_PROFILES_FILE` |
| GPIO HIGH after crash | Solenoid remains energised after node termination | `SIGKILL` bypasses `destroy_node()` cleanup | Run manual GPIO reset: `GPIO.output(21, GPIO.LOW); GPIO.cleanup()` |
| Second fire command ignored | `'Already firing'` warning in log, no second sequence | Expected — `firing` lock functioning correctly | Design mission coordinator to wait for `static_done` or `dynamic_done` before re-issuing |
| Inter-shot gaps inconsistent | Gaps vary by 0.2–0.5 s across runs | Fixed `time.sleep` after pulse (pre-rev5) | Upgrade to rev5; overall-delay computation accounts for pulse duration |