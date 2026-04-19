# Software Reflections
**CDE2310 — Final Run Post-Mortem**

This document analyses what went wrong during the final run, why it went wrong at the code and parameter level, and what concrete changes would have improved the outcome. Issues are grouped by subsystem.

---

## Table of Contents

1. [Navigation — Exploration & Path Planning](#1-navigation--exploration--path-planning)
2. [Navigation — Stuck Detection & Recovery](#2-navigation--stuck-detection--recovery)
3. [Dynamic Shooting Logic](#3-dynamic-shooting-logic)
4. [Mission Coordination](#4-mission-coordination)
5. [Operational & Infrastructure](#5-operational--infrastructure)
6. [Summary of Improvements](#6-summary-of-improvements)

---

## 1. Navigation — Exploration & Path Planning

**Inflation radius mismatch caused repeated wall contacts.**
The robot bumped into walls 2–3 times consecutively during the run — a clear signal that the inflation radius was insufficient for the environment. The original value of `0.13 m` was too tight; bumping it to `0.15 m` in testing produced noticeably cleaner navigation. On top of this, `INFLATION_R` was hardcoded in `nav_final.py` at a different value than what was set in `nav2_params_frontier.yaml`, meaning the Python node's gap-filtering logic and the actual Nav2 costmap were working off different assumptions — goals the node considered safe were being rejected by the planner. A single source of truth (defined once in YAML, read at runtime via ROS2 parameter) would have eliminated this entirely.

```python
# nav_final.py — was inconsistent with YAML; should be read from ROS2 param instead
INFLATION_R = 0.15   # must match inflation_radius in nav2_params_frontier.yaml exactly
```

More importantly: when we observed multiple consecutive wall contacts during the live run, we should have **restarted the mission immediately** rather than letting it continue. Repeated wall contacts don't self-resolve — the robot was burning time in a degraded state.

---

**Cost scaling factor too low — robot avoided narrow gaps.**
Nav2's inflation layer decays obstacle cost exponentially based on `cost_scaling_factor`. With ours set too low, narrow passages appeared near-impassable to the global planner even when geometrically navigable. The robot consistently chose open, easy routes and deferred tight gaps until all other frontiers were exhausted. Increasing `cost_scaling_factor` (to ~3.0–5.0) in `nav2_params_frontier.yaml` would have steepened the cost decay, making the centre of a navigable gap look meaningfully cheaper than a blocked cell.

---

**Frontier heading bias compounded the narrow-gap avoidance.**
The frontier scorer penalised frontiers requiring significant rotation using `W_HEADING = 1.5` — exactly the frontiers behind narrow passages the robot hadn't yet passed through. Combined with the high costmap cost from the low `cost_scaling_factor`, these regions were doubly penalised and never selected until all easier options were exhausted.

```python
# nav_final.py — frontier scoring
W_HEADING = 1.5   # disproportionately penalises behind-robot frontiers

score = efficiency + W_HEADING * heading_diff - W_SIZE * counts[i]
```

`W_HEADING` should have been reduced to ~0.8–1.0 and tuned jointly with `cost_scaling_factor` on a representative test map before the final run.

---

**Gap scoring had an inverted sign.**
The scoring function penalised gaps *farther* from walls rather than rewarding them, so the planner consistently chose the most wall-adjacent openings. The fix was a single sign change. Unit-testing the scorer on a synthetic set of gap candidates before any hardware run would have caught this immediately.

```python
# Buggy — penalises safer gaps (farther from walls), so robot prefers gaps near walls
score += W_GAP_DIST * d_edge

# Fixed — rewards safer gaps
score -= W_GAP_DIST * d_edge
```

---

## 2. Navigation — Stuck Detection & Recovery

**Stuck detection threshold was too permissive.**
`STUCK_DIST = 0.20 m` over `STUCK_TIME = 6.0 s` meant a robot oscillating against a wall could displace enough laterally to never trigger the stuck condition, so recovery never ran.

```python
# nav_final.py — tightened from 0.20 m / 6.0 s after the run
STUCK_DIST = 0.25    # must move this far in STUCK_TIME or declared stuck
STUCK_TIME = 3.5     # seconds between position snapshots
```

The tighter values help, but the real lesson is operational: don't let it get that far. Multiple wall contacts during a live run are a diagnostic signal to abort, not ignore.

---

**Backup distance was too short to clear the robot's own body.**
`BACKUP_DURATION = 1.8 s` at `BACKUP_SPEED = 0.08 m/s` produced only ~14 cm of reverse travel — less than the robot's chassis length (~18 cm). The robot couldn't clear itself from the obstacle, so the subsequent forward motion immediately re-triggered the same collision.

```python
# nav_final.py — corrected after run
BACKUP_SPEED    = 0.08
BACKUP_DURATION = 3.5     # ~28 cm of reverse travel (was 1.8 s = 14 cm — less than chassis length)
```

Backup distance should have been derived from the robot's physical dimensions from the start, not chosen empirically.

---

**Docstring-recorded fixes didn't match the actual constants.**
The bug-fix log in `nav_final.py` documents STUCK-BUG as fixed to `STUCK_DIST → 0.35 m, STUCK_TIME → 5.0 s`, but the actual constants in the file are `0.25` and `3.5`. Similarly, INFLATE-BUG is documented as fixed to `INFLATION_R = 0.14` but the file has `0.15`. Fixes were tested at one set of values, noted in comments, then partially reverted without updating the comments. A separate changelog, or automated assertions checking key constants match YAML at startup, would have caught this drift.

---

## 3. Dynamic Shooting Logic

**What we did — reactive fire-on-sight.**
After docking to the dynamic station, the coordinator entered `WAITING_DYNAMIC` and watched for the receptacle tag (ID 15). The moment the tag appeared in any frame, it immediately sent `fire_one` to the launcher. This repeated until all 3 balls were fired.

```python
# mission_coordinator_final.py — WAITING_DYNAMIC state
rec_tag = self._find_tag('dynamic_receptacle')
if rec_tag is not None:
    self.get_logger().info(
        f'Receptacle detected — firing ball '
        f'{self.dynamic_balls_fired + 1}/{DYNAMIC_BALL_COUNT}')
    self._pub(self.launch_cmd_pub, 'fire_one')
    self.launch_status = ''
    self.dynamic_balls_fired += 1
    self.state = self.FIRING_DYNAMIC
```

**This worked on the day** — the receptacle was moving slowly enough that firing on first detection reliably landed balls. However, this approach is fragile. If the target moved faster, the ball could easily be fired while the receptacle was already past the optimal position, since there is no account for the delay between detection, command transmission, and actual ball release.

---

**Alternative 1 — Fire only when the tag is centred in frame.**
Rather than firing the instant the tag is detected at any position, wait until the tag's pixel x-coordinate (`cx`) is close to the frame centre before sending the fire command. This ensures the receptacle is directly in front of the launcher at the moment of release.

```python
# Proposed change — add a centering gate before firing
FRAME_CX      = 320                  # frame centre for 640px wide image
CX_TOLERANCE  = 40                   # pixels either side of centre

rec_tag = self._find_tag('dynamic_receptacle')
if rec_tag is not None:
    cx = rec_tag.get('cx', FRAME_CX)
    if abs(cx - FRAME_CX) < CX_TOLERANCE:
        self._pub(self.launch_cmd_pub, 'fire_one')
        # ... rest of firing logic
```

The `cx` field is already published by `apriltag_detector_final.py`, so no changes to the detector are needed.

---

**Alternative 2 — Predictive firing based on tag velocity.**
Track the tag's `cx` position over a short history window and estimate its lateral velocity. Use this to predict where the tag will be after the known system delay (detection → ROS publish → coordinator → launcher → GPIO → ball exits), then fire early enough that the ball arrives when the receptacle is centred.

```python
# Conceptual — track cx history and extrapolate
cx_history.append((time.monotonic(), rec_tag['cx']))

if len(cx_history) >= 3:
    # Estimate velocity from last N samples
    dt  = cx_history[-1][0] - cx_history[-2][0]
    vel = (cx_history[-1][1] - cx_history[-2][1]) / dt   # px/s

    SYSTEM_DELAY_S = 0.15   # estimated: ROS latency + GPIO + ball travel
    predicted_cx   = cx_history[-1][1] + vel * SYSTEM_DELAY_S

    if abs(predicted_cx - FRAME_CX) < CX_TOLERANCE:
        self._pub(self.launch_cmd_pub, 'fire_one')
```

This is more robust to speed but requires tuning `SYSTEM_DELAY_S` against the actual hardware latency.

---

**Alternative 3 — Learn the oscillation period and fire at predicted phase.**
After docking, spend the first few seconds observing the tag's motion to estimate its oscillation period and phase. Then schedule shots at times when the receptacle is predicted to be centred, regardless of whether the tag is visible in that exact frame. This is the most robust approach for a consistently moving target but the most complex to implement and calibrate.

---

## 4. Mission Coordination

**Static-first gate blocked the dynamic target unnecessarily.**
In the v2 coordinator, the dynamic docking sequence was gated on `static_done = True`. If the robot encountered the dynamic target first (which the exploration algorithm didn't prevent), it was silently ignored until static was done. The fix — first-seen priority with no hard gate — was the right call but came too late. This sequencing issue should have been identified during the initial state machine design.

---

**Dock timeout recovery was flat regardless of context.**
After a failed dock attempt, the coordinator returned to exploring with the same 45 s timeout for any future retry. Under time pressure this was wasteful — if the robot was already close to a known target, a proximity heuristic should have triggered docking immediately rather than waiting for a clean re-detection.

```python
# mission_coordinator_final.py
DOCK_TIMEOUT_S = 45.0   # same budget used on every attempt regardless of visit count
```

A per-target attempt counter with a shorter timeout on retries would have been more efficient.

---

**30 s wait for dynamic receptacle tag may have been too short.**
If the receptacle happened to be facing away from the camera at the moment of docking, 30 s was not always enough to guarantee a clean detection window. An abort here forced the robot to re-acquire the dynamic dock — a costly recovery path given the time budget.

```python
# mission_coordinator_final.py
WAITING_DYNAMIC_TIMEOUT_S = 30.0   # abort if receptacle not seen within this many seconds
```

---

## 5. Operational & Infrastructure

**No launch script — 10 manual terminals was a liability.**
The full stack required five terminals on the Pi and five on the laptop, each needing the correct `ROS_DOMAIN_ID`, `source setup.bash`, and launch order. Under time pressure this is error-prone — a wrong domain ID, missing source, or a node launched out of order (e.g. Nav2 before Cartographer) silently breaks the system. A single bash script per machine or a Terminator layout file would have reduced this to one command per machine.

```bash
# Example: run_pi.sh — one command to bring up the full RPi stack
#!/bin/bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash

gnome-terminal \
  --tab -- bash -c "ros2 launch turtlebot3_bringup robot.launch.py; exec bash" \
  --tab -- bash -c "ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888; exec bash" \
  --tab -- bash -c "cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 apriltag_detector_final.py; exec bash" \
  --tab -- bash -c "cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 dock_controller_final.py; exec bash" \
  --tab -- bash -c "cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 launcher_controller_final.py; exec bash"
```

---

**No pre-flight node health check.**
With ten nodes across two machines, there was no mechanism to verify all nodes were up and publishing before the mission started. If any node failed silently, the mission would begin in a degraded state with no immediate indication of what was missing. A simple pre-flight check — asserting a first message on each expected topic within N seconds of startup — would have caught this before `EXPLORING` was entered.

---

## 6. Summary of Improvements

| Area | Issue | Recommended Fix |
|------|--------|-----------------|
| Inflation radius | `0.13 m` too tight; mismatch with YAML caused wall contacts | Increase to `0.15 m`; define once in YAML, read at runtime via ROS2 param |
| Cost scaling | Low `cost_scaling_factor` made narrow passages appear impassable | Increase to ~3.0–5.0 in `nav2_params_frontier.yaml` |
| Frontier bias | `W_HEADING = 1.5` avoided narrow-gap frontiers | Reduce to ~0.8–1.0; tune jointly with `cost_scaling_factor` |
| Gap scoring | Inverted sign caused wall-preferring behaviour | `score -= W_GAP_DIST * d_edge` (fixed in final) |
| Stuck detection | Threshold too permissive; never triggered during wall contacts | `STUCK_DIST = 0.25 m`, `STUCK_TIME = 3.5 s` |
| Backup distance | 14 cm insufficient to clear robot body | Derive from physical dimensions — ≥1.5× chassis length |
| Docstring-code drift | Fix values in comments didn't match actual constants | Automated assertions checking key constants match YAML at startup |
| Dynamic shooting | Fire-on-sight worked but is fragile for faster targets | Add centering gate on `cx`, or predictive firing based on tag velocity |
| Mission sequencing | v2 static_done gate blocked dynamic target | First-seen priority from day one; no sequencing assumptions |
| Dock timeout | Flat 45 s timeout regardless of visit count or proximity | Per-target attempt counter; reduce timeout on retry |
| Dynamic wait | 30 s may be insufficient if receptacle faces away at docking | Increase timeout or trigger re-dock immediately on expiry |
| Launch procedure | 10 manual terminals — error-prone under time pressure | Single bash script per machine or Terminator layout file |
| Node health | No pre-flight verification all nodes are publishing | Pre-flight node asserting first message on each expected topic |
| Operational discipline | Continued run after repeated wall contacts | Abort and restart if robot contacts same surface >1× in ~10 s |
