# Post-Run Reflections
**CDE2310 Autonomous Navigation & Docking — Final Run Analysis**

This document analyses what went wrong during the final run, why it went wrong at the code and parameter level, and what concrete changes would have improved the outcome. Issues are grouped by subsystem.

---

## Table of Contents

1. [Navigation — Exploration & Path Planning](#1-navigation--exploration--path-planning)
2. [Navigation — Stuck Detection & Recovery](#2-navigation--stuck-detection--recovery)
3. [Docking & Detection](#3-docking--detection)
4. [Mission Coordination](#4-mission-coordination)
5. [Operational & Infrastructure](#5-operational--infrastructure)
6. [Summary of Improvements](#6-summary-of-improvements)

---

## 1. Navigation — Exploration & Path Planning

### 1.1 Inverted Gap Scoring (SCORE-BUG)

**What went wrong:** The gap scoring function had the wrong sign on the obstacle distance term:

```python
# Buggy — PENALISES farther obstacles, so robot prefers gaps near walls
score += W_GAP_DIST * d_edge

# Fixed — REWARDS farther obstacles, so robot prefers safer open gaps
score -= W_GAP_DIST * d_edge
```

Since the planner minimises score, adding `+ W_GAP_DIST * d_edge` made gaps with *closer* walls score lower (better), meaning the robot consistently selected the most wall-adjacent openings available. This directly caused the robot to navigate aggressively toward walls rather than through the centre of gaps, which in turn caused repeated wall contacts before the bug was identified.

**What should have happened:** Unit-testing the scoring function on a synthetic set of gap candidates (e.g., one gap at 0.3 m from a wall, one at 1.0 m) would have immediately exposed the inversion before any hardware run.

### 1.2 Robot Avoiding Paths with Obstacles — Cost Scaling and Frontier Bias

**What went wrong:** The robot consistently selected frontier goals in open, unobstructed directions and avoided regions separated from it by narrow passages or obstacle-adjacent corridors. Two compounding factors caused this:

**Factor A — `cost_scaling_factor` too low in `nav2_params_frontier.yaml`.**  
Nav2's inflation layer assigns costs that decay exponentially from obstacle cells according to:

```
cost(d) = 253 × exp(−cost_scaling_factor × (d − inflation_radius))
```

With a low `cost_scaling_factor`, costs near obstacles decay slowly — narrow passages (where both walls contribute elevated cost) appear nearly impassable to the global planner even when geometrically navigable. The planner then either rejected these routes outright or assigned them such high path cost that alternative open routes were always preferred. Increasing `cost_scaling_factor` would have steepened the decay, reducing the effective cost penalty in the centre of a navigable gap relative to a blocked cell.

**Factor B — Frontier scoring weighted away from difficult regions.**  
The map-based frontier fallback scores candidates using:

```python
W_SIZE    = 0.3   # reward larger frontier bins
W_INFO    = 1.8   # reward higher unknown-cell ratio
W_HEADING = 1.5   # penalise frontiers requiring large heading change
```

`W_HEADING = 1.5` disproportionately penalises frontiers behind the robot or requiring significant rotation, which are exactly the frontiers behind narrow passages that the robot had not yet passed through. Combined with the costmap bias from Factor A, these regions were doubly penalised — high heading cost *and* high path cost — so the planner never selected them until all easier frontiers were exhausted. By that point, the time budget was frequently spent.

**What could have been improved:** `W_HEADING` should have been reduced (e.g., 0.8–1.0) and `cost_scaling_factor` increased (e.g., 3.0–5.0) to flatten the heading bias and allow the planner to consider narrow-gap frontiers earlier in the exploration sequence. These two parameters should have been tuned together on a representative test map before the final run.

### 1.3 Old Edge-Based Gap Detection Was Fundamentally Unreliable (GAP-REWORK)

**What went wrong:** The original gap detection found depth-jump edges in the LiDAR scan and computed a target by applying an `asin` shift past the edge. The direction of the shift depended on which of the two edge rays read the higher range value — a comparison that LiDAR range noise flips scan-to-scan. The result was that the computed target alternated between pointing *through* the gap and pointing *away from it* on consecutive scans, causing inconsistent goal selection.

**What replaced it:** A sector-median approach — bin the scan into 20° sectors, compute the median range per sector, cluster adjacent open sectors (median > 1.0 m), and place the goal along the cluster centroid direction. This has no edge geometry and no direction-flip ambiguity. It should have been the first implementation rather than a rework late in the project.

### 1.4 Inflation Radius Mismatch Between Code and YAML (INFLATE-BUG)

**What went wrong:** `INFLATION_R` in `nav_final.py` was set to 0.12 m in earlier revisions while `nav2_params_frontier.yaml` had `inflation_radius: 0.14`. Both round to 3 cells at 0.05 m/cell resolution, so the costmap was numerically correct, but the gap-filtering logic in the Python node used the 0.12 m figure to determine which goals were too close to obstacles. Goals that the costmap considered marginally safe were being accepted by the node as clearly safe, causing Nav2 to reject them on planning — producing frequent goal blacklistings without the node understanding why.

**What could have been improved:** A single source of truth for inflation radius — defined once in the YAML and read at runtime by the node via a ROS2 parameter — would have eliminated the mismatch entirely. Hardcoding the same physical constant in two separate files with no cross-reference check is a maintenance hazard.

---

## 2. Navigation — Stuck Detection & Recovery

### 2.1 Stuck Threshold Was Too Permissive (STUCK-BUG)

**What went wrong:** The stuck detector used `STUCK_DIST = 0.20 m` over `STUCK_TIME = 6.0 s`. A robot oscillating against a wall — repeatedly hitting, backing off slightly, and hitting again — can displace ≥20 cm laterally across the oscillation without ever escaping. The stuck condition never triggered, so the backup recovery never ran, and the robot continued its wall-contact behaviour unchecked.

**What should have been done:** Reducing `STUCK_DIST` and `STUCK_TIME` to tighter values (e.g., 0.25 m / 3.5 s as in the final code) makes the detector fire after fewer oscillation cycles. More importantly: **when the robot is observed hitting the wall 2–3 times consecutively during a run, the mission should have been manually restarted immediately.** Repeated wall contacts are a clear diagnostic signal that either the inflation radius is insufficient for the current environment or the stuck detector is failing — continuing the run compounds the problem and risks hardware damage.

### 2.2 Backup Distance Was Insufficient (BACKUP-BUG)

**What went wrong:** `BACKUP_DURATION = 1.8 s` at `BACKUP_SPEED = 0.08 m/s` produced only 14.4 cm of reverse travel — less than the robot's chassis length (approximately 18 cm). The robot could not clear its own body from the obstacle it was pressed against, so the backup manoeuvre ended with the robot still in contact or within the inflation zone, and the subsequent forward motion immediately re-triggered the same collision.

**What could have been improved:** `BACKUP_DURATION` should have been sized to at minimum clear 1× robot body length, ideally 1.5×. At 0.08 m/s, clearing 27 cm requires 3.4 s — close to the 3.5 s in the final code. This should have been derived from the robot's physical dimensions rather than chosen empirically.

### 2.3 Discrepancy Between Docstring Fixes and Final Constants

**What went wrong:** The bug-fix log in `nav_final.py` documents STUCK-BUG as fixed to `STUCK_DIST → 0.35 m, STUCK_TIME → 5.0 s`, but the actual constants in the file are `STUCK_DIST = 0.25` and `STUCK_TIME = 3.5`. The documented fix was never fully committed to the code. Similarly, the INFLATE-BUG log states the fix sets `INFLATION_R = 0.14` to match YAML, but the final code has `INFLATION_R = 0.15`.

These discrepancies indicate that the fix was tested at one set of values, noted in the docstring, then partially reverted or independently modified without updating the comment. For a system with multiple iterative bug fixes, maintaining a changelog separate from inline comments would have caught this.

---

## 3. Docking & Detection

### 3.1 Detection Range Gate May Have Caused Missed Triggers

**What went wrong:** `DETECTION_RANGE_M = 1.5 m` in the mission coordinator means any tag detected beyond 1.5 m is silently discarded. During exploration, the robot may have passed within LiDAR and camera visibility of a docking target at 1.8–2.0 m range, failed to trigger the docking sequence, and continued navigating away. The tag detector publishes at any range — the 1.5 m cutoff is applied only at the coordinator level.

**What could have been improved:** The range gate exists to prevent false triggers from distant, noisy pose estimates — a valid concern. However, it should have been paired with a "slow down and approach" phase when a tag is detected outside the gate, rather than a hard ignore. Alternatively, the gate could have been relaxed to 2.0 m for initial detection and tightened to 1.5 m only after confirmation across multiple frames.

### 3.2 `decision_margin` Threshold Not Validated on Final Hardware Setup

The `MIN_DECISION_MARGIN = 25.0` threshold in the detector was tuned under one lighting condition. If the final run environment had different ambient lighting (stronger overhead fixtures, shadows, or reflective surfaces), valid detections may have been systematically rejected. The detection rate log (`[RATE]`) would have shown this, but only if someone was monitoring it. No automated alert or coordinator-level fallback existed for the case where the detector was running but producing zero accepted tags.

---

## 4. Mission Coordination

### 4.1 No Explicit Handling for Partial Completion Under Time Pressure

**What went wrong:** The mission coordinator's `DOCK_TIMEOUT_S = 45 s` caused it to cancel a dock attempt and return to EXPLORING without marking the target as completed. This is correct behaviour for recovery, but under time pressure it created a problem: the robot would resume exploration, potentially spend significant time re-navigating to a position where the target was visible again, then attempt a second dock — consuming time that could have been used to locate the second target.

**What could have been improved:** The coordinator should have maintained a per-target visit count and, after a second failed dock attempt, applied a reduced timeout on the third attempt rather than using the same 45 s budget unconditionally. Alternatively, a target-proximity heuristic — if the robot is already within 0.5 m of a known target location, trigger docking immediately rather than waiting to re-detect.

### 4.2 V2 Static-First Gate Blocked Dynamic Target Unnecessarily

**What went wrong:** In `mission_coordinator_v2.py`, the dynamic docking sequence was gated on `static_done = True`. If the robot encountered the dynamic target first during exploration, it was silently ignored until the static sequence completed. Given that the exploration algorithm did not guarantee any particular discovery order, this gate could have cost significant time — the robot might have been positioned next to the dynamic target while waiting for static completion.

The v3 fix (first-seen priority, no gate) was the correct architectural decision but was implemented late. This should have been identified as a potential sequencing issue during the initial state machine design.

### 4.3 `WAITING_DYNAMIC_TIMEOUT_S = 30 s` May Have Been Too Short

After docking to the dynamic target, the coordinator waits up to 30 s for the receptacle tag (ID 15) to become visible. If the receptacle was moving and happened to be oriented away from the camera at the moment of docking, 30 s may have been insufficient. The coordinator aborted and resumed navigation, requiring the robot to re-acquire the dynamic dock target — a costly recovery path.

---

## 5. Operational & Infrastructure

### 5.1 No Launch Script or Multiplexer — Manual Terminal Management

**What went wrong:** The stack requires five terminals on the Pi and five on the laptop, each with distinct environment exports and commands. Launching this manually under time pressure introduces errors — wrong `ROS_DOMAIN_ID`, missing `source setup.bash`, wrong working directory, or a node launched out of order (e.g., Nav2 before Cartographer). Any of these silently breaks the system in ways that are not immediately obvious.

**What should have been in place:** A Terminator layout file or a single bash script per machine handles this reliably:

```bash
#!/bin/bash
# run_pi.sh — launch full RPi stack in a Terminator split layout
terminator --layout=pi_layout &
sleep 2

# Or equivalently, using gnome-terminal:
gnome-terminal \
  --tab -- bash -c "export TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=42; source /opt/ros/humble/setup.bash; ros2 launch turtlebot3_bringup robot.launch.py; exec bash" \
  --tab -- bash -c "export ROS_DOMAIN_ID=42; source /opt/ros/humble/setup.bash; ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888; exec bash" \
  --tab -- bash -c "export ROS_DOMAIN_ID=42; source /opt/ros/humble/setup.bash; cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 apriltag_detector_final.py; exec bash" \
  --tab -- bash -c "export ROS_DOMAIN_ID=42; source /opt/ros/humble/setup.bash; cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 dock_controller_final.py; exec bash" \
  --tab -- bash -c "export ROS_DOMAIN_ID=42; source /opt/ros/humble/setup.bash; cd ~/turtlebot3_ws/src/py_pubsub/py_pubsub && python3 launcher_controller_final.py; exec bash"
```

A proper ROS2 launch file combining all three RPi nodes into a single `ros2 launch` invocation would have been even cleaner, eliminating manual working directory management entirely.

### 5.2 No Automated Node Health Check

With ten nodes spread across two machines, there was no mechanism to verify that all nodes were up and publishing before the mission was started. If any one node failed to start, the mission would begin in a degraded state with no immediate indication of which component was missing. A pre-flight checklist node — subscribing to each expected topic and asserting a first message within N seconds of startup — would have caught this before `EXPLORING` was entered.

### 5.3 Wall Contacts Should Have Triggered an Immediate Mission Restart

When the robot was observed bumping into a wall 2–3 times consecutively, the run should have been manually restarted rather than allowed to continue. Repeated wall contacts within a short window are a definitive indicator of one of two failure modes — inflation radius insufficient for the environment, or stuck detection not triggering — neither of which self-resolves. Continuing the run after this observation consumed time budget while the robot was in an unrecoverable degraded state.

A clear pre-run decision rule would have helped: *if the robot contacts the same obstacle surface more than once within a 10-second window, abort and restart with adjusted parameters.*

---

## 6. Summary of Improvements

| Area | Issue | Recommended Fix |
|------|--------|-----------------|
| Gap scoring | Inverted sign caused wall-preferring behaviour | `score -= W_GAP_DIST * d_edge` (already fixed in final) |
| Frontier bias | `W_HEADING = 1.5` systematically avoided narrow-gap frontiers | Reduce to 0.8–1.0; tune with `cost_scaling_factor` jointly |
| Cost scaling | Low `cost_scaling_factor` made narrow passages appear impassable | Increase to 3.0–5.0 in `nav2_params_frontier.yaml` |
| Stuck detection | Threshold too permissive; stuck never triggered during wall contact | `STUCK_DIST = 0.25 m`, `STUCK_TIME = 3.5 s` (partially applied) |
| Backup distance | 14 cm insufficient to clear robot body | Size to ≥1.5× robot body length from physical dimensions |
| Inflation mismatch | `INFLATION_R` inconsistent between code and YAML | Single YAML parameter, read at runtime via ROS2 param |
| Docstring-code drift | Fix values in comments did not match actual constants | Automated test asserting key constants match YAML at startup |
| Detection range | Hard 1.5 m gate silently discards valid but distant detections | Two-stage gate: detect at 2.0 m, confirm at 1.5 m |
| Mission sequencing | v2 static_done gate blocked dynamic target unnecessarily | First-seen priority (v3 fix — implement from the start) |
| Dock timeout recovery | Flat 45 s timeout regardless of visit count | Per-target attempt counter; reduce timeout on retry |
| Launch procedure | 10 manual terminals — error-prone under time pressure | Single bash script per machine or ROS2 launch file |
| Node health | No pre-flight verification that all nodes are publishing | Pre-flight node asserting first message on each topic |
| Operational discipline | Continued run after repeated wall contacts | Abort-and-restart rule: >1 wall contact in 10 s → restart |
