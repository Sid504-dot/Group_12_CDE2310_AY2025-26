# Software Reflections
**CDE2310 — Final Run Post-Mortem**

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

**Inflation radius mismatch caused repeated wall contacts.**
The robot bumped into walls 2–3 times consecutively during the run — a clear signal that the inflation radius was insufficient for the environment. The original value of `0.13 m` was too tight; bumping it to `0.15 m` in testing produced noticeably cleaner navigation. On top of this, `INFLATION_R` was hardcoded in `nav_final.py` at a different value than what was set in `nav2_params_frontier.yaml`, meaning the Python node's gap-filtering logic and the actual Nav2 costmap were working off different assumptions — goals the node considered safe were being rejected by the planner. A single source of truth (defined once in YAML, read at runtime via ROS2 parameter) would have eliminated this entirely.

More importantly: when we observed multiple consecutive wall contacts during the live run, we should have **restarted the mission immediately** rather than letting it continue. Repeated wall contacts don't self-resolve — the robot was burning time in a degraded state.

**Cost scaling factor too low — robot avoided narrow gaps.**
Nav2's inflation layer decays obstacle cost exponentially based on `cost_scaling_factor`. With ours set too low, narrow passages appeared near-impassable to the global planner even when geometrically navigable. The robot consistently chose open, easy routes and deferred tight gaps until all other frontiers were exhausted. Increasing `cost_scaling_factor` (to ~3.0–5.0) would have steepened the cost decay, making the centre of a navigable gap look meaningfully cheaper than a blocked cell — and actually getting the robot through those passages earlier.

**Frontier heading bias compounded the narrow-gap avoidance.**
The frontier scorer applied `W_HEADING = 1.5`, which disproportionately penalised frontiers behind the robot or requiring significant rotation — exactly the frontiers behind narrow passages the robot hadn't yet passed through. Combined with the high costmap cost from the low `cost_scaling_factor`, these regions were doubly penalised and never selected until all easier options were exhausted. `W_HEADING` should have been reduced to ~0.8–1.0 and tuned jointly with `cost_scaling_factor` on a representative test map before the final run.

**Gap scoring had an inverted sign.**
The scoring function penalised gaps *farther* from walls rather than rewarding them, so the planner consistently chose the most wall-adjacent openings available. The fix was a single sign change (`score -= W_GAP_DIST * d_edge`). Unit-testing the scorer on a synthetic set of gaps before any hardware run would have caught this immediately.

---

## 2. Navigation — Stuck Detection & Recovery

**Stuck detection threshold was too permissive.**
`STUCK_DIST = 0.20 m` over `STUCK_TIME = 6.0 s` meant a robot oscillating against a wall — repeatedly hitting, backing off, and hitting again — could displace enough laterally to never trigger the stuck condition. Recovery never ran. Tightening to `0.25 m / 3.5 s` helps, but the more important lesson is the operational one above: don't let it get that far.

**Backup distance was too short to clear the robot's own body.**
`BACKUP_DURATION = 1.8 s` at `BACKUP_SPEED = 0.08 m/s` produced only ~14 cm of reverse travel — less than the robot's chassis length (~18 cm). The robot could not clear itself from the obstacle, so the subsequent forward motion immediately re-triggered the same collision. Backup distance should have been derived from the robot's physical dimensions from the start: clearing 1.5× body length at 0.08 m/s requires ~3.4 s, close to the 3.5 s in the final code.

**Docstring-recorded fixes didn't match the actual constants.**
The bug-fix log in `nav_final.py` documents STUCK-BUG as fixed to `STUCK_DIST → 0.35 m, STUCK_TIME → 5.0 s`, but the actual constants are `0.25` and `3.5`. Similarly, INFLATE-BUG is documented as fixed to `INFLATION_R = 0.14` but the file has `0.15`. Fixes were tested at one set of values, noted in comments, then partially reverted without updating the comments. A separate changelog, or at minimum automated assertions checking key constants match YAML at startup, would have caught this drift.

---

## 3. Docking & Detection

**Hard detection range gate silently discarded valid targets.**
The mission coordinator dropped any tag detected beyond `1.5 m`. During exploration, the robot likely passed within camera range of a docking target at ~1.8–2.0 m and simply ignored it, then navigated away. A two-stage approach — detect at 2.0 m, confirm across multiple frames at 1.5 m — would have caught these without increasing false trigger risk.

**Detection margin threshold not validated under final run lighting.**
The `MIN_DECISION_MARGIN = 25.0` threshold in the detector was tuned under one lighting condition. If the final run environment had different ambient lighting, valid detections may have been systematically rejected. The detection rate log (`[RATE]`) would have shown this, but only if someone was actively monitoring it — there was no automated alert or coordinator-level fallback for the case where the detector was running but producing zero accepted tags.

---

## 4. Mission Coordination

**Static-first gate blocked the dynamic target unnecessarily.**
In the v2 coordinator, the dynamic docking sequence was gated on `static_done = True`. If the robot encountered the dynamic target first (which the exploration algorithm didn't prevent), it was silently ignored until static was done. The fix — first-seen priority with no hard gate — was the right call but came too late. This sequencing issue should have been identified during the initial state machine design.

**Dock timeout recovery was flat regardless of context.**
After a failed dock attempt, the coordinator returned to exploring with the same 45 s timeout for any future retry. Under time pressure this was wasteful — if the robot was already close to a known target, a proximity heuristic should have triggered docking immediately rather than waiting for a clean re-detection. A per-target attempt counter with a shorter timeout on retries would have helped.

**30 s wait for dynamic receptacle tag may have been too short.**
If the receptacle happened to be facing away from the camera at the moment of docking, 30 s was not enough to guarantee a clean detection window. An abort here forced the robot to re-acquire the dynamic dock — a costly recovery path given the time budget.

---

## 5. Operational & Infrastructure

**No launch script — 10 manual terminals was a liability.**
The full stack required five terminals on the Pi and five on the laptop, each needing the correct `ROS_DOMAIN_ID`, `source setup.bash`, and launch order. Under time pressure this is error-prone — wrong domain ID, missing source, or a node launched out of order (e.g. Nav2 before Cartographer) can silently break the system in ways that aren't immediately obvious. A single bash script per machine or a Terminator layout file would have reduced this to one command per machine and eliminated this class of errors entirely.

**No pre-flight node health check.**
With ten nodes across two machines, there was no mechanism to verify all nodes were up and publishing before the mission started. If any one node failed silently, the mission would begin in a degraded state with no immediate indication of which component was missing. A simple pre-flight check — asserting a first message on each expected topic within N seconds of startup — would have caught this before `EXPLORING` was entered.

---

## 6. Summary of Improvements

| Area | Issue | Recommended Fix |
|------|--------|-----------------|
| Inflation radius | `0.13 m` too tight; caused wall contacts | Increase to `0.15 m`; define once in YAML, read at runtime |
| Cost scaling | Low `cost_scaling_factor` made narrow passages appear impassable | Increase to ~3.0–5.0 in `nav2_params_frontier.yaml` |
| Frontier bias | `W_HEADING = 1.5` avoided narrow-gap frontiers | Reduce to ~0.8–1.0; tune jointly with `cost_scaling_factor` |
| Gap scoring | Inverted sign caused wall-preferring behaviour | `score -= W_GAP_DIST * d_edge` (fixed in final) |
| Stuck detection | Threshold too permissive; never triggered during wall contacts | `STUCK_DIST = 0.25 m`, `STUCK_TIME = 3.5 s` |
| Backup distance | 14 cm insufficient to clear robot body | Size to ≥1.5× robot body length from physical dimensions |
| Docstring-code drift | Fix values in comments didn't match actual constants | Automated test asserting key constants match YAML at startup |
| Detection range | Hard 1.5 m gate silently discarded valid distant detections | Two-stage gate: detect at 2.0 m, confirm at 1.5 m |
| Detection margin | Threshold not validated under final run lighting | Monitor `[RATE]` log actively; add coordinator-level fallback alert |
| Mission sequencing | v2 static_done gate blocked dynamic target | First-seen priority from day one; no sequencing assumptions |
| Dock timeout | Flat 45 s timeout regardless of visit count or proximity | Per-target attempt counter; reduce timeout on retry |
| Dynamic wait | 30 s may be insufficient if receptacle faces away | Increase wait window or add re-dock on timeout |
| Launch procedure | 10 manual terminals — error-prone under time pressure | Single bash script per machine or Terminator layout file |
| Node health | No pre-flight verification all nodes are publishing | Pre-flight node asserting first message on each expected topic |
| Operational discipline | Continued run after repeated wall contacts | Abort and restart if robot contacts same surface >1× in 10 s |
