# Software Design — Remote PC Codebase

This document describes the high-level design rationale behind the autonomous navigation and mission control system. It covers algorithm choices, key design decisions, safety mechanisms, Nav2 configuration, and tuning guidance. For the detailed API reference (parameters, attributes, methods), see [Developer_Guide.md](Developer_Guide.md).

---

## System Architecture Overview

The remote PC runs two ROS2 nodes with distinct responsibilities that communicate via topics.

| Node | File | Responsibility |
|------|------|----------------|
| `Nav2GapNav` | `nav_final.py` | Autonomous exploration — selects goals, manages Nav2, handles stuck recovery |
| `MissionCoordinator` | `mission_coordinator_final.py` | Mission logic — detects AprilTags, orchestrates docking, firing, and backing up |

The division of responsibility is intentional. Navigation and mission control are kept completely separate so each can be developed and tested independently. The coordinator never touches `/cmd_vel` directly — it only issues pause and resume commands to the navigation node, which handles all movement.

The RPi runs three additional nodes: the AprilTag detector, the dock controller, and the launcher controller. All six nodes communicate exclusively via `std_msgs/String` topics carrying JSON payloads — there are no custom message types in the system.

---

## Why `std_msgs/String` with JSON Instead of Custom Messages

Custom ROS2 message types require the package to be compiled on every machine that uses them. On the Raspberry Pi, this adds significant build overhead and introduces version mismatch risks between the laptop and RPi workspaces.

Using `std_msgs/String` with JSON sidesteps this entirely. Every node can publish and subscribe to standard String topics with no compilation dependency. The tradeoff is a small amount of serialisation overhead per message, which is negligible at the message rates used here (≤ 20 Hz).

The mission coordinator parses incoming JSON like this:

```python
def _det_cb(self, msg: String):
    try:
        data = json.loads(msg.data)
        self.latest_tags   = data.get('tags', [])
        self.last_det_time = time.monotonic()
    except json.JSONDecodeError:
        pass
```

If the JSON is malformed for any reason, the decode error is silently caught and the message is dropped — the node never crashes due to a bad detection message.

---

## Navigation Design

### Goal Selection Strategy

The navigation node uses a **gap-first** strategy: it always tries to find a navigable LiDAR opening first, and only falls back to map-based frontier detection if no suitable gap exists.

```python
# From _goal_tick() in nav_final.py
goal = self._select_gap(rx, ry, yaw)
goal_type = 'gap'
if goal is None:
    goal = self._select_frontier(rx, ry, yaw)
    goal_type = 'frontier'
```

The motivation for prioritising gaps over frontiers is responsiveness. Frontier selection requires processing the full occupancy grid, which is slower and depends on having a reasonably complete map. Gap selection works directly from the raw LiDAR scan and produces a goal within milliseconds regardless of map quality. In the early stages of exploration when the map is sparse, gap selection is significantly more reliable.

If both return `None`, the node clears the blacklist and retries before giving up:

```python
if goal is None and self.blacklist:
    self.get_logger().info(
        f'No goal — clearing normal blacklist ({len(self.blacklist)} entries).')
    self.blacklist = []
    gap_r      = self._select_gap(rx, ry, yaw)
    frontier_r = self._select_frontier(rx, ry, yaw)
```

---

### Gap Detection: Open-Sector Approach

The final implementation uses **open-sector detection**. The entire LiDAR scan is binned into 20° sectors, the median range per sector is computed, adjacent open sectors (median > 1.0 m) are clustered into contiguous openings, and a goal is placed 0.9 m along the centre direction of the best opening.

#### Why Not Edge-Based Detection?

The initial implementation used depth-jump edge detection: find the transition from a near ray to a far ray, shift the target sideways past the wall using an arcsine calculation, and drive through the gap.

This approach had a fundamental noise sensitivity problem. The shift direction (left or right) depended on which of the two edge rays read a higher range value. A few millimetres of LiDAR noise was enough to flip this comparison between consecutive scans, so the computed goal target randomly pointed into the gap or directly at the wall. The robot would drive toward a wall, notice an obstacle, and repeat.

The open-sector approach eliminates this entirely. Instead of asking "where does the wall end?", it asks "where is there actually open space?". There is no edge geometry, no arcsine, and no direction that can be flipped by noise.

#### Step-by-Step Implementation

**Step 1 — Bin LiDAR rays into 20° sectors and take the median per sector:**

```python
sector_rad = math.radians(SECTOR_DEG)   # SECTOR_DEG = 20
bucket_idx = ((ray_angles_rf + math.pi) / sector_rad).astype(int)
n_sectors  = int(math.ceil(2 * math.pi / sector_rad))

sector_median = np.full(n_sectors, 0.0)
for s in range(n_sectors):
    mask  = (bucket_idx == s)
    vals  = self.scan_ranges[mask]
    valid = vals[~np.isnan(vals)]
    sector_median[s] = float(np.median(valid)) if valid.size > 0 else 0.0
```

The median is used rather than the mean because a single noisy ray reading zero or infinity in a sector should not pull the result — the median is robust to outliers.

**Step 2 — Mark open sectors and cluster into contiguous openings:**

```python
open_flags = sector_median > OPEN_THRESH_M   # OPEN_THRESH_M = 1.0 m

openings = []
i = 0
while i < n_sectors:
    if open_flags[i]:
        j = i
        while j < n_sectors and open_flags[j]:
            j += 1
        span_deg  = (j - i) * SECTOR_DEG
        centre_rf = (sector_centre[i] + sector_centre[j - 1]) / 2.0
        openings.append((span_deg, centre_rf))
        i = j
    else:
        i += 1
```

**Step 3 — Filter, place goal, and score:**

```python
for span_deg, centre_rf in openings:
    if span_deg < MIN_OPEN_DEG:   # MIN_OPEN_DEG = 40°, discard narrow gaps
        continue

    # Place goal 0.9 m along the opening centre direction
    centre_wf = yaw + centre_rf
    tx = rx + OPEN_GOAL_D * math.cos(centre_wf)
    ty = ry + OPEN_GOAL_D * math.sin(centre_wf)

    # Validity filters: in map, not blocked, not blacklisted, has info gain
    ...

    # Score: penalise turning, reward width
    align_cost = abs(centre_rf)
    score = W_OPEN_ALIGN * align_cost - W_OPEN_WIDTH * span_deg
```

The score penalises openings that require a large turn (`W_OPEN_ALIGN = 2.0`) and rewards wider openings (`W_OPEN_WIDTH = 0.4`). The robot prefers to drive straight through wide gaps rather than spin to face a narrow one.

---

### Map-Based Frontier Fallback

When no LiDAR gap passes the validity filters, the node falls back to **map-based frontier detection**. Frontier cells — free cells adjacent to at least one unknown cell — are identified using a vectorised numpy operation:

```python
free    = (grid >= 0) & (grid < OCC_THRESH)
unknown = (grid < 0)
adj_unk = np.zeros((H, W), dtype=bool)
adj_unk[1:-1, 1:-1] = (unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
                        unknown[1:-1, :-2] | unknown[1:-1, 2:])
pts = np.argwhere(free & adj_unk)
```

These frontier cells are then binned spatially into 10×10 cell clusters. The centroid of each cluster is the candidate goal. Candidates are scored by:

```python
efficiency = dist / (ig_r + 0.1)
score = efficiency + W_HEADING * heading_diff - W_SIZE * counts[i]
```

This rewards goals that are close, information-rich, and roughly ahead of the robot, while penalising small isolated frontier clusters that are unlikely to lead anywhere useful.

---

### Why Nav2 for Path Execution

Rather than implementing our own obstacle avoidance and path following, path execution is delegated entirely to the Nav2 stack. Nav2 handles global path planning, local trajectory control, and recovery behaviours (spin, backup, wait) through its behaviour tree. This lets the navigation node focus entirely on goal selection.

The Nav2 goal is sent as a `NavigateToPose` action with the heading oriented toward the goal:

```python
def _send_nav2_goal(self, gx, gy, rx, ry):
    heading_yaw = math.atan2(gy - ry, gx - rx)
    qz = math.sin(heading_yaw / 2.0)
    qw = math.cos(heading_yaw / 2.0)

    pose = PoseStamped()
    pose.header.frame_id    = 'map'
    pose.pose.position.x    = gx
    pose.pose.position.y    = gy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    goal_msg      = NavigateToPose.Goal()
    goal_msg.pose = pose
    future = self.nav_client.send_goal_async(goal_msg)
    future.add_done_callback(self._on_nav_response)
```

The specific Nav2 components used are:

| Component | Choice | Reason |
|-----------|--------|--------|
| Global planner | `SmacPlanner2D` with `allow_unknown=true` | Plans into unmapped space, essential for frontier exploration |
| Local controller | `MPPIController` | Smoother trajectories in narrow spaces than rule-based controllers like DWB |
| Recovery | Nav2 default behaviour tree | Handles spin, backup, and wait without custom code |

#### Why MPPI Over DWB

The MPPI (Model Predictive Path Integral) controller continuously samples multiple candidate velocity trajectories and selects the one with the lowest expected cost. This predictive sampling produces smoother motion in tight spaces compared to DWB (Dynamic Window Approach), which applies hard velocity limits and struggles in narrow corridors.

In terms of tight-space performance: **MPPI > Pure Pursuit > DWB**.

---

## Safety Mechanisms — Navigation Node

### Stuck Detection and Recovery

The navigation node checks for stuck conditions at 10 Hz inside `_control_tick()`. A position snapshot is taken every `STUCK_TIME` seconds. If the robot has moved less than `STUCK_DIST` in that window, it is declared stuck:

```python
elif now_t - self.stuck_check_t > STUCK_TIME:
    moved = math.hypot(rx - self.stuck_check_pos[0],
                       ry - self.stuck_check_pos[1])
    if moved < STUCK_DIST:
        self.get_logger().warn(
            f'Stuck detected (moved {moved:.3f}m in {STUCK_TIME}s) — backing up.')
        self.stuck_pos      = (rx, ry)
        self.backup_start_t = time.monotonic()
        self._cancel_nav2_goal()
        self._blacklist_goal()
        self.state = 'BACKING_UP'
```

During backup, the node checks rear clearance using the LiDAR and decides whether to reverse or spin:

```python
rear_d = self._arc_min(180.0, 30.0)
cmd = Twist()
if rear_d > BACKUP_REAR_CLEAR:
    cmd.linear.x = -BACKUP_SPEED    # back up
else:
    cmd.linear.x  =  0.05           # rear blocked: nudge fwd + spin
    cmd.angular.z =  0.6
self._cmd_pub.publish(cmd)
```

#### Why 0.25 m and 3.5 s?

The original thresholds were 0.20 m over 6.0 s. Testing showed that a robot oscillating against a wall can easily drift 20 cm in 6 seconds without escaping — the robot bounces back and forth covering distance without making progress. Stuck detection never fired. Tightening to 0.25 m over 3.5 s catches these cases reliably within the first or second oscillation cycle.

#### Backup Duration

The backup duration of 3.5 seconds at 0.08 m/s covers approximately 28 cm. The original 1.8 s only covered 14 cm, which was often not enough to clear the robot body from the obstacle it was stuck against.

### Blacklisting

After backup completes, the stuck position is added to the stuck-blacklist, and if the same region has been stuck 3 or more times it is buried with 5 duplicate entries:

```python
same_area = sum(
    1 for bx, by, _ in self.stuck_blacklist
    if math.hypot(bx - self.stuck_pos[0], by - self.stuck_pos[1]) < REGION_BL_R
)
if same_area >= REGION_STUCK_THRESH:
    self.get_logger().warn(f'Region inaccessible — burying.')
    for _ in range(5):
        self.stuck_blacklist.append((gx, gy, time.monotonic()))
```

The node maintains two separate blacklists:

- **Normal blacklist** (`BLACKLIST_R = 0.80 m`): Goals that Nav2 consistently rejects. Entries expire after 270 seconds.
- **Stuck blacklist** (`STUCK_BL_R = 1.00 m`): Positions where the robot physically got stuck. Larger rejection radius. Entries expire after 360 seconds.

### Idempotent Pause and Resume

The pause and resume commands are both idempotent — sending pause when already paused is a no operation, and sending resume when not paused is a no operation:

```python
if cmd == 'pause':
    if self._paused:
        return                      # no operation — already paused
    self._paused = True
    if self.state == 'BACKING_UP':
        self.stuck_pos      = None  # abort active backup (INT-BUG 3 fix)
        self.backup_start_t = None
    if self.state == 'NAVIGATING':
        self._cancel_nav2_goal()
    self._blacklist_goal()          # INT-BUG 1 fix: blacklist before stopping
    self._stop()
    self.state = 'PAUSED'

elif cmd == 'resume':
    if not self._paused:
        return                      # no operation — not paused
    self._paused       = False
    self.frontier_goal = None
    self.goal_start_t  = None
    self.state         = 'GAP_SELECT'
```

The blacklisting on pause is critical. Without it (the original INT-BUG 1), the navigation node would cancel the Nav2 goal but keep `frontier_goal` set. On resume, `_goal_tick()` would immediately re-select the same goal and re-send it to Nav2, making the pause appear to have no effect.

---

## Mission Control Design

### Finite State Machine

The `MissionCoordinator` is implemented as a **polling-based finite state machine** running at 20 Hz. A single `_tick()` method checks the current state and transitions based on inputs from tag detections, dock status, and launcher status.

The states are defined as class-level string constants so they are readable in logs and topic output:

```python
class MissionCoordinator(Node):
    EXPLORING          = 'EXPLORING'
    DOCKING_STATIC     = 'DOCKING_STATIC'
    FIRING_STATIC      = 'FIRING_STATIC'
    BACKING_UP_STATIC  = 'BACKING_UP_STATIC'
    COOLDOWN           = 'COOLDOWN'
    DOCKING_DYNAMIC    = 'DOCKING_DYNAMIC'
    WAITING_DYNAMIC    = 'WAITING_DYNAMIC'
    FIRING_DYNAMIC     = 'FIRING_DYNAMIC'
    BACKING_UP_DYNAMIC = 'BACKING_UP_DYNAMIC'
    DONE               = 'DONE'
```

At every tick the current state is published, making it trivial to monitor the mission from any terminal:

```python
def _tick(self):
    self._pub(self.state_pub, self.state)   # broadcast state at 20 Hz
```

The choice of polling over event-driven callbacks is deliberate. With polling, every state transition is visible in one place (`_tick`), making the logic easy to trace and debug. Event-driven designs scatter transition logic across multiple callbacks, which makes it harder to reason about race conditions when dock status, tag detections, and launch confirmations can all arrive simultaneously.

---

### EXPLORING State — Tag Detection

In EXPLORING state, the coordinator scans for AprilTags at every tick. Static is checked first (priority), then dynamic:

```python
if self.state == self.EXPLORING:
    self._dock_sent         = False
    self._fire_sent         = False
    self._backup_done_latch = False

    # Static has priority when both are simultaneously visible
    if not self.static_done:
        tag = self._find_tag('static')
        if tag is not None:
            self._start_docking(tag, self.DOCKING_STATIC)
            return

    # Dynamic check is independent — no static_done gate
    if not self.dynamic_done:
        tag = self._find_tag('dynamic_dock')
        if tag is not None:
            self._start_docking(tag, self.DOCKING_DYNAMIC)
            return

    if self.static_done and self.dynamic_done:
        self.state = self.DONE
```

The `_find_tag()` helper enforces both a staleness check and a range limit:

```python
def _find_tag(self, tag_type):
    if time.monotonic() - self.last_det_time > TAG_STALE_S:  # TAG_STALE_S = 1.0s
        return None
    best = None
    for t in self.latest_tags:
        if t['type'] == tag_type and t['dist'] < DETECTION_RANGE_M:  # 1.5m range
            if best is None or t['dist'] < best['dist']:
                best = t
    return best
```

This ensures the robot only acts on tags that are both fresh (seen within the last 1 second) and close enough to be reliable.

---

### Target Priority: Final vs Earlier Design

In an earlier version of the coordinator, the dynamic target sequence was gated behind full completion of the static sequence. If the dynamic marker was seen first, it was silently ignored:

```python
# Earlier version behaviour (removed):
if not self.static_done:
    self.get_logger().info('Ignoring dynamic tag because static_done=False')
    return
```

The final version (`mission_coordinator_final.py`) removes this gate entirely. Whichever target type is detected first is docked and fired immediately. This change makes the mission more robust — if the static target is obstructed or inaccessible early in the run, the robot can complete the dynamic sequence first and retry static later. If both targets are visible simultaneously, static is still checked first as a conservative default.

---

### _start_docking() — Shared Transition Helper

When a tag is detected, the coordinator uses a shared helper to initiate docking for both target types:

```python
def _start_docking(self, tag, target_state: str):
    self.get_logger().info(
        f'Tag {tag["id"]} ({tag["type"]}) at {tag["dist"]:.2f}m'
        f' — pausing nav, settling 0.5s → {target_state}')
    self._pause_nav()
    self._dock_sent        = False
    self._dock_start_t     = time.monotonic()
    self._nav_settle_until = time.monotonic() + 0.5   # 0.5s settle window
    self.state = target_state
```

The 0.5-second settle window is important. When navigation is paused, the robot may still be decelerating. Sending the dock command immediately could mean the dock controller starts visual servoing while the robot is still moving, causing it to overshoot the target.

---

### DOCKING State — Waiting for Confirmed Dock

Once in a docking state, the coordinator waits for the dock controller to confirm before firing:

```python
elif self.state == self.DOCKING_STATIC:
    self._pause_nav()

    if not self._dock_sent:
        if time.monotonic() < self._nav_settle_until:
            return                         # still in settle window
        self._pub(self.dock_cmd_pub, 'dock_static')
        self._dock_sent = True
        return

    if self._dock_timed_out():             # 45s hard timeout
        self._abort_dock_retry('DOCKING_STATIC')
        return

    if self.dock_status == 'docked':
        self._pub(self.launch_cmd_pub, 'fire_static')
        self.launch_status = ''
        self.state = self.FIRING_STATIC

    elif self.dock_status == 'lost' and self._lost_too_long():  # 8s debounce
        self._pub(self.dock_cmd_pub, 'cancel')
        self._resume_nav()
        self.state = self.EXPLORING
```

The same pattern applies to `DOCKING_DYNAMIC`.

---

### Dock Lost Debouncing

When the dock controller reports a tag has been lost, the coordinator does not immediately abort. It tracks when the lost state began and only acts after 8 continuous seconds:

```python
def _dock_status_cb(self, msg: String):
    s = msg.data.strip().lower()
    if s == 'lost' and self.dock_status != 'lost':
        self._dock_lost_since = time.monotonic()    # record when lost started
    elif s != 'lost':
        self._dock_lost_since = None                # reset if recovered

def _lost_too_long(self) -> bool:
    return (self._dock_lost_since is not None and
            time.monotonic() - self._dock_lost_since > DOCK_LOST_DEBOUNCE_S)
```

Without this debounce, a single dropped camera frame would abort the entire docking sequence and send the robot back to exploring. The 8-second window is long enough to ride out transient detection gaps while still being short enough to abort quickly if the tag is genuinely gone.

---

### WAITING_DYNAMIC State — Receptacle-Triggered Firing

The dynamic target requires waiting for a receptacle tag to appear before firing each ball. Each appearance of the receptacle triggers exactly one shot:

```python
elif self.state == self.WAITING_DYNAMIC:
    # Timeout check — abort if no receptacle seen within 30s
    if (time.monotonic() - self._waiting_dynamic_start_t > WAITING_DYNAMIC_TIMEOUT_S):
        self._pub(self.dock_cmd_pub, 'cancel')
        self._resume_nav()
        self.state = self.EXPLORING
        return

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

After each confirmed shot (`FIRING_DYNAMIC`), if more balls remain, the coordinator returns to `WAITING_DYNAMIC` to wait for the next receptacle appearance:

```python
elif self.state == self.FIRING_DYNAMIC:
    if ('dynamic_fired' in self.launch_status or 'dynamic_done' in self.launch_status):
        if self.dynamic_balls_fired >= DYNAMIC_BALL_COUNT:
            self._pub(self.dock_cmd_pub, 'backup')
            self.state = self.BACKING_UP_DYNAMIC
        else:
            self.launch_status = ''
            self.state = self.WAITING_DYNAMIC   # wait for next receptacle
```

---

### Dock Timeout and Abort-with-Retry

If the entire dock and fire cycle takes longer than 45 seconds, the coordinator aborts but does **not** mark the target as done — allowing retry on a future encounter:

```python
def _abort_dock_retry(self, label: str):
    self.get_logger().warn(
        f'{label}: dock timeout ({DOCK_TIMEOUT_S}s) — '
        f'cancelling and resuming nav (target NOT marked done, will retry)')
    self._pub(self.dock_cmd_pub, 'cancel')
    self._dock_start_t            = None
    self._dock_lost_since         = None
    self._waiting_dynamic_start_t = None
    self._resume_nav()
    self.state = self.EXPLORING
```

---

### BACKING_UP States — Latch Pattern

After firing, the coordinator sends a backup command and waits for the dock controller to confirm completion using a latch flag:

```python
def _dock_status_cb(self, msg: String):
    ...
    if s == 'backup_done':
        self._backup_done_latch = True    # set by callback

elif self.state == self.BACKING_UP_STATIC:
    if self._backup_done_latch:           # checked by tick
        self.static_done = True
        self._set_ignore('static')        # tell detector to ignore static tags
        self._resume_nav()
        self.state = self.EXPLORING
```

The latch is reset to `False` at the start of each EXPLORING cycle so it cannot carry over from a previous sequence.

---

### TEST_MODE

Setting `TEST_MODE = True` changes the behaviour after completing the static sequence. Instead of permanently ignoring static tags, the coordinator enters a 10-second cooldown and then returns to EXPLORING ready to trigger the static target again:

```python
if TEST_MODE:
    self.get_logger().info(f'TEST MODE — cooldown {STATIC_COOLDOWN_S}s')
    self._cooldown_t = time.monotonic()
    self._resume_nav()
    self.state = self.COOLDOWN
else:
    self._set_ignore('static')    # production: never trigger static again
    self._resume_nav()
    self.state = self.EXPLORING
```

This allows repeated dock-and-fire testing without restarting the node.

---

## Nav2 Configuration (`nav2_params_frontier.yaml`)

The Nav2 stack is configured via `nav2_params_frontier.yaml`. Below are the key settings and the rationale behind each, including changes made during the run itself.

### Global Planner — SmacPlanner2D

```yaml
GridBased:
  plugin: "nav2_smac_planner/SmacPlanner2D"
  tolerance: 0.20
  allow_unknown: true
```

Setting `allow_unknown: true` allows the planner to compute paths through unmapped cells. This is essential for frontier exploration — the robot must be able to plan routes toward areas it has not yet mapped. Without this, the planner refuses to route through unknown space and no frontier goals can be reached.

However, this creates a subtle bug: gap goals placed in unknown space would be accepted by the node's validity filter (which only rejects occupied cells) but then cause the robot to follow a path into an unmapped wall. The fix in `nav_final.py` is to require all gap goals to land in **known-free** cells:

```python
gval = self.map_raw[cell[1], cell[0]]
if gval < 0 or gval >= OCC_THRESH:   # unknown (-1) or occupied → reject
    continue
```

---

### MPPI Controller — Speed and Sampling

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  time_steps: 56
  batch_size: 2000
  vx_max: 0.11
  vx_min: -0.05
  wz_max: 1.0
```

`vx_max` was lowered from 0.16 m/s to 0.11 m/s. At 0.16 m/s, the pipeline latency (costmap at 10 Hz + MPPI at 20 Hz, approximately 150 ms total) gave around 2.4 cm of blind travel per update cycle. At 0.11 m/s this is reduced to around 1.7 cm, giving more reaction margin when navigating near walls.

`batch_size: 2000` means the controller samples 2000 candidate trajectories per cycle. A higher batch size improves trajectory quality at the cost of compute time. 2000 was found to be a practical balance on the laptop.

---

### Obstacle and Inflation Layers — Costmap Tuning

#### Inflation Radius — Mid-Run Adjustment

```yaml
inflation_layer:
  inflation_radius: 0.15
  cost_scaling_factor: 5.4
```

We initially set `inflation_radius` to **0.13 m**. During the run, the robot was bumping into walls too frequently — the 0.13 m buffer was too tight for the robot body width in the test environment. We increased this to **0.15 m mid-run**, which added a larger safety margin around all obstacles and reduced wall collisions noticeably.

This value must match `INFLATION_R` in `nav_final.py`. If they differ, the internal costmap used for goal filtering disagrees with Nav2's costmap, causing goals to be accepted by the node but rejected by Nav2.

`cost_scaling_factor: 5.4` controls how steeply the inflation cost decays away from an obstacle. A higher value means cost falls off faster, so the planner still strongly prefers the centre of corridors despite the increased inflation radius.

#### Phantom Wall Bug — Raytrace Range Mismatch

```yaml
obstacle_layer:
  observation_sources: scan
  scan:
    obstacle_max_range: 2.5
    raytrace_max_range: 2.5   # fixed — was previously 2.0
```

In an earlier configuration, `obstacle_max_range` was 2.5 m but `raytrace_max_range` was only 2.0 m. This caused a phantom wall problem: cells between 2.0 and 2.5 m were **marked** as occupied when an obstacle was detected there, but they were never **cleared** by raytrace as the robot moved past (because raytrace only clears up to its own range). These stale phantom cells persisted in the costmap and forced SmacPlanner2D to route around them — sometimes into real walls. Setting both values to 2.5 m fixed this.

#### Collision Margin

```yaml
ObstaclesCritic:
  collision_margin_distance: 0.30   # raised from 0.20
```

Raising `collision_margin_distance` from 0.20 m to 0.30 m means the MPPI controller starts penalising trajectories that approach within 0.30 m of obstacles rather than 0.20 m. With `inflation_radius = 0.15 m`, the 0.20 m margin barely overlapped the inflated zone before the robot was already close to the wall. At 0.30 m, MPPI begins deflecting trajectories 15 cm before entering the inflation zone, providing earlier course correction.

#### Robot Footprint

```yaml
footprint: "[[0.26, 0.13], [0.26, -0.13], [-0.13, -0.13], [-0.13, 0.13]]"
```

An explicit rectangular footprint is used rather than a circular `robot_radius`. This more accurately represents the TurtleBot3 Burger's actual shape. Mismatched footprints between the global and local costmaps (where the global planner approves a path that the local controller then treats as a collision) were a known issue that explicit matching footprint values prevent.

---

## Key Bug Fixes and Lessons Learned

### Gap Scoring Sign Error

The initial gap scoring formula penalised gaps with farther walls rather than rewarding them:

```python
# WRONG — penalises safe gaps (farther wall = higher score = worse)
score = W_OPEN_ALIGN * align_cost + W_GAP_DIST * d_edge

# CORRECT — rewards safer gaps (wider opening = lower score = better)
score = W_OPEN_ALIGN * align_cost - W_OPEN_WIDTH * span_deg
```

Since the goal is to **minimise** score, the `+` sign meant the robot consistently preferred gaps where the wall was very close — the most dangerous options. A single character change had a significant behavioural impact.

**Lesson:** When implementing minimisation-based scoring, verify every term's sign. A reward should subtract from the score; a penalty should add.

---

### Stuck Detection Threshold Too Loose

The original thresholds (0.20 m, 6.0 s) meant a robot oscillating against a wall could move 20 cm in 6 seconds without escaping — stuck detection never fired. The fix was to measure actual oscillation distances during a real stuck event and tighten to 0.25 m over 3.5 s.

**Lesson:** Stuck detection thresholds must be validated against real hardware, not estimated. An oscillating robot covers more distance than intuition suggests.

---

### Pause Not Blacklisting Current Goal (INT-BUG 1)

When the mission coordinator issued a pause, the navigation node cancelled the Nav2 goal but did not blacklist it. On resume, `_goal_tick()` would immediately re-select the same goal and re-send it, making the pause appear ineffective.

```python
# Fix: always blacklist before stopping
self._blacklist_goal()
self._stop()
self.state = 'PAUSED'
```

**Lesson:** When interrupting an in-progress action, explicitly invalidate the interrupted state. Simply stopping is not enough if the resume path can reconstruct the same state.

---

### Nav Result Firing During Pause (INT-BUG 2)

The `_on_nav_result` callback fired while `_paused=True` and transitioned state from `NAVIGATING` to `GAP_SELECT`, causing the node to select a new goal even while paused.

```python
# Fix: guard state transition on _paused
if not self._paused and self.state == 'NAVIGATING':
    self.state = 'GAP_SELECT'
```

**Lesson:** Asynchronous callbacks must always check whether the system is in a state where acting on their result is valid.

---

### Static-First Gate in Earlier Version

An earlier version of the coordinator required the static target to be fully completed before the dynamic target could be engaged. In environments where the dynamic marker was encountered much earlier, this caused significant wasted exploration time. The final version removes this gate.

**Lesson:** Rigid ordering constraints in a mission FSM should be justified by physical necessity. If the ordering does not matter mechanically, remove the gate.

---

### Inflation Radius Too Small (Mid-Run Fix)

The initial `inflation_radius` of 0.13 m was insufficient for the test environment. The robot was clipping walls because the safety buffer was too narrow for the actual corridor widths being navigated. Increasing to 0.15 m mid-run resolved the wall collision issue.

**Lesson:** Costmap inflation radius should be validated in the actual test environment before the run. What works in simulation or a wide-open space may be too tight in a real corridor.

---

### Phantom Wall Bug (Raytrace Range Mismatch)

Setting `obstacle_max_range` larger than `raytrace_max_range` caused cells to be marked as occupied but never cleared as the robot moved past. These persistent stale cells blocked path planning. Matching both values to 2.5 m eliminated the phantom walls.

**Lesson:** In Nav2 costmaps, `raytrace_max_range` must always be ≥ `obstacle_max_range`. If they differ, the clearing raytrace cannot undo what the marking step wrote.

---

## Tuning Guide

### Navigation Node (`nav_final.py`)

| Parameter | Final Value | Effect | When to Change |
|-----------|-------------|--------|----------------|
| `OPEN_THRESH_M` | 1.00 m | Minimum sector median to be considered open | Lower if valid gaps are being missed; raise if robot tries unsafe gaps |
| `MIN_OPEN_DEG` | 40° | Minimum opening angular width | Lower to allow narrower gaps; raise if robot attempts impossible spaces |
| `OPEN_GOAL_D` | 0.90 m | How far ahead the gap goal is placed | Lower in small rooms; raise in large open spaces |
| `STUCK_DIST` | 0.25 m | Movement threshold for stuck detection | Lower if stuck detection does not trigger; raise if false positives occur |
| `STUCK_TIME` | 3.5 s | Snapshot window for stuck detection | Lower for faster recovery; raise if tight corridors cause false positives |
| `BACKUP_DURATION` | 3.5 s | How long the robot reverses | Raise if robot does not clear the obstacle after backup |
| `BACKUP_REAR_CLEAR` | 0.28 m | Minimum rear clearance before reversing | Lower if robot refuses to back up; raise if it backs into walls |
| `FRONTIER_TO` | 45.0 s | Goal timeout before cancelling | Lower if Nav2 frequently gets stuck; raise in large environments |
| `BLACKLIST_R` | 0.80 m | Rejection radius around blacklisted goals | Lower if too many valid goals are blocked; raise if robot retries failed areas |
| `INFLATION_R` | 0.15 m | Obstacle inflation in internal costmap | Must match `inflation_radius` in Nav2 YAML exactly |

### Mission Coordinator (`mission_coordinator_final.py`)

| Parameter | Final Value | Effect | When to Change |
|-----------|-------------|--------|----------------|
| `DETECTION_RANGE_M` | 1.5 m | Maximum tag range to trigger docking | Lower if robot docks from too far; raise if nearby tags are ignored |
| `DOCK_LOST_DEBOUNCE_S` | 8.0 s | How long tag must be lost before aborting | Lower if robot hangs on a lost tag; raise if noise causes premature aborts |
| `DOCK_TIMEOUT_S` | 45.0 s | Maximum dock+fire cycle duration | Raise if docking takes longer due to slow visual servoing |
| `WAITING_DYNAMIC_TIMEOUT_S` | 30.0 s | How long to wait for receptacle tag | Raise if the dynamic target mechanism moves slowly |
| `DYNAMIC_BALL_COUNT` | 3 | Number of balls to fire at dynamic target | Change to match physical ball count loaded |
| `TAG_STALE_S` | 1.0 s | Age at which a detection is discarded | Lower in fast-moving scenarios; raise if camera publishes infrequently |

### Nav2 YAML (`nav2_params_frontier.yaml`)

| Parameter | Final Value | Effect | Notes |
|-----------|-------------|--------|-------|
| `inflation_radius` | 0.15 m | Obstacle padding in both costmaps | Raised from 0.13 m mid-run due to wall collisions. Must match `INFLATION_R` in `nav_final.py` |
| `cost_scaling_factor` | 5.4 | How steeply inflation cost decays | Raise if robot still hugs walls; lower if paths avoid open corridors |
| `collision_margin_distance` | 0.30 m | How early MPPI penalises near-wall trajectories | Raised from 0.20 m for earlier deflection from walls |
| `raytrace_max_range` | 2.5 m | How far the costmap clears cells | Must be ≥ `obstacle_max_range` to avoid phantom walls |
| `vx_max` | 0.11 m/s | Maximum forward speed for MPPI | Lower if wall collisions persist; raise in open environments |
| `allow_unknown` | true | Whether planner routes through unmapped space | Must be `true` for frontier exploration |

---

## System Launch Architecture

For terminal commands and the full launch order, see the **Laptop Terminals** section in [Developer_Guide.md](Developer_Guide.md). Terminals must be started in order: SLAM → Nav2 → Navigation node → Mission Coordinator. RPi-side nodes are launched separately on the Raspberry Pi with `ROS_DOMAIN_ID=42` set on both machines.

---

## Visualisation and Debugging

For monitoring commands (`ros2 topic echo`), manual pause/resume testing, and `TEST_MODE` usage, see the **Testing guide** section in [Developer_Guide.md](Developer_Guide.md).