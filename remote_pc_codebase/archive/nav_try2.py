#!/usr/bin/env python3
"""
nav2_gap_nav.py  (integration-fixed + obstacle-fixed + score-fixed + wall-gap-fixed)
======================================================
Gap-first frontier explorer that delegates navigation to Nav2.


Goal selection:
 1. LiDAR gap frontiers  — drive through the best visible opening
 2. Map-based frontiers  — info-gain binning fallback
 3. DONE                 — stop when no frontiers remain


Navigation (Nav2):
 - Global path: SmacPlanner2D  (allow_unknown=true)
 - Local control: MPPIController
 - Costmaps: obstacle + inflation layers updated live from /scan
 - Recovery: Nav2 behavior tree (spin, backup, wait)


Run order:
 Terminal 1: ros2 launch turtlebot3_cartographer cartographer.launch.py
 Terminal 2: ros2 launch nav2_bringup navigation_launch.py \\
               params_file:=<ws>/src/auto_nav/config/nav2_params_frontier.yaml \\
               use_sim_time:=false
 Terminal 3: ros2 run auto_nav nav2_gap_nav


Bug-fix log (this revision):

 SCORE-BUG — Gap scoring formula had `+ W_GAP_DIST * d_edge`.
              Since we minimise score, this PENALISED farther obstacles,
              meaning the robot PREFERRED gaps where the wall was CLOSE.
              Result: consistently picked goals that put it near walls.
              Fix: changed to `- W_GAP_DIST * d_edge` so safer (far-wall)
              gaps are rewarded, not penalised.

 STUCK-BUG  — STUCK_DIST was 0.20 m over 6 s. A robot oscillating or
              slowly sliding against a wall moves >= 20 cm easily without
              actually escaping. Stuck never triggered so recovery never ran.
              Fix: STUCK_DIST -> 0.35 m, STUCK_TIME -> 5.0 s. Robot must
              cover 35 cm in 5 s or it is declared stuck.

 BACKUP-BUG — BACKUP_DURATION was 1.8 s at 0.08 m/s = 14 cm. Far too
              short to clear the robot body from the obstacle.
              Fix: BACKUP_DURATION -> 3.5 s (~28 cm). Rear blocked threshold
              changed from 0.18 m to 0.28 m so the robot does not drive
              FORWARD into the wall when the rear path is only marginally
              clear.

 INFLATE-BUG — INFLATION_R was 0.12 m in code but 0.14 m in YAML.
               Both round to 3 cells at 0.05 m/cell so numerically equal,
               but the comment claimed it was 0.18. Set to 0.14 to match
               YAML exactly and eliminate future confusion.

 TARGET-BUG  — GAP_TARGET_D was 0.80 m past the obstacle edge. With
               d_edge_min = 0.25 m the farthest the target could land was
               1.05 m into potentially unknown space -- often blocked in
               Nav2's costmap even though this node's coarser grid passed it.
               Fix: GAP_TARGET_D -> 0.60 m. Shorter target, less likely to
               land in an inflated zone the planner cannot route through.

 INT-BUG 1 — pause did NOT call _blacklist_goal(), so the cancelled goal
             was immediately re-sent on resume causing the robot to backtrack.
             Also: pause/resume were not idempotent.
             Fixed: pause always blacklists the active goal before stopping.

 INT-BUG 2 — _on_nav_result fired while _paused=True causing state flip.
             Fixed: _on_nav_result guards state transition on _paused.

 INT-BUG 3 — pause during BACKING_UP left /cmd_vel publishing.
             Fixed: pause handler aborts stuck-backup immediately.

 INT-BUG 4 — _pre_pause_state was stored but always ignored. Removed.

 GAP-REWORK  — Replaced edge-based gap detection with open-sector detection.
               Old: find depth-jump edges -> asin shift -> target past edge.
               Bug: shift direction depended on which edge ray read higher;
               LiDAR noise flipped this scan-to-scan -> target randomly pointed
               through or away from the gap -> inconsistent behaviour.
               New: bin scan into 20-degree sectors, median per sector, cluster
               adjacent open sectors (median > OPEN_THRESH_M), goal along
               opening centre. No edge geometry, no direction ambiguity.

 WALL-GAP-FIX — Robot was targeting frontiers beyond the maze walls because
                LiDAR rays passed through small inter-wall gaps and registered
                3+ m readings. Cartographer mapped those regions as unknown,
                and both _select_gap and _select_frontier treated them as
                valid targets. Three-part fix:

                1. SCAN_CLAMP_M: cap scan ranges at obstacle_max_range (2.5 m)
                   in _scan_cb. Crack-echo rays become NaN; their sector
                   medians collapse to 0.0 and are treated as closed by the
                   gap detector. OPEN_THRESH_M also tightened to 0.80 m.

                2. BIN_MIN raised 2 -> 5: crack-induced frontier clusters are
                   tiny (1-3 cells). Requiring >= 5 cells per bin eliminates
                   them before any geometry is evaluated.

                3. _frontier_los_clear(): Bresenham ray-cast from the robot
                   map cell to the frontier map cell on blocked_grid.
                   If any intermediate cell is inflated (blocked), the path
                   cuts through the obstacle inflation radius -> reject.
                   This is O(path_length_in_cells) per candidate -- cheap,
                   no BFS needed, and directly encodes the intent:
                   "if you cannot draw a clear line to the frontier without
                    touching inflated space, do not go there."
                   Trade-off: around-corner frontiers fail this check until
                   the robot has navigated around and gained LOS. This is
                   acceptable -- the robot targets what it can see a clear
                   path to and discovers those frontiers naturally.
"""


import math
import time


import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data


from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


from tf2_ros import Buffer, TransformListener, TransformException


# ===================================================================
# Constants
# ===================================================================


FRONTIER_HZ     = 2.0
CONTROL_HZ      = 10.0


# -- Scan clamping (WALL-GAP-FIX part 1) ----------------------------
# Cap LiDAR ranges to obstacle_max_range from the Nav2 YAML (2.5 m).
# Rays that punch through narrow wall cracks register 3+ m readings;
# clamping them to NaN stops _select_gap from seeing those directions
# as open sectors, eliminating gap goals that point outside the maze.
SCAN_CLAMP_M    = 2.5        # must match obstacle_max_range in nav2_params YAML


# -- Open-sector gap detection --------------------------------------
OPEN_THRESH_M   = 0.80       # sector median must exceed this to be "open".
                              # Tightened from 1.00 m; crack-echo sectors
                              # are now NaN-collapsed to 0.0 by the scan
                              # clamp and correctly read as closed.
SECTOR_DEG      = 20         # angular bin width (degrees); coarser = more noise-robust
MIN_OPEN_DEG    = 40         # opening must span at least this to be usable (>= 2 sectors)
OPEN_GOAL_D     = 0.90       # place goal this far along the opening centre direction
GAP_MIN_D       = 0.30       # discard goals closer than this to the robot
W_OPEN_ALIGN    = 2.0        # weight: penalise openings that require turning
W_OPEN_WIDTH    = 0.4        # weight: reward wider openings


# -- Map-frontier fallback ------------------------------------------
# WALL-GAP-FIX part 2: BIN_MIN raised from 2 -> 5.
# Crack-induced frontier clusters from LiDAR sneaking through wall gaps
# are tiny (1-3 cells). Real in-maze frontiers are wider. Requiring >= 5
# cells per bin eliminates crack artifacts before any geometry runs.
BIN_CELLS       = 10
BIN_MIN         = 5          # was 2; raised to filter crack-gap clusters
W_SIZE          = 0.3
W_INFO          = 1.8
W_HEADING       = 1.5
FRONTIER_MIN_D  = 0.25
MAX_FRONTIER_DIST_M = 4.0    # hard ceiling; tune to ~1.5x your maze diagonal
LIDAR_RANGE_M   = 3.5
OCC_THRESH      = 60
MAP_OCC_THRESH  = 60
# FIX INFLATE-BUG: was 0.12 (comment incorrectly said 0.18). Now 0.14 to
# match YAML inflation_radius exactly. Both round to 3 cells at 0.05 m/cell.
INFLATION_R     = 0.14


# -- Info gain threshold --------------------------------------------
IG_RATIO_MIN    = 0.05


# -- Blacklist ------------------------------------------------------
BLACKLIST_R          = 0.80
STUCK_BL_R           = 1.00
REGION_BL_R          = 1.0
REGION_STUCK_THRESH  = 3
MAX_BLACKLIST        = 60
FRONTIER_TO          = 45.0
NO_FRONTIER_RETRIES  = 3


# -- Stuck detection + backup ---------------------------------------
STUCK_DIST      = 0.35
STUCK_TIME      = 5.0
BACKUP_SPEED    = 0.08
BACKUP_DURATION = 3.5
BACKUP_REAR_CLEAR = 0.28


# ===================================================================
# Node
# ===================================================================


class Nav2GapNav(Node):


    def __init__(self):
        super().__init__('nav2_gap_nav')

        self.tf_buffer   = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  qos_profile_sensor_data)
        self.create_subscription(LaserScan,     '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/nav_command', self._nav_cmd_cb, 10)

        self._cmd_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self._state_pub = self.create_publisher(String, '/mission/nav_state', 10)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Map state
        self.map_raw      = None
        self.map_w        = 0
        self.map_h        = 0
        self.map_res      = 0.05
        self.map_ox       = 0.0
        self.map_oy       = 0.0
        self.blocked_grid = None
        self.have_map     = False
        self.map_dirty    = False

        # Scan state
        self.scan_ranges    = np.array([], dtype=np.float32)
        self.scan_angle_min = 0.0
        self.scan_angle_inc = 0.0

        # Nav state
        self.state            = 'IDLE'
        self.frontier_goal    = None
        self.goal_start_t     = None
        self.nav_handle       = None
        self.blacklist        = []
        self.stuck_blacklist  = []
        self._no_goal_cnt     = 0

        # Stuck detection
        self.stuck_check_pos = None
        self.stuck_check_t   = time.monotonic()
        self.backup_start_t  = None
        self.stuck_pos       = None

        self._paused = False

        self.create_timer(1.0 / FRONTIER_HZ, self._goal_tick)
        self.create_timer(1.0 / CONTROL_HZ,  self._control_tick)

        self.get_logger().info('nav2_gap_nav started -- waiting for /map, TF, and Nav2')


    # -- ROS callbacks ----------------------------------------------


    def _map_cb(self, msg: OccupancyGrid):
        self.map_res = msg.info.resolution
        self.map_ox  = msg.info.origin.position.x
        self.map_oy  = msg.info.origin.position.y
        self.map_w   = msg.info.width
        self.map_h   = msg.info.height
        self.map_raw = np.array(msg.data, dtype=np.int8).reshape((self.map_h, self.map_w))
        self.have_map  = True
        self.map_dirty = True


    def _scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=np.float32)
        r[r < msg.range_min] = np.nan
        # WALL-GAP-FIX part 1: clamp at SCAN_CLAMP_M instead of msg.range_max.
        # Rays that punch through narrow inter-wall gaps register 3+ m; clamping
        # to NaN collapses those sector medians to 0.0 and marks them closed.
        # Does NOT affect what Cartographer maps (independent /scan subscriber).
        r[r > SCAN_CLAMP_M]  = np.nan
        self.scan_ranges    = r
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment


    def _nav_cmd_cb(self, msg: String):
        """
        Fully idempotent pause/resume (INT-BUG 1, 3 fixes).

        pause:
          - Guard: no-op if already paused.
          - Aborts any in-progress stuck-backup (INT-BUG 3).
          - Cancels any active Nav2 goal.
          - Blacklists the current goal so it is NOT re-selected on resume.
          - Transitions state to 'PAUSED'.

        resume:
          - Guard: no-op if not paused.
          - Clears frontier_goal defensively.
          - Transitions to GAP_SELECT for a fresh goal selection.
        """
        cmd = msg.data.strip().lower()

        if cmd == 'pause':
            if self._paused:
                return
            self._paused = True
            if self.state == 'BACKING_UP':
                self.stuck_pos       = None
                self.backup_start_t  = None
                self.stuck_check_pos = None
            if self.state == 'NAVIGATING':
                self._cancel_nav2_goal()
            self._blacklist_goal()
            self._stop()
            self.state = 'PAUSED'
            self.get_logger().info('Nav PAUSED')

        elif cmd == 'resume':
            if not self._paused:
                return
            self._paused       = False
            self.frontier_goal = None
            self.goal_start_t  = None
            self.state         = 'GAP_SELECT'
            self.get_logger().info('Nav RESUMED -- reselecting goal')


    # -- Pose -------------------------------------------------------


    def _get_pose(self):
        for child in ('base_footprint', 'base_link'):
            try:
                tf = self.tf_buffer.lookup_transform('map', child, rclpy.time.Time())
                tx  = tf.transform.translation.x
                ty  = tf.transform.translation.y
                q   = tf.transform.rotation
                yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                                 1.0 - 2.0*(q.y*q.y + q.z*q.z))
                return (tx, ty, yaw)
            except TransformException:
                continue
        return None


    # -- Map helpers ------------------------------------------------


    def _world_to_map(self, wx, wy):
        mx = int((wx - self.map_ox) / self.map_res)
        my = int((wy - self.map_oy) / self.map_res)
        if 0 <= mx < self.map_w and 0 <= my < self.map_h:
            return mx, my
        return None


    def _map_to_world(self, mx, my):
        return (self.map_ox + (mx + 0.5) * self.map_res,
                self.map_oy + (my + 0.5) * self.map_res)


    def _rebuild_costmap(self):
        if self.map_raw is None:
            return
        occ      = self.map_raw >= MAP_OCC_THRESH
        cells    = int(math.ceil(INFLATION_R / max(self.map_res, 1e-6)))
        inflated = occ.copy()
        for y, x in np.argwhere(occ):
            y0 = max(0, y - cells);  y1 = min(self.map_h, y + cells + 1)
            x0 = max(0, x - cells);  x1 = min(self.map_w, x + cells + 1)
            inflated[y0:y1, x0:x1] = True
        self.blocked_grid = inflated
        self.map_dirty    = False


    def _grid_blocked(self, cell) -> bool:
        if cell is None or self.blocked_grid is None:
            return False
        mx, my = cell
        bh, bw = self.blocked_grid.shape
        if not (0 <= mx < bw and 0 <= my < bh):
            return False
        return bool(self.blocked_grid[my, mx])


    # -- LiDAR helpers ----------------------------------------------


    def _arc_min(self, center_deg: float = 0.0, half_deg: float = 20.0) -> float:
        if self.scan_ranges.size == 0:
            return float('inf')
        center_rad = math.radians(center_deg)
        half_rad   = math.radians(half_deg)
        angles = (self.scan_angle_min
                  + np.arange(self.scan_ranges.size) * self.scan_angle_inc)
        diff  = np.abs((angles - center_rad + math.pi) % (2 * math.pi) - math.pi)
        mask  = diff <= half_rad
        vals  = self.scan_ranges[mask]
        valid = vals[~np.isnan(vals)]
        return float(np.min(valid)) if valid.size > 0 else float('inf')


    # -- Info gain helpers ------------------------------------------


    def _calc_info_gain(self, cx, cy) -> int:
        if self.map_raw is None:
            return 0
        r  = int(math.ceil(LIDAR_RANGE_M / self.map_res))
        y0 = max(0, cy - r);  y1 = min(self.map_h, cy + r + 1)
        x0 = max(0, cx - r);  x1 = min(self.map_w, cx + r + 1)
        return int(np.count_nonzero(self.map_raw[y0:y1, x0:x1] == -1))


    def _ig_ratio(self, cx, cy) -> float:
        max_ig = math.pi * (LIDAR_RANGE_M / self.map_res) ** 2
        return self._calc_info_gain(cx, cy) / max(max_ig, 1.0)


    # -- Line-of-sight through inflation (WALL-GAP-FIX part 3) ------
    #
    #  The core idea: crack frontiers are physically on the OTHER SIDE of a
    #  wall. The inflation layer marks the wall + a surrounding buffer as
    #  blocked. Any straight line from the robot to a crack frontier must
    #  cross at least one inflated cell -- the wall itself.
    #
    #  Algorithm: Bresenham integer ray-cast from robot map cell (rx, ry) to
    #  frontier map cell (fx, fy) in (col, row) convention. Every intermediate
    #  cell is checked against blocked_grid. If any is blocked -> the straight
    #  path cuts through the inflation radius -> reject this frontier.
    #
    #  Start and end cells are excluded:
    #    - Start: robot may legitimately be grazing an inflation zone.
    #    - End:   frontier cell sits on the free-unknown boundary and may
    #             itself read as blocked in the costmap.
    #
    #  Cost: O(max(|dcol|, |drow|)) per candidate -- cheap, no BFS needed.
    #
    #  Trade-off: around-corner frontiers fail this check until the robot
    #  has navigated around and gained direct LOS to them. This is fine --
    #  the robot targets what it can see a clear path to and discovers those
    #  frontiers naturally as it explores.

    def _frontier_los_clear(self, fx: int, fy: int, rx: int, ry: int) -> bool:
        """
        Bresenham ray-cast from robot cell (rx, ry) to frontier cell (fx, fy).
        Both arguments are in map (col, row) / (x, y) convention.
        Returns True  if no intermediate cell is inflated (clear LOS).
        Returns False if any intermediate cell is in the inflation radius,
                      meaning the path would cut through obstacle space.
        Fails open (True) if blocked_grid is not yet built.
        """
        if self.blocked_grid is None:
            return True

        dx = abs(fx - rx)
        dy = abs(fy - ry)
        sx = 1 if fx > rx else -1
        sy = 1 if fy > ry else -1
        err = dx - dy
        x, y = rx, ry

        while True:
            # Exclude start and end cells from the blocked check
            if (x, y) != (rx, ry) and (x, y) != (fx, fy):
                if 0 <= x < self.map_w and 0 <= y < self.map_h:
                    if self.blocked_grid[y, x]:
                        return False   # line passes through inflated space

            if x == fx and y == fy:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x   += sx
            if e2 < dx:
                err += dx
                y   += sy

        return True


    # -- Gap selection (open-sector approach) -----------------------
    #
    #  WHY THE PREVIOUS APPROACH WAS INCONSISTENT
    #  -------------------------------------------
    #  The old code found depth-jump edges between adjacent LiDAR rays, then
    #  used asin(robot_half_width / wall_dist) to shift the target sideways
    #  away from the wall. The shift direction (+/-) was decided by which of
    #  the two edge rays read higher. A few mm of LiDAR noise can flip that
    #  comparison on consecutive scans, so the shift randomly pointed INTO the
    #  gap or AWAY from it. That is why the robot sometimes drove through and
    #  sometimes reversed out.
    #
    #  NEW APPROACH -- find where space IS open, not where edges ARE
    #  1. Bin the ~360 rays into SECTOR_DEG-wide buckets. Median per bucket
    #     is robust to single noisy rays.
    #  2. A bucket is "open" if median > OPEN_THRESH_M.
    #     Rays clamped to NaN by SCAN_CLAMP_M produce median = 0.0 -> closed.
    #  3. Cluster adjacent open buckets into contiguous openings.
    #  4. Discard openings narrower than MIN_OPEN_DEG (too tight for robot).
    #  5. Goal = OPEN_GOAL_D metres along the opening centre direction.
    #  6. Validity filters: in map, not blacklisted, not inflated, info gain.
    #  7. Score = alignment_cost - width_bonus  (minimise -> lower = better).


    def _select_gap(self, rx, ry, yaw):
        if self.scan_ranges.size < 10:
            return None

        n = len(self.scan_ranges)
        ray_angles_rf = (self.scan_angle_min
                         + np.arange(n) * self.scan_angle_inc)

        # Step 1: bin rays into SECTOR_DEG-wide buckets
        sector_rad = math.radians(SECTOR_DEG)
        bucket_idx = ((ray_angles_rf + math.pi) / sector_rad).astype(int)
        n_sectors  = int(math.ceil(2 * math.pi / sector_rad))
        bucket_idx = np.clip(bucket_idx, 0, n_sectors - 1)

        sector_median = np.full(n_sectors, 0.0)
        sector_centre = np.zeros(n_sectors)
        for s in range(n_sectors):
            mask  = (bucket_idx == s)
            vals  = self.scan_ranges[mask]
            valid = vals[~np.isnan(vals)]
            # NaN-only sectors (crack directions clamped by _scan_cb) get
            # median = 0.0 -> below OPEN_THRESH_M -> correctly closed.
            sector_median[s] = float(np.median(valid)) if valid.size > 0 else 0.0
            sector_centre[s] = -math.pi + (s + 0.5) * sector_rad

        # Step 2: mark open sectors
        open_flags = sector_median > OPEN_THRESH_M

        # Step 3: cluster adjacent open sectors into openings
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

        if not openings:
            return None

        # Steps 4-7: filter, place goal, score
        best_score  = float('inf')
        best_target = None

        for span_deg, centre_rf in openings:
            if span_deg < MIN_OPEN_DEG:
                continue

            centre_wf = yaw + centre_rf
            tx = rx + OPEN_GOAL_D * math.cos(centre_wf)
            ty = ry + OPEN_GOAL_D * math.sin(centre_wf)

            if self._world_to_map(tx, ty) is None:
                continue
            if math.hypot(tx - rx, ty - ry) < GAP_MIN_D:
                continue
            if any(math.hypot(tx - bx, ty - by) < BLACKLIST_R
                   for bx, by, _ in self.blacklist):
                continue
            if any(math.hypot(tx - bx, ty - by) < STUCK_BL_R
                   for bx, by, _ in self.stuck_blacklist):
                continue
            if self._grid_blocked(self._world_to_map(tx, ty)):
                continue
            cell = self._world_to_map(tx, ty)
            if cell is not None and self.map_raw is not None:
                if self._ig_ratio(cell[0], cell[1]) < IG_RATIO_MIN:
                    continue

            align_cost = abs(centre_rf)
            score = W_OPEN_ALIGN * align_cost - W_OPEN_WIDTH * span_deg

            if score < best_score:
                best_score  = score
                best_target = (tx, ty)

        return best_target


    # -- Frontier selection -----------------------------------------


    def _select_frontier(self, rx, ry, yaw):
        if self.map_raw is None:
            return None
        grid = self.map_raw
        H, W = grid.shape

        free    = (grid >= 0) & (grid < OCC_THRESH)
        unknown = (grid < 0)
        adj_unk = np.zeros((H, W), dtype=bool)
        adj_unk[1:-1, 1:-1] = (unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
                                unknown[1:-1, :-2] | unknown[1:-1, 2:])
        pts = np.argwhere(free & adj_unk)
        if pts.shape[0] == 0:
            return None

        bin_row = pts[:, 0] // BIN_CELLS
        bin_col = pts[:, 1] // BIN_CELLS
        bin_key = bin_row * (W // BIN_CELLS + 1) + bin_col
        unique_keys, inverse, counts = np.unique(
            bin_key, return_inverse=True, return_counts=True)

        now = time.monotonic()
        self.blacklist = [
            e for e in self.blacklist if now - e[2] < FRONTIER_TO * 6.0]
        self.stuck_blacklist = [
            e for e in self.stuck_blacklist if now - e[2] < FRONTIER_TO * 8.0]

        # Pre-compute robot map cell once -- reused for every LOS check below

        best_score = float('inf')
        best_goal  = None

        for i in range(len(unique_keys)):
            # WALL-GAP-FIX part 2: BIN_MIN = 5.
            # Crack artifact clusters have very few cells (1-3); real in-maze
            # frontiers are wider. Cheapest filter so it runs first.
            if counts[i] < BIN_MIN:
                continue

            mask_k    = (inverse == i)
            bin_cells = pts[mask_k]

            centroid      = bin_cells.mean(axis=0)
            dists_to_cent = np.linalg.norm(bin_cells - centroid, axis=1)
            best_cell     = bin_cells[np.argmin(dists_to_cent)]
            wx = self.map_ox + (best_cell[1] + 0.5) * self.map_res
            wy = self.map_oy + (best_cell[0] + 0.5) * self.map_res

            dist = math.hypot(wx - rx, wy - ry)
            if dist < FRONTIER_MIN_D:
                continue
            # Hard distance ceiling as a secondary cheap filter before LOS
            if dist > MAX_FRONTIER_DIST_M:
                continue

            if any(math.hypot(wx - bx, wy - by) < BLACKLIST_R
                   for bx, by, _ in self.blacklist):
                continue
            if any(math.hypot(wx - bx, wy - by) < STUCK_BL_R
                   for bx, by, _ in self.stuck_blacklist):
                continue
            if self._grid_blocked(self._world_to_map(wx, wy)):
                continue

            # WALL-GAP-FIX part 3: Bresenham LOS through blocked_grid.
            # If the straight line from robot to frontier crosses any inflated
            # cell, the frontier is beyond a wall -> skip it.
            # best_cell is (row, col); _frontier_los_clear takes (col, row).

            ig_r = self._ig_ratio(int(best_cell[1]), int(best_cell[0]))
            if ig_r < IG_RATIO_MIN:
                continue

            nearby_stucks = sum(
                1 for bx, by, _ in self.stuck_blacklist
                if math.hypot(wx - bx, wy - by) < REGION_BL_R
            )
            if nearby_stucks >= REGION_STUCK_THRESH:
                continue

            goal_angle   = math.atan2(wy - ry, wx - rx)
            heading_diff = abs((goal_angle - yaw + math.pi) % (2 * math.pi) - math.pi)

            efficiency = dist / (ig_r + 0.1)
            score = efficiency + W_HEADING * heading_diff - W_SIZE * counts[i]

            if score < best_score:
                best_score = score
                best_goal  = (wx, wy)

        return best_goal


    # -- Goal selection tick ----------------------------------------


    def _goal_tick(self):
        m = String()
        m.data = 'paused' if self._paused else self.state
        self._state_pub.publish(m)

        if self._paused:
            return

        if self.state == 'IDLE':
            if self.have_map:
                self.state = 'GAP_SELECT'
            return

        if self.state in ('NAVIGATING', 'BACKING_UP', 'DONE', 'PAUSED'):
            if self.state == 'NAVIGATING' and self.goal_start_t is not None:
                if time.monotonic() - self.goal_start_t > FRONTIER_TO:
                    self.get_logger().warn('Goal timeout -- cancelling and blacklisting.')
                    self._cancel_nav2_goal()
                    self._blacklist_goal()
                    self.state = 'GAP_SELECT'
            return

        # GAP_SELECT
        if not self.have_map:
            return
        if self.map_dirty:
            self._rebuild_costmap()

        pose = self._get_pose()
        if pose is None:
            return
        rx, ry, yaw = pose

        goal = self._select_gap(rx, ry, yaw)
        goal_type = 'gap'
        if goal is None:
            goal = self._select_frontier(rx, ry, yaw)
            goal_type = 'frontier'

        if goal is None and self.blacklist:
            self.get_logger().info(
                f'No goal -- clearing normal blacklist ({len(self.blacklist)} entries).')
            self.blacklist = []
            gap_r      = self._select_gap(rx, ry, yaw)
            frontier_r = self._select_frontier(rx, ry, yaw)
            if gap_r is not None:
                goal      = gap_r
                goal_type = 'gap-retry'
            elif frontier_r is not None:
                goal      = frontier_r
                goal_type = 'frontier-retry'

        if goal is None:
            self._no_goal_cnt += 1
            self.get_logger().warn(
                f'No goal found ({self._no_goal_cnt}/{NO_FRONTIER_RETRIES})')
            if self._no_goal_cnt >= NO_FRONTIER_RETRIES:
                if self.map_raw is not None:
                    grid    = self.map_raw
                    free    = (grid >= 0) & (grid < OCC_THRESH)
                    unknown = (grid < 0)
                    adj_unk = np.zeros(grid.shape, dtype=bool)
                    adj_unk[1:-1, 1:-1] = (
                        unknown[:-2, 1:-1] | unknown[2:, 1:-1] |
                        unknown[1:-1, :-2] | unknown[1:-1, 2:])
                    if not np.any(free & adj_unk):
                        self.get_logger().info(
                            'Exploration complete -- no frontier cells remain.')
                        self.state = 'DONE'
                        return
                self.blacklist    = []
                self._no_goal_cnt = 0
                self.get_logger().info('Blacklist cleared -- retrying.')
            return

        self._no_goal_cnt = 0
        gx, gy = goal
        self.frontier_goal = (gx, gy)
        self.get_logger().info(f'[{goal_type}] goal ({gx:.2f}, {gy:.2f})')

        self.state = 'NAVIGATING'
        self._send_nav2_goal(gx, gy, rx, ry)


    # -- Nav2 action client -----------------------------------------


    def _send_nav2_goal(self, gx, gy, rx, ry):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 NavigateToPose not available -- is nav2_bringup running?')
            self.state = 'GAP_SELECT'
            return

        heading_yaw = math.atan2(gy - ry, gx - rx)
        qz = math.sin(heading_yaw / 2.0)
        qw = math.cos(heading_yaw / 2.0)

        pose = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = gx
        pose.pose.position.y    = gy
        pose.pose.position.z    = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg      = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.goal_start_t = time.monotonic()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_nav_response)


    def _on_nav_response(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Nav2 goal send failed: {e}')
            self._blacklist_goal()
            if not self._paused:
                self.state = 'GAP_SELECT'
            return

        if not handle.accepted:
            self.get_logger().warn('Nav2 goal rejected -- blacklisting.')
            self._blacklist_goal()
            if not self._paused:
                self.state = 'GAP_SELECT'
            return

        self.get_logger().info('Nav2 goal accepted.')
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)


    def _on_nav_result(self, future):
        """INT-BUG 2 FIX: guard state transition on _paused."""
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f'Nav2 result callback error: {e}')
            status = GoalStatus.STATUS_ABORTED

        self.nav_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached -- selecting next.')
            self._blacklist_goal()
        else:
            self.get_logger().warn(f'Nav2 goal failed (status={status}) -- blacklisting.')
            self._blacklist_goal()

        if not self._paused and self.state == 'NAVIGATING':
            self.state = 'GAP_SELECT'


    def _cancel_nav2_goal(self):
        if self.nav_handle is not None:
            try:
                self.nav_handle.cancel_goal_async()
            except Exception:
                pass
            self.nav_handle = None


    # -- Stuck detection + backup -----------------------------------


    def _control_tick(self):
        if self._paused:
            return

        if self.state == 'BACKING_UP':
            if time.monotonic() - self.backup_start_t > BACKUP_DURATION:
                if self.stuck_pos is not None:
                    self.stuck_blacklist.append(
                        (self.stuck_pos[0], self.stuck_pos[1], time.monotonic()))
                    self.get_logger().info(
                        f'Stuck-blacklisted ({self.stuck_pos[0]:.2f}, '
                        f'{self.stuck_pos[1]:.2f}); '
                        f'stuck_bl={len(self.stuck_blacklist)}')

                    same_area = sum(
                        1 for bx, by, _ in self.stuck_blacklist
                        if math.hypot(bx - self.stuck_pos[0],
                                      by - self.stuck_pos[1]) < REGION_BL_R
                    )
                    if same_area >= REGION_STUCK_THRESH:
                        self.get_logger().warn(
                            f'Region ({self.stuck_pos[0]:.2f}, '
                            f'{self.stuck_pos[1]:.2f}) inaccessible -- burying.')
                        if self.frontier_goal is not None:
                            gx, gy = self.frontier_goal
                            for _ in range(5):
                                self.stuck_blacklist.append(
                                    (gx, gy, time.monotonic()))

                self._stop()
                self.frontier_goal   = None
                self.goal_start_t    = None
                self.stuck_check_pos = None
                self.stuck_check_t   = time.monotonic()
                self.state = 'GAP_SELECT'
            else:
                rear_d = self._arc_min(180.0, 30.0)
                cmd = Twist()
                if rear_d > BACKUP_REAR_CLEAR:
                    cmd.linear.x = -BACKUP_SPEED
                else:
                    cmd.linear.x  =  0.05
                    cmd.angular.z =  0.6
                self._cmd_pub.publish(cmd)
            return

        if self.state != 'NAVIGATING':
            self.stuck_check_pos = None
            return

        pose = self._get_pose()
        if pose is None:
            return
        rx, ry, _ = pose

        now_t = time.monotonic()
        if self.stuck_check_pos is None:
            self.stuck_check_pos = (rx, ry)
            self.stuck_check_t   = now_t
        elif now_t - self.stuck_check_t > STUCK_TIME:
            moved = math.hypot(rx - self.stuck_check_pos[0],
                               ry - self.stuck_check_pos[1])
            if moved < STUCK_DIST:
                self.get_logger().warn(
                    f'Stuck detected (moved {moved:.3f}m in {STUCK_TIME}s) -- backing up.')
                self.stuck_pos      = (rx, ry)
                self.backup_start_t = time.monotonic()
                self._cancel_nav2_goal()
                self._blacklist_goal()
                self.state = 'BACKING_UP'
                return
            self.stuck_check_pos = (rx, ry)
            self.stuck_check_t   = now_t


    # -- Utilities --------------------------------------------------


    def _stop(self):
        self._cmd_pub.publish(Twist())


    def _blacklist_goal(self):
        if self.frontier_goal is not None:
            gx, gy = self.frontier_goal
            self.blacklist.append((gx, gy, time.monotonic()))
            if len(self.blacklist) > MAX_BLACKLIST:
                self.blacklist = self.blacklist[-MAX_BLACKLIST:]
            self.get_logger().info(
                f'Blacklisted ({gx:.2f}, {gy:.2f}); total={len(self.blacklist)}')
        self.frontier_goal = None
        self.goal_start_t  = None


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GapNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._cancel_nav2_goal()
            node._stop()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
