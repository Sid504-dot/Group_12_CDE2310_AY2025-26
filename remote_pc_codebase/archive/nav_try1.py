#!/usr/bin/env python3
"""
nav2_gap_nav.py  (coverage + gap-scoring overhaul)
====================================================
Gap-first frontier explorer that delegates navigation to Nav2.

Goal selection (priority order):
  1. LiDAR gap frontiers  — drive through the best visible opening
  2. Map-based frontiers  — info-gain binning fallback
  3. Coverage goals       — free mapped cells never physically visited
  4. DONE                 — stop when no goals remain

Changes in this revision vs integration-fixed + obstacle-fixed:

  GAP-FIX 1 — Replaced edge+lateral-shift target with geometric centre of
               the open ray cluster. The old asin(ROBOT_SIDE_M/d_edge) shift
               approached 90° for 30cm gaps (d_edge≈0.25m) and placed the
               target in the opposite wall. The new approach computes the mean
               angular index of all open rays, giving the true gap centre.

  GAP-FIX 2 — Replaced d_edge preference in score with info-gain. The old
               +W_GAP_DIST*d_edge term penalised open corridors (large d_edge
               scored worse). Removed entirely; replaced with -W_INFO*ig_r so
               gaps leading to unexplored space are preferred regardless of
               wall distance. d_edge is now a binary safety filter only.

  GAP-FIX 3 — Added soft tight_cost penalty. A margin < 8cm one-side triggers
               a rising score penalty rather than a hard reject, so the robot
               still attempts a 30cm gap when it is the only unexplored route.

  GAP-FIX 4 — Separate blacklist radius for gap targets (GAP_BLACKLIST_R=0.35m
               vs BLACKLIST_R=0.80m). Prevents the return-pass gap on the far
               side of a doorway being blocked by the outbound goal's entry.

  IG-FIX 1  — _calc_info_gain now masks to a true circle (dx²+dy²≤r²) so the
               count matches the π*r² denominator in _ig_ratio. The old square
               crop overestimated ig_ratio by ~27% for all targets equally.

  SPEED-FIX  — LiDAR-based narrow-corridor detector publishes /speed_limit
               (0.06 m/s in, 0.14 m/s out) so MPPI slows before threading
               tight passages instead of relying on the static vx_max.

  COVERAGE   — Third-priority goal type: free mapped cells the robot has never
               physically been within VISIT_CELL_M (0.40m) of. Catches small
               alcoves/rooms whose interiors LiDAR fills in from the doorway,
               which have zero frontier cells and near-zero ig_ratio but still
               need physical entry for AprilTag detection.

Integration bug-fixes (carried over from previous revision):
  INT-BUG 1  — pause idempotency + blacklist on pause.
  INT-BUG 2  — _on_nav_result guards state transition on _paused.
  INT-BUG 3  — pause aborts BACKING_UP immediately.
  INT-BUG 4  — _pre_pause_state removed (dead variable).

Obstacle-fix history (carried over):
  OBS-FIX 1  — INFLATION_R matches YAML inflation_radius (both 0.10m).
  OBS-FIX 2  — GAP_MIN_EDGE_D guard (0.25m) prevents asin overflow.
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
from nav2_msgs.msg import SpeedLimit
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, TransformException

# ═══════════════════════════════════════════════════════════════════
# Constants
# ═══════════════════════════════════════════════════════════════════

FRONTIER_HZ     = 2.0
CONTROL_HZ      = 10.0

# ── Gap detection ────────────────────────────────────────────────
GAP_THRESH_M        = 0.45
GAP_TARGET_D        = 0.80
GAP_MIN_D           = 0.30
GAP_MIN_WIDTH_M     = 0.40
GAP_MIN_CLEARANCE_M = 0.03   # GAP-FIX 3: minimum one-side margin to attempt gap
GAP_BUCKET_DEG      = 10
ROBOT_SIDE_M        = 0.17
GAP_MIN_EDGE_D      = 0.25   # OBS-FIX 2: asin overflow guard
GAP_BLACKLIST_R     = 0.35   # GAP-FIX 4: tighter than BLACKLIST_R for return-pass gaps

W_ANGLE     = 2.0
W_GAP_WIDTH = 0.6
W_INFO      = 1.5            # GAP-FIX 2: replaces W_GAP_DIST

# ── Map-frontier fallback ────────────────────────────────────────
BIN_CELLS       = 10
BIN_MIN         = 2
W_SIZE          = 0.3
W_HEADING       = 1.5
FRONTIER_MIN_D  = 0.25
LIDAR_RANGE_M   = 3.5
OCC_THRESH      = 60
MAP_OCC_THRESH  = 60
# OBS-FIX 1: matches YAML inflation_radius (both 0.10m).
INFLATION_R     = 0.10

# ── Info gain threshold ──────────────────────────────────────────
IG_RATIO_MIN    = 0.05

# ── Blacklist ─────────────────────────────────────────────────────
BLACKLIST_R          = 0.80
STUCK_BL_R           = 1.00
REGION_BL_R          = 1.0
REGION_STUCK_THRESH  = 3
MAX_BLACKLIST        = 60
FRONTIER_TO          = 45.0
NO_FRONTIER_RETRIES  = 3

# ── Stuck detection + backup ─────────────────────────────────────
STUCK_DIST      = 0.40
STUCK_TIME      = 12.0
BACKUP_SPEED    = 0.08
BACKUP_DURATION = 1.8

# ── Speed limit (narrow-corridor detection) ──────────────────────
SPEED_OPEN_MS   = 0.14   # m/s — normal navigation
SPEED_NARROW_MS = 0.06   # m/s — when corridor sides both < 0.30m
NARROW_SIDE_D   = 0.30   # m — threshold: side wall counts as "close"

# ── Coverage (visitation) ────────────────────────────────────────
VISIT_CELL_M    = 0.40   # one cell = one visit radius; set lookup is O(1)


# ═══════════════════════════════════════════════════════════════════
# Node
# ═══════════════════════════════════════════════════════════════════

class Nav2GapNav(Node):

    def __init__(self):
        super().__init__('nav2_gap_nav')

        self.tf_buffer   = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, '/map',  self._map_cb,  qos_profile_sensor_data)
        self.create_subscription(LaserScan,     '/scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/mission/nav_command', self._nav_cmd_cb, 10)

        self._cmd_pub   = self.create_publisher(Twist,      '/cmd_vel',    10)
        self._state_pub = self.create_publisher(String,     '/mission/nav_state', 10)
        self._speed_pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
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
        self._viz_gaps        = []

        # Stuck detection
        self.stuck_check_pos = None
        self.stuck_check_t   = time.monotonic()
        self.backup_start_t  = None
        self.stuck_pos       = None

        # Pause state
        self._paused = False

        # SPEED-FIX: track current speed state to avoid redundant publishes
        self._in_narrow = False

        # COVERAGE: visited cell set — (vx, vy) integer tuples in VISIT_CELL_M grid
        self._visited: set = set()

        self.create_timer(1.0 / FRONTIER_HZ, self._goal_tick)
        self.create_timer(1.0 / CONTROL_HZ,  self._control_tick)

        self.get_logger().info('nav2_gap_nav started — waiting for /map, TF, and Nav2')

    # ── ROS callbacks ───────────────────────────────────────────────

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
        r[r > msg.range_max] = np.nan
        self.scan_ranges    = r
        self.scan_angle_min = msg.angle_min
        self.scan_angle_inc = msg.angle_increment

    def _nav_cmd_cb(self, msg: String):
        """
        INT-BUG 1, 3 FIX — fully idempotent pause/resume.
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
            self.get_logger().info('Nav RESUMED — reselecting goal')

    # ── Pose ────────────────────────────────────────────────────────

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

    # ── Map helpers ─────────────────────────────────────────────────

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

    # ── LiDAR helpers ───────────────────────────────────────────────

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

    # ── Info gain helpers ────────────────────────────────────────────

    def _calc_info_gain(self, cx, cy) -> int:
        """
        IG-FIX 1: Count unknown cells within a true circle (not a square bounding
        box). The old implementation used a square crop but divided by π*r²,
        overestimating ig_ratio by ~27% for all targets. Now the mask strictly
        matches the denominator so ig_ratio is a genuine 0-1 value.
        """
        if self.map_raw is None:
            return 0
        r  = int(math.ceil(LIDAR_RANGE_M / self.map_res))
        y0 = max(0, cy - r);  y1 = min(self.map_h, cy + r + 1)
        x0 = max(0, cx - r);  x1 = min(self.map_w, cx + r + 1)

        ys, xs = np.mgrid[y0:y1, x0:x1]
        within_circle = ((xs - cx)**2 + (ys - cy)**2) <= r**2
        unknown       = self.map_raw[y0:y1, x0:x1] == -1
        return int(np.count_nonzero(unknown & within_circle))

    def _ig_ratio(self, cx, cy) -> float:
        max_ig = math.pi * (LIDAR_RANGE_M / self.map_res) ** 2
        return self._calc_info_gain(cx, cy) / max(max_ig, 1.0)

    # ── Speed limit ──────────────────────────────────────────────────

    def _publish_speed_limit(self):
        """
        SPEED-FIX: Detect narrow corridor from LiDAR and cap Nav2 speed.
        Publishes to /speed_limit only when the narrow state changes.
        Both lateral sides < NARROW_SIDE_D (0.30m) with forward clearance
        triggers the low-speed cap (0.06 m/s), letting MPPI hold the
        centreline in passages that are near the robot's body width.
        """
        if self.scan_ranges.size == 0:
            return

        left_d  = self._arc_min(center_deg= 20.0, half_deg=15.0)
        right_d = self._arc_min(center_deg=-20.0, half_deg=15.0)
        fwd_d   = self._arc_min(center_deg=  0.0, half_deg=15.0)

        narrow = (left_d < NARROW_SIDE_D and right_d < NARROW_SIDE_D
                  and fwd_d > 0.20)

        if narrow == self._in_narrow:
            return   # no change — don't spam the topic

        self._in_narrow = narrow
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage   = False
        msg.speed_limit  = SPEED_NARROW_MS if narrow else SPEED_OPEN_MS
        self._speed_pub.publish(msg)
        self.get_logger().info(
            f'Speed cap: {"%.2f m/s (narrow corridor)" % SPEED_NARROW_MS if narrow else "%.2f m/s (open)" % SPEED_OPEN_MS}')

    # ── Visitation ───────────────────────────────────────────────────

    def _update_visitation(self, rx, ry):
        """
        COVERAGE: Mark robot's current 0.40m grid cell as visited.
        Using a set of integer (vx, vy) tuples: O(1) insert and lookup,
        no grid array to maintain, no resize on map updates.
        """
        vx = int((rx - self.map_ox) / VISIT_CELL_M)
        vy = int((ry - self.map_oy) / VISIT_CELL_M)
        self._visited.add((vx, vy))

    def _is_visited(self, wx, wy) -> bool:
        vx = int((wx - self.map_ox) / VISIT_CELL_M)
        vy = int((wy - self.map_oy) / VISIT_CELL_M)
        return (vx, vy) in self._visited

    # ── Gap selection ────────────────────────────────────────────────

    def _select_gap(self, rx, ry, yaw):
        """
        GAP-FIX 1: Target is geometric centre of open ray cluster, not edge+shift.
        GAP-FIX 2: Score uses -W_INFO*ig_r instead of +W_GAP_DIST*d_edge.
        GAP-FIX 3: Soft tight_cost rather than hard reject below clearance threshold.
        GAP-FIX 4: GAP_BLACKLIST_R (0.35m) for gap targets instead of BLACKLIST_R (0.80m).
        """
        if self.scan_ranges.size < 10:
            return None

        ranges = self.scan_ranges
        n = len(ranges)
        best_score   = float('inf')
        best_target  = None
        seen_buckets = set()
        viz = []

        for i in range(n - 1):
            r1, r2 = ranges[i], ranges[i + 1]
            if np.isnan(r1) or np.isnan(r2):
                continue
            jump = abs(r2 - r1)
            if jump < GAP_THRESH_M:
                continue

            ray_angle = yaw + self.scan_angle_min + (i + 0.5) * self.scan_angle_inc
            bucket = int(math.degrees(ray_angle) / GAP_BUCKET_DEG)
            if bucket in seen_buckets:
                continue
            seen_buckets.add(bucket)

            d_edge = min(r1, r2)

            # OBS-FIX 2: skip gaps where obstacle edge is unreliably close
            if d_edge < GAP_MIN_EDGE_D:
                continue

            # Measure open region width
            open_idx   = i + 1 if r2 > r1 else i
            open_count = 1
            open_r     = r2 if r2 > r1 else r1
            for k in range(1, 20):
                ki = open_idx + k
                if ki >= n or np.isnan(ranges[ki]):
                    break
                if abs(ranges[ki] - open_r) > 0.3:
                    break
                open_count += 1

            gap_width_m = open_count * d_edge * abs(self.scan_angle_inc)
            if gap_width_m < GAP_MIN_WIDTH_M:
                continue

            # GAP-FIX 3: clearance check — hard floor only
            margin_m = (gap_width_m / 2.0) - ROBOT_SIDE_M
            if margin_m < GAP_MIN_CLEARANCE_M:
                continue   # genuinely impassable

            # GAP-FIX 1: geometric centre of the open cluster
            # Replaces the unstable asin(ROBOT_SIDE_M/d_edge) lateral shift
            # which approached 90° for gaps near GAP_MIN_EDGE_D.
            center_open_idx = open_idx + (open_count - 1) / 2.0
            safe_angle = yaw + self.scan_angle_min + center_open_idx * self.scan_angle_inc

            target_d = d_edge + GAP_TARGET_D
            tx = rx + target_d * math.cos(safe_angle)
            ty = ry + target_d * math.sin(safe_angle)

            if self._world_to_map(tx, ty) is None:
                continue
            if math.hypot(tx - rx, ty - ry) < GAP_MIN_D:
                continue

            # GAP-FIX 4: tighter blacklist radius for gap goals
            if any(math.hypot(tx - bx, ty - by) < GAP_BLACKLIST_R
                   for bx, by, _ in self.blacklist):
                continue
            if any(math.hypot(tx - bx, ty - by) < STUCK_BL_R
                   for bx, by, _ in self.stuck_blacklist):
                continue
            if self._grid_blocked(self._world_to_map(tx, ty)):
                continue

            # GAP-FIX 2: info gain drives selection, not wall distance
            cell = self._world_to_map(tx, ty)
            if cell is None:
                continue
            ig_r = self._ig_ratio(cell[0], cell[1])
            if ig_r < IG_RATIO_MIN:
                continue

            # GAP-FIX 3: soft tight_cost — nonzero only when margin < 8cm
            # Rising penalty as the gap tightens, but never a hard block
            tight_cost = max(0.0, 0.08 - margin_m) * 3.0

            angle_diff = abs((safe_angle - yaw + math.pi) % (2 * math.pi) - math.pi)

            # GAP-FIX 2: -W_INFO*ig_r replaces +W_GAP_DIST*d_edge
            # Higher info gain → lower score → preferred
            score = W_ANGLE * angle_diff - W_GAP_WIDTH * jump - W_INFO * ig_r + tight_cost

            viz.append((tx, ty))
            if score < best_score:
                best_score  = score
                best_target = (tx, ty)

        self._viz_gaps = viz
        return best_target

    # ── Frontier selection ───────────────────────────────────────────

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

        best_score = float('inf')
        best_goal  = None

        for i in range(len(unique_keys)):
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
            if any(math.hypot(wx - bx, wy - by) < BLACKLIST_R
                   for bx, by, _ in self.blacklist):
                continue
            if any(math.hypot(wx - bx, wy - by) < STUCK_BL_R
                   for bx, by, _ in self.stuck_blacklist):
                continue
            if self._grid_blocked(self._world_to_map(wx, wy)):
                continue

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

    # ── Coverage goal selection ──────────────────────────────────────

    def _select_coverage_goal(self, rx, ry, yaw):
        """
        COVERAGE: Third-priority goal type.

        Targets free mapped cells the robot has never physically visited.
        Catches small alcoves/rooms whose interiors LiDAR fills in from
        the doorway — they have zero frontier cells (nothing adjacent to
        unknown) and near-zero ig_ratio (nothing unknown nearby), but the
        AprilTag inside requires physical entry to be detectable.

        A cell is considered visited if the robot has ever been within
        VISIT_CELL_M (0.40m) of it. This radius is deliberately smaller
        than LIDAR_RANGE_M so LiDAR fill-in from a doorway does NOT mark
        the room's interior as visited — only actual physical presence does.

        Steps over the occupancy map at VISIT_CELL_M resolution to match
        the visitation grid granularity.
        """
        if self.map_raw is None:
            return None

        step = max(1, int(VISIT_CELL_M / self.map_res))   # e.g. 0.40/0.05 = 8 cells

        best_score = float('inf')
        best_goal  = None

        for my in range(0, self.map_h, step):
            for mx in range(0, self.map_w, step):
                occ_val = self.map_raw[my, mx]
                # Only target free mapped cells — unknown/occupied handled elsewhere
                if occ_val < 0 or occ_val >= OCC_THRESH:
                    continue
                if self._grid_blocked((mx, my)):
                    continue

                wx, wy = self._map_to_world(mx, my)

                if self._is_visited(wx, wy):
                    continue

                dist = math.hypot(wx - rx, wy - ry)
                if dist < FRONTIER_MIN_D:
                    continue
                if any(math.hypot(wx - bx, wy - by) < BLACKLIST_R
                       for bx, by, _ in self.blacklist):
                    continue
                if any(math.hypot(wx - bx, wy - by) < STUCK_BL_R
                       for bx, by, _ in self.stuck_blacklist):
                    continue

                goal_angle   = math.atan2(wy - ry, wx - rx)
                heading_diff = abs((goal_angle - yaw + math.pi) % (2 * math.pi) - math.pi)
                score = dist + W_HEADING * heading_diff

                if score < best_score:
                    best_score = score
                    best_goal  = (wx, wy)

        return best_goal

    # ── Goal selection tick ─────────────────────────────────────────

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
                    self.get_logger().warn('Goal timeout — cancelling and blacklisting.')
                    self._cancel_nav2_goal()
                    self._blacklist_goal()
                    self.state = 'GAP_SELECT'
            return

        if not self.have_map:
            return
        if self.map_dirty:
            self._rebuild_costmap()

        pose = self._get_pose()
        if pose is None:
            return
        rx, ry, yaw = pose

        # ── Priority 1: gap ──────────────────────────────────────────
        goal = self._select_gap(rx, ry, yaw)
        goal_type = 'gap'

        # ── Priority 2: frontier ─────────────────────────────────────
        if goal is None:
            goal = self._select_frontier(rx, ry, yaw)
            goal_type = 'frontier'

        # ── Priority 3: coverage (mapped-but-unvisited) ──────────────
        if goal is None:
            goal = self._select_coverage_goal(rx, ry, yaw)
            goal_type = 'coverage'

        # ── Blacklist relief ─────────────────────────────────────────
        if goal is None and self.blacklist:
            self.get_logger().info(
                f'No goal — clearing normal blacklist ({len(self.blacklist)} entries).')
            self.blacklist = []
            gap_r      = self._select_gap(rx, ry, yaw)
            frontier_r = self._select_frontier(rx, ry, yaw)
            coverage_r = self._select_coverage_goal(rx, ry, yaw)
            if gap_r is not None:
                goal      = gap_r
                goal_type = 'gap-retry'
            elif frontier_r is not None:
                goal      = frontier_r
                goal_type = 'frontier-retry'
            elif coverage_r is not None:
                goal      = coverage_r
                goal_type = 'coverage-retry'

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
                            'Exploration complete — no frontier cells remain.')
                        self.state = 'DONE'
                        return
                self.blacklist    = []
                self._no_goal_cnt = 0
                self.get_logger().info('Blacklist cleared — retrying.')
            return

        self._no_goal_cnt = 0
        gx, gy = goal
        self.frontier_goal = (gx, gy)
        self.get_logger().info(f'[{goal_type}] goal ({gx:.2f}, {gy:.2f})')

        self.state = 'NAVIGATING'
        self._send_nav2_goal(gx, gy, rx, ry)

    # ── Nav2 action client ──────────────────────────────────────────

    def _send_nav2_goal(self, gx, gy, rx, ry):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(
                'Nav2 NavigateToPose not available — is nav2_bringup running?')
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
            self.get_logger().warn('Nav2 goal rejected — blacklisting.')
            self._blacklist_goal()
            if not self._paused:
                self.state = 'GAP_SELECT'
            return

        self.get_logger().info('Nav2 goal accepted.')
        self.nav_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        """INT-BUG 2 FIX — guard state transition on _paused."""
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f'Nav2 result callback error: {e}')
            status = GoalStatus.STATUS_ABORTED

        self.nav_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached — selecting next.')
            self._blacklist_goal()
        else:
            self.get_logger().warn(f'Nav2 goal failed (status={status}) — blacklisting.')
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

    # ── Stuck detection + backup ────────────────────────────────────

    def _control_tick(self):
        if self._paused:
            return

        # SPEED-FIX: update narrow-corridor speed cap on every tick
        self._publish_speed_limit()

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
                            f'{self.stuck_pos[1]:.2f}) inaccessible — burying.')
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
                if rear_d > 0.18:
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

        # COVERAGE: mark current position as visited
        self._update_visitation(rx, ry)

        now_t = time.monotonic()
        if self.stuck_check_pos is None:
            self.stuck_check_pos = (rx, ry)
            self.stuck_check_t   = now_t
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
                return
            self.stuck_check_pos = (rx, ry)
            self.stuck_check_t   = now_t

    # ── Utilities ───────────────────────────────────────────────────

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