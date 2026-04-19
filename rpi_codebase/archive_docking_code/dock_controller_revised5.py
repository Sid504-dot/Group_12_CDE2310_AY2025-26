#!/usr/bin/env python3
"""
dock_controller_revised5.py — Runs on RPi
==========================================
Changes vs dock_controller_revised9.py:

  REV-10A  Sequential FSM replaces the simultaneous P-controller.

           OLD (rev8/9): vx and wz applied at the same time every tick.
           Problem: angular correction swings the camera; the tag drifts
           toward the FOV edge and detection drops mid-approach.

           NEW: three sequential internal phases, each doing ONE thing:

             COARSE_YAW  — rotate in place (vx=0) until yaw aligned.
                           Tag stays centred in frame: no lateral motion.

             APPROACH    — drive straight forward (wz=0) until close.
                           Abort back to COARSE_YAW if yaw drifts past
                           YAW_DRIFT_LIMIT (0.15 rad). Tag glides toward
                           the camera centre with no yaw swings.

             HOLD        — robot fully stopped, count N consecutive
                           in-tolerance ticks before declaring docked.
                           Decay on out-of-tolerance (REV-9E preserved).
                           If count decays to 0 and robot has drifted away,
                           return to APPROACH.

  REV-10B  DOCK_DISTANCE reduced 0.20 → 0.08 m (ball launcher requirement).
           At 0.08 m: TAG_SIZE=0.08 m, fx=462 px → projected tag width ≈ 462 px.
           Camera is 640 px wide — tag fits. Tune up to 0.12 m if the
           detector starts losing the tag at this close range.

  REV-10C  DOCKED_CONFIRM_TICKS increased 10 → 15 (~0.30 s at 50 Hz).
           Slightly longer hold before declaring docked ensures the robot
           is truly settled at the tighter 8 cm stopping distance.

  REV-10D  DIST_TOLERANCE tightened 0.05 → 0.03 m to match the shorter
           stopping distance. At 8 cm, 5 cm tolerance would accept the
           robot anywhere between 5–13 cm — too loose.

Inherited from REV-9:
  REV-9C   LOST_TIMEOUT 6.0 s (increased from 3.0 s for RPi frame-drop tolerance)
  REV-9D   Three-tier tag loss handling (brief / medium / full timeout)
  REV-9E   _docked_count decay instead of hard reset on out-of-tolerance tick

Interface (unchanged from rev8/9 — mission_coordinator needs no edits):
  Subscribe:  /apriltag/detections   (String JSON)
              /mission/dock_command  (String)
  Publish:    /cmd_vel               (Twist)
              /mission/dock_status   (String)

Commands received on /mission/dock_command:
  'dock_static'   — dock to tag type 'static'
  'dock_dynamic'  — dock to tag type 'dynamic_dock'
  'backup'        — reverse BACKUP_DURATION seconds
  'cancel'        — abort docking, return to idle

Status strings published on /mission/dock_status:
  'idle'        — IDLE state
  'docking'     — any of COARSE_YAW / APPROACH / HOLD
  'docked'      — DOCKED state (fires mission coordinator's next step)
  'lost'        — tag absent > LOST_TIMEOUT
  'backup_done' — end of BACKING_UP sequence
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ────────────────────────────────────────────────────────
# REV-10B: reduced from 0.20 → 0.08 m (ball launcher requirement)
DOCK_DISTANCE  = 0.08   # m — stop this far (forward depth) from tag face

# REV-10D: tightened from 0.05 → 0.03 m to match shorter stopping distance
DIST_TOLERANCE = 0.03   # m — ±3 cm counts as "at dock distance"

# Yaw tolerance in HOLD phase (slightly looser to avoid HOLD → APPROACH thrash)
YAW_HOLD_THRESH = 0.10  # rad (~5.7°)

# REV-10A: sequential FSM thresholds
YAW_COARSE_THRESH = 0.08  # rad (~4.5°) — exit COARSE_YAW, enter APPROACH
YAW_DRIFT_LIMIT   = 0.15  # rad (~8.6°) — abort APPROACH → back to COARSE_YAW

KP_DIST = 0.28   # linear gain  (APPROACH phase only)
KP_YAW  = 0.45   # angular gain (COARSE_YAW phase only)
MAX_VX  = 0.06   # m/s — forward cap (same as rev9)
MAX_WZ  = 0.30   # rad/s — angular cap (gentler than rev9's 0.50)

# REV-9C: increased from 3.0 — RPi frame-drops can blank detection for 1-2 s
LOST_TIMEOUT      = 6.0   # s — full stop + counter reset after this long without tag
# REV-9D: over 0.3 s at 0.06 m/s the robot moves < 2 cm — stale pose is safe
BRIEF_LOSS_HOLD_S = 0.3   # s — use stale detection; beyond this hold still

DOCK_HZ = 50   # Hz

# REV-10C: increased from 10 → 15 (~0.30 s at 50 Hz)
DOCKED_CONFIRM_TICKS = 15

# ── Backup parameters ─────────────────────────────────────────────────────────
BACKUP_SPEED    = 0.05  # m/s
BACKUP_DURATION = 2.0   # s


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command', self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',             10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        # ── Outer state (coordinator sees this via published status strings) ───
        self.state       = 'idle'    # idle | docking | docked | backing_up
        self.target_type = None

        # ── Inner FSM phase (only active when state == 'docking') ─────────────
        # REV-10A: COARSE_YAW → APPROACH → HOLD
        self._dock_phase = 'COARSE_YAW'

        # ── Tag detection ──────────────────────────────────────────────────────
        self.last_det   = None
        self.last_det_t = time.monotonic()

        # ── Confirmation counter ───────────────────────────────────────────────
        self._docked_count = 0

        # ── Backup ────────────────────────────────────────────────────────────
        self._backup_start = None

        # ── Status publish throttle (REV-8D) ──────────────────────────────────
        self._last_status   = ''
        self._last_status_t = 0.0

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev10) ready  '
            f'DOCK_DIST={DOCK_DISTANCE}m  MAX_VX={MAX_VX}  '
            f'YAW_COARSE_THRESH={math.degrees(YAW_COARSE_THRESH):.1f}°  '
            f'YAW_DRIFT_LIMIT={math.degrees(YAW_DRIFT_LIMIT):.1f}°  '
            f'LOST_TO={LOST_TIMEOUT}s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'dock_static':
            self._start_dock('static', 'STATIC')

        elif cmd == 'dock_dynamic':
            self._start_dock('dynamic_dock', 'DYNAMIC')

        elif cmd == 'backup':
            self._stop()
            self._backup_start = time.monotonic()
            self.state         = 'backing_up'
            self._docked_count = 0
            self.get_logger().info(f'Backup — reversing {BACKUP_DURATION} s')

        elif cmd == 'cancel':
            self.state         = 'idle'
            self.target_type   = None
            self._docked_count = 0
            self._stop()
            self.get_logger().info('Docking cancelled')

    def _start_dock(self, target_type: str, label: str):
        self.target_type   = target_type
        self.state         = 'docking'
        self._dock_phase   = 'COARSE_YAW'
        self.last_det      = None
        self.last_det_t    = time.monotonic()
        self._docked_count = 0
        self._stop()
        self.get_logger().info(f'Dock cmd: {label}  phase→COARSE_YAW')

    def _det_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if self.target_type is None:
            return
        for tag in data.get('tags', []):
            if tag['type'] == self.target_type:
                self.last_det   = tag
                self.last_det_t = time.monotonic()
                return

    # ── Control tick ──────────────────────────────────────────────────────────

    def _tick(self):
        # BACKING UP ──────────────────────────────────────────────────────────
        if self.state == 'backing_up':
            if time.monotonic() - self._backup_start >= BACKUP_DURATION:
                self._stop()
                self.state = 'idle'
                self._pub_status('backup_done')
                self.get_logger().info('Backup complete')
            else:
                t = Twist()
                t.linear.x = -BACKUP_SPEED
                self.cmd_pub.publish(t)
            return

        if self.state in ('idle', 'docked'):
            self._pub_status(self.state)
            return

        if self.state != 'docking':
            return

        # DOCKING FSM ─────────────────────────────────────────────────────────
        tag_age = time.monotonic() - self.last_det_t

        # REV-9D tier 1: full timeout — tag genuinely gone
        if tag_age > LOST_TIMEOUT:
            self._stop()
            self._docked_count = 0
            self.get_logger().warn(
                f'Tag lost {tag_age:.1f} s — stopping',
                throttle_duration_sec=2.0)
            self._pub_status('lost')
            return

        # Waiting for first detection after dock command
        if self.last_det is None:
            self._pub_status('docking')
            return

        # REV-9D tier 2: brief loss — hold position, preserve counter and phase
        if tag_age > BRIEF_LOSS_HOLD_S:
            self._stop()
            self._pub_status('docking')
            return

        # Fresh detection — dispatch to inner FSM ─────────────────────────────
        tz      = self.last_det['tz']
        yaw_off = self.last_det['yaw_offset']

        if self._dock_phase == 'COARSE_YAW':
            self._phase_coarse_yaw(tz, yaw_off)
        elif self._dock_phase == 'APPROACH':
            self._phase_approach(tz, yaw_off)
        elif self._dock_phase == 'HOLD':
            self._phase_hold(tz, yaw_off)

        self._pub_status('docking')

    # ── Inner FSM phases ──────────────────────────────────────────────────────

    def _phase_coarse_yaw(self, tz: float, yaw_off: float):
        """
        REV-10A COARSE_YAW: rotate in place only (vx = 0).

        The camera pivots around the robot base, so the tag stays in the FOV
        as long as the initial yaw error is < ~25-30°. No lateral drift means
        the tag never sweeps out of frame during alignment.

        Exits to APPROACH when |yaw_off| < YAW_COARSE_THRESH.
        """
        if abs(yaw_off) < YAW_COARSE_THRESH:
            self.get_logger().info(
                f'COARSE_YAW done  yaw={math.degrees(yaw_off):.1f}°  tz={tz:.2f}m'
                f' → APPROACH')
            self._dock_phase = 'APPROACH'
            return

        wz = -KP_YAW * yaw_off
        wz = max(-MAX_WZ, min(MAX_WZ, wz))

        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[COARSE_YAW] tz={tz:.2f}m  yaw={math.degrees(yaw_off):.1f}°'
            f'  wz={wz:.2f}',
            throttle_duration_sec=0.5)

    def _phase_approach(self, tz: float, yaw_off: float):
        """
        REV-10A APPROACH: pure forward drive (wz = 0).

        No angular correction keeps the tag centred as the robot closes.
        If the floor is uneven and yaw drifts past YAW_DRIFT_LIMIT, we stop
        and return to COARSE_YAW to re-align before the tag leaves frame.

        Exits to HOLD when within DIST_TOLERANCE of DOCK_DISTANCE.
        Exits to COARSE_YAW if |yaw_off| > YAW_DRIFT_LIMIT.
        """
        if abs(yaw_off) > YAW_DRIFT_LIMIT:
            self.get_logger().info(
                f'APPROACH drift  yaw={math.degrees(yaw_off):.1f}° >'
                f' {math.degrees(YAW_DRIFT_LIMIT):.1f}° — stopping → COARSE_YAW')
            self._stop()
            self._dock_phase = 'COARSE_YAW'
            return

        dist_err = tz - DOCK_DISTANCE

        if dist_err < DIST_TOLERANCE:
            self.get_logger().info(
                f'APPROACH done  tz={tz:.3f}m  err={dist_err:.3f}m → HOLD')
            self._stop()
            self._dock_phase   = 'HOLD'
            self._docked_count = 0
            return

        vx = min(MAX_VX, KP_DIST * max(0.0, dist_err))

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[APPROACH] tz={tz:.2f}m  err={dist_err:.3f}m  vx={vx:.2f}'
            f'  yaw={math.degrees(yaw_off):.1f}°',
            throttle_duration_sec=0.5)

    def _phase_hold(self, tz: float, yaw_off: float):
        """
        REV-10A HOLD: robot fully stopped, count consecutive in-tolerance ticks.

        REV-9E decay preserved: one out-of-tolerance tick subtracts 1 count
        (not a hard reset to 0). Handles brief oscillation at the tolerance
        boundary without losing all accumulated confirmation ticks.

        If count decays fully to 0 AND dist_err is large (robot drifted),
        return to APPROACH so the robot can close the gap.
        """
        dist_err = tz - DOCK_DISTANCE

        # Robot is stopped in HOLD — no cmd_vel published
        self._stop()

        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_HOLD_THRESH:
            self._docked_count += 1
            if self._docked_count >= DOCKED_CONFIRM_TICKS:
                self.state = 'docked'
                self.get_logger().info(
                    f'DOCKED  tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}°'
                    f'  ({DOCKED_CONFIRM_TICKS} consecutive ticks)')
                self._pub_status('docked')
                return
        else:
            # REV-9E: decay rather than hard reset
            self._docked_count = max(0, self._docked_count - 1)
            # If fully decayed and clearly out of distance, re-approach
            if self._docked_count == 0 and dist_err > DIST_TOLERANCE * 2:
                self.get_logger().info(
                    f'HOLD decay → APPROACH  tz={tz:.3f}m  err={dist_err:.3f}m')
                self._dock_phase = 'APPROACH'

        self.get_logger().info(
            f'[HOLD] tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}°'
            f'  confirm={self._docked_count}/{DOCKED_CONFIRM_TICKS}',
            throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_status(self, s: str):
        """Publish only on state change or every 0.5 s heartbeat (REV-8D)."""
        now = time.monotonic()
        if s != self._last_status or (now - self._last_status_t) > 0.5:
            m = String()
            m.data = s
            self.status_pub.publish(m)
            self._last_status   = s
            self._last_status_t = now


def main(args=None):
    rclpy.init(args=args)
    node = DockController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
