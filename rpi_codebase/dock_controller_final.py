#!/usr/bin/env python3
"""
dock_controller_revised12.py — Runs on RPi
==========================================
Changes vs dock_controller_revised11.py:

  FIX-7  HOLD now has a yaw-drift escape path → COARSE_YAW.
         REV-11 HOLD had no exit for the case where dist_err is in tolerance
         but yaw_off exceeds YAW_HOLD_THRESH: _docked_count decayed to 0 but
         the only re-approach guard (abs(dist_err) > DIST_TOLERANCE * 2) never
         fired because distance was fine. Robot was permanently wedged in HOLD
         with bad yaw alignment, unable to dock.
         Fix: when _docked_count reaches 0 in the out-of-tolerance branch,
         check yaw as well as distance:
           - abs(dist_err) > DIST_TOLERANCE * 2  →  APPROACH  (unchanged)
           - abs(yaw_off)  > YAW_HOLD_THRESH     →  COARSE_YAW  (new)
         The distance check takes priority so that a combined dist+yaw error
         routes to APPROACH first (APPROACH now corrects yaw — see FIX-8).

  FIX-8  APPROACH applies gentle proportional yaw correction.
         REV-11 hardcoded angular.z = 0.0 for the entire APPROACH phase. Yaw
         errors accumulated during the straight-line drive, frequently causing
         the robot to arrive at HOLD already misaligned (triggering FIX-7 on
         every attempt). Two new constants scope this narrowly:
           KP_YAW_APPROACH = 0.20   (vs KP_YAW = 0.45 in COARSE_YAW)
           MAX_WZ_APPROACH = 0.15 rad/s  (vs MAX_WZ = 0.30 in COARSE_YAW)
         The gentler gain avoids fighting the linear drive while still keeping
         the heading pointed at the tag. The YAW_DRIFT_LIMIT abort (→ COARSE_YAW)
         is preserved as a hard backstop.

  FIX-9  COARSE_YAW FIX-5b only jumps to APPROACH when yaw is also aligned.
         REV-11 FIX-5b sent an overshot robot immediately to APPROACH regardless
         of heading. During the reverse, yaw drift could exceed YAW_DRIFT_LIMIT,
         aborting back to COARSE_YAW, which then saw the overshoot again and
         sent it back to APPROACH — an infinite loop with no progress.
         Fix: FIX-5b now requires abs(yaw_off) < YAW_COARSE_THRESH before
         transitioning. If yaw is also misaligned, the function falls through
         to the normal wz-only correction. On the next tick(s), once yaw is
         within threshold, FIX-5b will fire and send the robot to APPROACH.
         With FIX-8 active, the short reverse leg then maintains heading.

Inherited from REV-11:
  FIX-1  APPROACH exit: abs(dist_err) < DIST_TOLERANCE
  FIX-2  Bidirectional APPROACH + HOLD re-approach (abs)
  FIX-3  MIN_VX = 0.02 m/s floor
  FIX-4  Tag-loss resets _dock_phase to COARSE_YAW
  FIX-5  COARSE_YAW checks distance before rotating
  FIX-6  HOLD publishes stop only once per phase entry
  REV-10A Sequential COARSE_YAW → APPROACH → HOLD FSM
  REV-10B DOCK_DISTANCE = 0.08 m
  REV-10C DOCKED_CONFIRM_TICKS = 15
  REV-10D DIST_TOLERANCE = 0.03 m
  REV-9C  LOST_TIMEOUT = 6.0 s
  REV-9D  Three-tier tag loss handling
  REV-9E  _docked_count decay (not hard reset) on out-of-tolerance tick

Interface (unchanged — mission_coordinator needs no edits):
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
MAX_VX  = 0.06   # m/s — forward/reverse cap
# FIX-3: minimum speed to overcome TurtleBot3 Burger motor deadband (~0.01-0.02 m/s)
MIN_VX  = 0.02   # m/s — applied whenever |dist_err| > DIST_TOLERANCE
MAX_WZ  = 0.30   # rad/s — angular cap for COARSE_YAW (gentler than rev9's 0.50)

# FIX-8: separate, gentler yaw gains for APPROACH so heading correction does
# not fight the linear drive. Half the COARSE_YAW values.
KP_YAW_APPROACH  = 0.20   # angular P-gain during APPROACH
MAX_WZ_APPROACH  = 0.15   # rad/s — angular cap during APPROACH

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

        # FIX-6: track whether the HOLD entry stop has been published
        self._hold_stop_sent = False

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev12) ready  '
            f'DOCK_DIST={DOCK_DISTANCE}m  MAX_VX={MAX_VX}  MIN_VX={MIN_VX}  '
            f'KP_YAW_APPROACH={KP_YAW_APPROACH}  MAX_WZ_APPROACH={MAX_WZ_APPROACH}  '
            f'YAW_COARSE_THRESH={math.degrees(YAW_COARSE_THRESH):.1f}°  '
            f'YAW_DRIFT_LIMIT={math.degrees(YAW_DRIFT_LIMIT):.1f}°  '
            f'LOST_TO={LOST_TIMEOUT}s')

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

    def _enter_phase(self, phase: str, reason: str = ''):
        """Transition inner FSM to a new phase with logging and flag resets."""
        self._dock_phase     = phase
        self._hold_stop_sent = False   # FIX-6: reset on every phase change
        if reason:
            self.get_logger().info(f'phase → {phase}  {reason}')

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
        self.target_type     = target_type
        self.state           = 'docking'
        self._docked_count   = 0
        self._hold_stop_sent = False
        self.last_det        = None
        self.last_det_t      = time.monotonic()
        self._stop()
        self._enter_phase('COARSE_YAW', f'dock cmd: {label}')

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
            # FIX-4: reset phase so re-acquire always starts with fresh alignment
            if self._dock_phase != 'COARSE_YAW':
                self._enter_phase('COARSE_YAW',
                                  f'tag lost {tag_age:.1f} s — phase reset')
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

        FIX-5: Distance is checked before rotating. If already within both
        distance and yaw tolerance, jump straight to HOLD (FIX-5a).

        FIX-5b (updated by FIX-9): If the robot is past the dock point
        (tz < DOCK_DISTANCE - DIST_TOLERANCE) AND yaw is already aligned,
        jump directly to APPROACH (which can now reverse — FIX-2).
        If yaw is ALSO misaligned, fall through to normal yaw correction
        first. This prevents the overshoot → APPROACH → yaw drift →
        COARSE_YAW → overshoot loop that existed in REV-11.

        Exits to APPROACH when |yaw_off| < YAW_COARSE_THRESH.
        """
        dist_err = tz - DOCK_DISTANCE

        # FIX-5a: already within full tolerance → skip straight to HOLD
        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_COARSE_THRESH:
            self.get_logger().info(
                f'COARSE_YAW: already in tolerance  tz={tz:.3f}m'
                f'  yaw={math.degrees(yaw_off):.1f}° → HOLD')
            self._enter_phase('HOLD', '')
            self._docked_count = 0
            return

        # FIX-5b + FIX-9: overshot — only reverse if yaw is also aligned.
        # If yaw is bad, correct heading first (fall through to wz spin below);
        # once aligned, this branch will fire again and send us to APPROACH.
        if dist_err < -DIST_TOLERANCE:
            if abs(yaw_off) < YAW_COARSE_THRESH:
                self.get_logger().info(
                    f'COARSE_YAW: overshot (tz={tz:.3f}m) yaw OK'
                    f' → APPROACH to reverse')
                self._enter_phase('APPROACH', '')
                return
            # yaw also misaligned — log and fall through to yaw correction
            self.get_logger().info(
                f'COARSE_YAW: overshot AND misaligned'
                f'  tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}° — fix yaw first',
                throttle_duration_sec=0.5)

        # Normal case: align yaw first (also handles yaw-bad overshoot above)
        if abs(yaw_off) < YAW_COARSE_THRESH:
            self.get_logger().info(
                f'COARSE_YAW done  yaw={math.degrees(yaw_off):.1f}°'
                f'  tz={tz:.2f}m → APPROACH')
            self._enter_phase('APPROACH', '')
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
        REV-10A APPROACH: longitudinal drive with gentle yaw correction.

        FIX-1: exit to HOLD uses abs(dist_err) < DIST_TOLERANCE (bidirectional).
        FIX-2: vx is bidirectional — negative dist_err gives controlled reverse.
        FIX-3: MIN_VX floor overcomes motor deadband.

        FIX-8 (new): gentle proportional yaw correction is now applied
        alongside the linear drive. KP_YAW_APPROACH (0.20) and MAX_WZ_APPROACH
        (0.15 rad/s) are intentionally smaller than the COARSE_YAW values so
        the yaw term does not fight vx. This prevents heading from drifting
        all the way to YAW_DRIFT_LIMIT during a long approach, which previously
        caused the robot to arrive at HOLD already misaligned.
        YAW_DRIFT_LIMIT abort (→ COARSE_YAW) is preserved as a hard backstop.

        Aborts to COARSE_YAW if yaw drifts past YAW_DRIFT_LIMIT.
        Exits to HOLD when |dist_err| < DIST_TOLERANCE.
        """
        if abs(yaw_off) > YAW_DRIFT_LIMIT:
            self.get_logger().info(
                f'APPROACH drift  yaw={math.degrees(yaw_off):.1f}° >'
                f' {math.degrees(YAW_DRIFT_LIMIT):.1f}° — stopping → COARSE_YAW')
            self._stop()
            self._enter_phase('COARSE_YAW', '')
            return

        dist_err = tz - DOCK_DISTANCE

        # FIX-1: use abs() so overshoot also triggers transition to HOLD
        if abs(dist_err) < DIST_TOLERANCE:
            self.get_logger().info(
                f'APPROACH done  tz={tz:.3f}m  err={dist_err:.3f}m → HOLD')
            self._stop()
            self._enter_phase('HOLD', '')
            self._docked_count = 0
            return

        # FIX-2 & FIX-3: bidirectional drive with minimum speed floor
        raw_vx = KP_DIST * dist_err                        # negative when overshot
        if dist_err > 0:
            vx = max(MIN_VX, min(MAX_VX, raw_vx))          # forward: [MIN_VX, MAX_VX]
        else:
            vx = min(-MIN_VX, max(-MAX_VX, raw_vx))        # reverse: [-MAX_VX, -MIN_VX]

        # FIX-8: gentle yaw correction — keeps heading pointed at tag during
        # approach without fighting the linear drive
        wz = -KP_YAW_APPROACH * yaw_off
        wz = max(-MAX_WZ_APPROACH, min(MAX_WZ_APPROACH, wz))

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        direction = 'fwd' if vx > 0 else 'rev'
        self.get_logger().info(
            f'[APPROACH] tz={tz:.2f}m  err={dist_err:.3f}m  vx={vx:.2f} ({direction})'
            f'  yaw={math.degrees(yaw_off):.1f}°  wz={wz:.2f}',
            throttle_duration_sec=0.5)

    def _phase_hold(self, tz: float, yaw_off: float):
        """
        REV-10A HOLD: robot fully stopped, count consecutive in-tolerance ticks.

        FIX-6: the stop command is published only once per HOLD entry.
        REV-9E decay preserved: one out-of-tolerance tick subtracts 1 count.
        FIX-2: re-approach uses abs(dist_err) for bidirectional recovery.

        FIX-7 (new): yaw-drift escape path added. Previously, if dist_err was
        in tolerance but yaw_off exceeded YAW_HOLD_THRESH, the count decayed
        to 0 but the only exit (abs(dist_err) > DIST_TOLERANCE * 2) never
        fired — robot was permanently wedged. Now when _docked_count reaches 0,
        two exits are checked in priority order:
          1. abs(dist_err) > DIST_TOLERANCE * 2  →  APPROACH  (distance fix first;
             APPROACH now also corrects yaw via FIX-8)
          2. abs(yaw_off)  > YAW_HOLD_THRESH     →  COARSE_YAW  (yaw-only fix)
        """
        dist_err = tz - DOCK_DISTANCE

        # FIX-6: publish stop only once on phase entry, not every tick
        if not self._hold_stop_sent:
            self._stop()
            self._hold_stop_sent = True

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

            if self._docked_count == 0:
                # Priority 1: distance is too far off — re-approach
                # (APPROACH will also correct any yaw drift via FIX-8)
                if abs(dist_err) > DIST_TOLERANCE * 2:
                    self.get_logger().info(
                        f'HOLD decay → APPROACH  tz={tz:.3f}m  err={dist_err:.3f}m'
                        f'  ({"reverse" if dist_err < 0 else "forward"})')
                    self._enter_phase('APPROACH', '')
                    return

                # FIX-7: Priority 2: distance is OK but yaw has drifted —
                # re-align heading before trying to confirm again
                if abs(yaw_off) > YAW_HOLD_THRESH:
                    self.get_logger().info(
                        f'HOLD decay → COARSE_YAW  yaw drift'
                        f'  yaw={math.degrees(yaw_off):.1f}°'
                        f'  tz={tz:.3f}m  (distance OK)')
                    self._enter_phase('COARSE_YAW', '')
                    return

        self.get_logger().info(
            f'[HOLD] tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}°'
            f'  confirm={self._docked_count}/{DOCKED_CONFIRM_TICKS}',
            throttle_duration_sec=0.5)


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
