#!/usr/bin/env python3
"""
dock_controller_revised11.py — Runs on RPi
==========================================
Changes vs dock_controller_revised10.py:

  FIX-1  APPROACH exit condition: abs(dist_err) < DIST_TOLERANCE
         (was: dist_err < DIST_TOLERANCE — signed check let any overshoot
         satisfy the condition, dumping the robot into HOLD with no recovery
         path because dist_err was negative and the re-approach guard never
         fired.)

  FIX-2  Bidirectional APPROACH + HOLD re-approach covers both directions.
         APPROACH now allows negative vx (controlled reverse) so it can
         correct an overshoot as well as a too-far position. The HOLD
         re-approach condition was extended from
           dist_err > DIST_TOLERANCE * 2          (only too-far)
         to
           abs(dist_err) > DIST_TOLERANCE * 2     (too-far AND too-close).
         Without this, an overshoot that reached HOLD had no exit: count
         decayed to 0 but the re-approach guard never fired for negative
         dist_err.

  FIX-3  Minimum velocity floor MIN_VX = 0.02 m/s added to APPROACH.
         At DOCK_DISTANCE=0.08 m and KP_DIST=0.28 the P-output at the
         tolerance boundary is 0.28 × 0.03 ≈ 0.008 m/s, well below the
         TurtleBot3 Burger motor deadband (~0.01-0.02 m/s). The robot would
         stall before closing the last centimetre and never enter HOLD.
         vx is now clamped to [MIN_VX, MAX_VX] (or [-MAX_VX, -MIN_VX]
         for reverse) whenever |dist_err| > DIST_TOLERANCE.

  FIX-4  Tag-loss full timeout resets _dock_phase to COARSE_YAW.
         Previously, after a >6 s loss, the phase was preserved. If the
         robot was in APPROACH or HOLD during the loss and tag reappears
         without a new dock command, it would resume in a phase whose
         preconditions (yaw aligned, distance valid) are not guaranteed.
         Resetting to COARSE_YAW forces a clean re-alignment on every
         reacquire.

  FIX-5  COARSE_YAW now checks distance before rotating.
         If the robot is already past the dock point (tz < DOCK_DISTANCE -
         DIST_TOLERANCE) it jumps directly to APPROACH (which can now
         reverse) instead of spinning in place at <8 cm from the tag.
         If it is already within both distance and yaw tolerance it jumps
         straight to HOLD.

  FIX-6  HOLD publishes the stop command only once per phase entry.
         _hold_stop_sent flag is set on the first tick of HOLD and cleared
         on any phase exit. The 50 Hz stream of redundant zero-Twist
         messages is eliminated.

Inherited from REV-10:
  REV-10A  Sequential COARSE_YAW → APPROACH → HOLD FSM
  REV-10B  DOCK_DISTANCE = 0.08 m
  REV-10C  DOCKED_CONFIRM_TICKS = 15
  REV-10D  DIST_TOLERANCE = 0.03 m
  REV-9C   LOST_TIMEOUT = 6.0 s
  REV-9D   Three-tier tag loss handling
  REV-9E   _docked_count decay (not hard reset) on out-of-tolerance tick

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

        # FIX-6: track whether the HOLD entry stop has been published
        self._hold_stop_sent = False

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev11) ready  '
            f'DOCK_DIST={DOCK_DISTANCE}m  MAX_VX={MAX_VX}  MIN_VX={MIN_VX}  '
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

        FIX-5: Distance is checked before rotating. If the robot is already
        past the dock point (tz < DOCK_DISTANCE - DIST_TOLERANCE) it jumps
        directly to APPROACH (which can now reverse — FIX-2) instead of
        spinning in place at close range. If already within both distance
        and yaw tolerance it jumps straight to HOLD.

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

        # FIX-5b: robot is past the dock point → skip yaw spin, go correct distance
        if dist_err < -DIST_TOLERANCE:
            self.get_logger().info(
                f'COARSE_YAW: overshot (tz={tz:.3f}m < target)  → APPROACH to reverse')
            self._enter_phase('APPROACH', '')
            return

        # Normal case: align yaw first
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
        REV-10A APPROACH: pure longitudinal drive (wz = 0).

        FIX-1: exit to HOLD uses abs(dist_err) < DIST_TOLERANCE so that
        both overshoot (tz < DOCK_DISTANCE) and undershoot (tz > DOCK_DISTANCE)
        trigger the transition correctly. The old signed check let any
        overshoot satisfy dist_err < DIST_TOLERANCE unconditionally.

        FIX-2: vx is now bidirectional. Negative dist_err (overshot) produces
        a negative vx (controlled reverse). The MIN_VX floor (FIX-3) is
        applied symmetrically so the motor deadband is always overcome.

        FIX-3: minimum speed floor MIN_VX applied whenever the robot still
        needs to move, preventing stall near the tolerance boundary.

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

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        direction = 'fwd' if vx > 0 else 'rev'
        self.get_logger().info(
            f'[APPROACH] tz={tz:.2f}m  err={dist_err:.3f}m  vx={vx:.2f} ({direction})'
            f'  yaw={math.degrees(yaw_off):.1f}°',
            throttle_duration_sec=0.5)

    def _phase_hold(self, tz: float, yaw_off: float):
        """
        REV-10A HOLD: robot fully stopped, count consecutive in-tolerance ticks.

        FIX-6: the stop command is published only once per HOLD entry (via
        _hold_stop_sent flag) instead of every tick at 50 Hz.

        REV-9E decay preserved: one out-of-tolerance tick subtracts 1 count.

        FIX-2: re-approach condition now uses abs(dist_err) so an overshoot
        (negative dist_err) also escapes back to APPROACH — previously it
        was stuck because the signed guard never fired for negative values.
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
            # FIX-2: abs(dist_err) catches both too-far and too-close overshoot
            if self._docked_count == 0 and abs(dist_err) > DIST_TOLERANCE * 2:
                self.get_logger().info(
                    f'HOLD decay → APPROACH  tz={tz:.3f}m  err={dist_err:.3f}m'
                    f'  ({"reverse" if dist_err < 0 else "forward"})')
                self._enter_phase('APPROACH', '')
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
