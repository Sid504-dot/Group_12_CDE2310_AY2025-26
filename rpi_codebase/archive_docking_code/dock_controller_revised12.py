#!/usr/bin/env python3
"""
dock_controller_revised13.py — Runs on RPi
==========================================
Incorporates FIX-7, FIX-8, FIX-9 from dock_controller_revised12.py
into the rev10 FSM baseline.

  FIX-7  HOLD now has a yaw-drift escape path → COARSE_YAW.
         Previously HOLD had no exit for: dist_err in tolerance but
         yaw_off > YAW_HOLD_THRESH. _docked_count decayed to 0 but the
         only re-approach guard (abs(dist_err) > DIST_TOLERANCE * 2) never
         fired because distance was fine — robot permanently wedged in HOLD.
         Fix: when _docked_count reaches 0, check both:
           - abs(dist_err) > DIST_TOLERANCE * 2  →  APPROACH  (priority 1)
           - abs(yaw_off)  > YAW_HOLD_THRESH     →  COARSE_YAW  (priority 2)

  FIX-8  APPROACH applies gentle proportional yaw correction.
         Previously angular.z = 0.0 throughout APPROACH. Yaw drift
         accumulated, robot arrived at HOLD already misaligned.
         New constants KP_YAW_APPROACH=0.20, MAX_WZ_APPROACH=0.15 rad/s
         gently correct heading without fighting the linear drive.
         YAW_DRIFT_LIMIT abort → COARSE_YAW preserved as hard backstop.

  FIX-9  COARSE_YAW overshoot branch (FIX-5b) now requires yaw to also
         be aligned before jumping to APPROACH. Previously an overshot
         robot went straight to APPROACH regardless of heading, causing:
         overshoot → APPROACH → yaw drift → COARSE_YAW → overshoot → loop.
         Fix: fall through to normal wz correction if yaw is also bad.

  FIX-6  HOLD publishes the stop command only once per phase entry
         (not every tick). _hold_stop_sent flag reset by _enter_phase().

  FIX-5  COARSE_YAW checks distance before rotating:
         FIX-5a: already in full tolerance → skip straight to HOLD.
         FIX-5b: overshot → go to APPROACH (to reverse) when yaw OK.

  FIX-4  Tag-loss (> LOST_TIMEOUT) always resets _dock_phase to COARSE_YAW
         so re-acquisition always starts with a fresh yaw alignment.

  FIX-3  MIN_VX = 0.02 m/s — overcomes TurtleBot3 Burger motor deadband.

  FIX-2  Bidirectional APPROACH (vx can be negative to reverse when overshot).
         HOLD decay also checks abs(dist_err) for bidirectional re-approach.

  FIX-1  APPROACH exit uses abs(dist_err) < DIST_TOLERANCE so an overshoot
         also correctly triggers the HOLD transition.

Inherited from REV-10 (FSM baseline):
  REV-10A  Sequential COARSE_YAW → APPROACH → HOLD inner FSM
  REV-10B  DOCK_DISTANCE = 0.08 m (8 cm, ball launcher requirement)
  REV-10C  DOCKED_CONFIRM_TICKS = 15 (~0.30 s at 50 Hz)
  REV-10D  DIST_TOLERANCE = 0.03 m (tighter at close range)
  REV-9C   LOST_TIMEOUT = 6.0 s (RPi frame-drop tolerance)
  REV-9D   Three-tier tag loss handling (brief / medium / full timeout)
  REV-9E   _docked_count decay (not hard reset) on out-of-tolerance tick
  REV-8D   _pub_status throttled (state-change + 0.5 s heartbeat)

Interface (unchanged — mission_coordinator_v3 needs no edits):
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
  'docked'      — DOCKED state
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
DOCK_DISTANCE  = 0.08   # m — REV-10B: 8 cm (ball launcher requirement)
DIST_TOLERANCE = 0.03   # m — REV-10D: tighter at close range

YAW_HOLD_THRESH   = 0.10  # rad (~5.7°)  — HOLD phase tolerance
YAW_COARSE_THRESH = 0.08  # rad (~4.5°)  — exit COARSE_YAW, enter APPROACH
YAW_DRIFT_LIMIT   = 0.15  # rad (~8.6°)  — abort APPROACH → COARSE_YAW

KP_DIST = 0.28   # linear gain  (APPROACH)
KP_YAW  = 0.45   # angular gain (COARSE_YAW)
MAX_VX  = 0.06   # m/s forward/reverse cap
MIN_VX  = 0.02   # m/s — FIX-3: overcomes motor deadband
MAX_WZ  = 0.30   # rad/s — COARSE_YAW cap

# FIX-8: gentler gains for APPROACH — corrects drift without fighting vx
KP_YAW_APPROACH = 0.20
MAX_WZ_APPROACH = 0.15   # rad/s

LOST_TIMEOUT      = 6.0   # s — REV-9C
BRIEF_LOSS_HOLD_S = 0.3   # s — REV-9D

DOCK_HZ              = 50   # Hz
DOCKED_CONFIRM_TICKS = 15   # REV-10C: ~0.30 s at 50 Hz

BACKUP_SPEED    = 0.05  # m/s
BACKUP_DURATION = 2.0   # s


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command', self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',             10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self.state       = 'idle'
        self.target_type = None

        self._dock_phase     = 'COARSE_YAW'
        self._hold_stop_sent = False   # FIX-6

        self.last_det   = None
        self.last_det_t = time.monotonic()

        self._docked_count = 0
        self._backup_start = None

        self._last_status   = ''
        self._last_status_t = 0.0

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev13) ready  '
            f'DOCK_DIST={DOCK_DISTANCE}m  MAX_VX={MAX_VX}  MIN_VX={MIN_VX}  '
            f'KP_YAW_APPROACH={KP_YAW_APPROACH}  MAX_WZ_APPROACH={MAX_WZ_APPROACH}  '
            f'YAW_COARSE={math.degrees(YAW_COARSE_THRESH):.1f}°  '
            f'DRIFT_LIMIT={math.degrees(YAW_DRIFT_LIMIT):.1f}°')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_status(self, s: str):
        now = time.monotonic()
        if s != self._last_status or (now - self._last_status_t) > 0.5:
            m = String()
            m.data = s
            self.status_pub.publish(m)
            self._last_status   = s
            self._last_status_t = now

    def _enter_phase(self, phase: str, reason: str = ''):
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

        # Tier 1: full timeout — tag genuinely gone (REV-9D)
        if tag_age > LOST_TIMEOUT:
            self._stop()
            self._docked_count = 0
            if self._dock_phase != 'COARSE_YAW':
                # FIX-4: always re-align from scratch after a full tag loss
                self._enter_phase('COARSE_YAW',
                                  f'tag lost {tag_age:.1f} s — phase reset')
            self.get_logger().warn(
                f'Tag lost {tag_age:.1f} s — stopping',
                throttle_duration_sec=2.0)
            self._pub_status('lost')
            return

        # Waiting for first detection
        if self.last_det is None:
            self._pub_status('docking')
            return

        # Tier 2: brief loss — hold position, preserve counter and phase (REV-9D)
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
        Rotate in place only (vx = 0) until heading is aligned.

        FIX-5a: already in full tolerance → HOLD directly.
        FIX-5b + FIX-9: overshot → APPROACH only when yaw is also fine;
          otherwise fix yaw first to prevent the overshoot re-entry loop.
        Normal: spin until |yaw_off| < YAW_COARSE_THRESH → APPROACH.
        """
        dist_err = tz - DOCK_DISTANCE

        # FIX-5a: already fully in tolerance — skip to HOLD
        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_COARSE_THRESH:
            self.get_logger().info(
                f'COARSE_YAW: in tolerance  tz={tz:.3f}m'
                f'  yaw={math.degrees(yaw_off):.1f}° → HOLD')
            self._enter_phase('HOLD', '')
            self._docked_count = 0
            return

        # FIX-5b + FIX-9: overshot — only go to APPROACH if yaw also fine
        if dist_err < -DIST_TOLERANCE:
            if abs(yaw_off) < YAW_COARSE_THRESH:
                self.get_logger().info(
                    f'COARSE_YAW: overshot tz={tz:.3f}m  yaw OK → APPROACH (reverse)')
                self._enter_phase('APPROACH', '')
                return
            # Yaw also bad — fix heading first, FIX-5b fires on next aligned tick
            self.get_logger().info(
                f'COARSE_YAW: overshot + misaligned'
                f'  tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}° — aligning first',
                throttle_duration_sec=0.5)

        # Normal: align yaw
        if abs(yaw_off) < YAW_COARSE_THRESH:
            self.get_logger().info(
                f'COARSE_YAW done  yaw={math.degrees(yaw_off):.1f}°'
                f'  tz={tz:.2f}m → APPROACH')
            self._enter_phase('APPROACH', '')
            return

        wz = max(-MAX_WZ, min(MAX_WZ, -KP_YAW * yaw_off))

        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[COARSE_YAW] tz={tz:.2f}m  yaw={math.degrees(yaw_off):.1f}°  wz={wz:.2f}',
            throttle_duration_sec=0.5)

    def _phase_approach(self, tz: float, yaw_off: float):
        """
        Drive toward (or away from) tag with gentle yaw correction.

        FIX-1: abs(dist_err) for HOLD transition — catches overshoot.
        FIX-2: bidirectional vx — negative when overshot.
        FIX-3: MIN_VX floor — overcomes motor deadband.
        FIX-8: KP_YAW_APPROACH / MAX_WZ_APPROACH — gentle heading correction
               that keeps robot pointed at tag without fighting linear drive.
        """
        if abs(yaw_off) > YAW_DRIFT_LIMIT:
            self.get_logger().info(
                f'APPROACH drift  yaw={math.degrees(yaw_off):.1f}° → COARSE_YAW')
            self._stop()
            self._enter_phase('COARSE_YAW', '')
            return

        dist_err = tz - DOCK_DISTANCE

        # FIX-1: abs() catches overshoot
        if abs(dist_err) < DIST_TOLERANCE:
            self.get_logger().info(
                f'APPROACH done  tz={tz:.3f}m  err={dist_err:.3f}m → HOLD')
            self._stop()
            self._enter_phase('HOLD', '')
            self._docked_count = 0
            return

        # FIX-2 + FIX-3: bidirectional with deadband-busting floor
        raw_vx = KP_DIST * dist_err
        if dist_err > 0:
            vx = max(MIN_VX, min(MAX_VX, raw_vx))
        else:
            vx = min(-MIN_VX, max(-MAX_VX, raw_vx))

        # FIX-8: gentle proportional yaw correction
        wz = max(-MAX_WZ_APPROACH, min(MAX_WZ_APPROACH, -KP_YAW_APPROACH * yaw_off))

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'[APPROACH] tz={tz:.2f}m  err={dist_err:.3f}m'
            f'  vx={vx:.2f} ({"fwd" if vx > 0 else "rev"})'
            f'  yaw={math.degrees(yaw_off):.1f}°  wz={wz:.2f}',
            throttle_duration_sec=0.5)

    def _phase_hold(self, tz: float, yaw_off: float):
        """
        Robot stopped; count consecutive ticks within both tolerances.

        FIX-6: stop published once per phase entry only.
        REV-9E: decay _docked_count rather than hard-reset on out-of-tolerance.
        FIX-7: when count decays to 0, two ordered escape paths:
          1. abs(dist_err) > DIST_TOLERANCE*2 → APPROACH (also corrects yaw via FIX-8)
          2. abs(yaw_off)  > YAW_HOLD_THRESH  → COARSE_YAW (yaw-only fix)
        """
        dist_err = tz - DOCK_DISTANCE

        # FIX-6: stop once on entry
        if not self._hold_stop_sent:
            self._stop()
            self._hold_stop_sent = True

        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_HOLD_THRESH:
            self._docked_count += 1
            if self._docked_count >= DOCKED_CONFIRM_TICKS:
                self.state = 'docked'
                self.get_logger().info(
                    f'DOCKED  tz={tz:.3f}m  yaw={math.degrees(yaw_off):.1f}°'
                    f'  ({DOCKED_CONFIRM_TICKS} ticks)')
                self._pub_status('docked')
                return
        else:
            # REV-9E: decay
            self._docked_count = max(0, self._docked_count - 1)

            if self._docked_count == 0:
                # Priority 1: distance clearly off — re-approach (FIX-8 fixes yaw too)
                if abs(dist_err) > DIST_TOLERANCE * 2:
                    self.get_logger().info(
                        f'HOLD decay → APPROACH  tz={tz:.3f}m  err={dist_err:.3f}m'
                        f'  ({"rev" if dist_err < 0 else "fwd"})')
                    self._enter_phase('APPROACH', '')
                    return
                # Priority 2: distance OK, yaw drifted — re-align (FIX-7)
                if abs(yaw_off) > YAW_HOLD_THRESH:
                    self.get_logger().info(
                        f'HOLD decay → COARSE_YAW  yaw={math.degrees(yaw_off):.1f}°'
                        f'  tz={tz:.3f}m')
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
