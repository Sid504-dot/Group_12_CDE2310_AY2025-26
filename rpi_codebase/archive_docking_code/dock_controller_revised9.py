#!/usr/bin/env python3
"""
dock_controller_revised9.py — Runs on RPi
==========================================
Changes vs dock_controller_revised8.py:

  REV-9A  MAX_VX reduced 0.10 → 0.06 m/s. The robot was approaching too
          fast and overshooting the tag, causing the detector to lose the
          tag (tag exits frame or goes out of focus). Slower approach
          keeps the tag in frame throughout the final metres.

  REV-9B  KP_DIST reduced 0.40 → 0.25. The proportional gain was too
          aggressive — even at 0.5 m error the robot hit MAX_VX instantly.
          Lower gain gives a smoother ramp and less oscillation.

  REV-9C  KP_YAW reduced 0.70 → 0.50, MAX_WZ reduced 0.50 → 0.35 rad/s.
          High angular velocity caused the tag to sweep out of the camera
          FOV during alignment, triggering a "lost" detection. Gentler
          yaw corrections keep the tag centred.

  REV-9D  LOST_TIMEOUT increased 3.0 → 5.0 s. The camera on the RPi runs
          at ~15 FPS and pupil-apriltags can miss 2–5 consecutive frames
          when the tag is at an oblique angle or partially occluded.
          3 s was too aggressive — the controller would give up and stop
          while the tag was still physically in front of the robot.

  REV-9E  Docking logic now applies identically for both 'static' and
          'dynamic_dock' target types. The _det_cb already matches on
          self.target_type, so no code change was needed — but this
          docstring clarifies: the dock controller does NOT distinguish
          between static and dynamic docking behaviour. Both use the
          same P-controller approach to the tag. The difference is
          which tag type the mission coordinator tells us to track.

Inherited from REV-8:
  REV-8A  DOCK_DISTANCE 0.20 m (tag stays in frame)
  REV-8B  DIST_TOLERANCE 0.05 m
  REV-8C  Hysteresis counter (10 ticks) before declaring docked
  REV-8D  Status publishing throttled to state-change + 0.5 s heartbeat

How docking works (for reference):
  The dock controller uses a simple proportional (P) controller:
  - It subscribes to /apriltag/detections and looks for a tag matching
    self.target_type ('static' or 'dynamic_dock').
  - From the tag's pose estimate it reads:
      tz        = forward distance to tag (metres)
      yaw_offset = lateral angle to tag centre (radians)
  - Linear velocity:  vx = KP_DIST * (tz - DOCK_DISTANCE)
    → drives forward proportionally to how far it is from the target
      stopping distance. Clamped to [0, MAX_VX] (no reverse).
  - Angular velocity: wz = -KP_YAW * yaw_offset
    → turns toward the tag centre. Clamped to [-MAX_WZ, MAX_WZ].
  - When both dist_err < DIST_TOLERANCE and yaw_offset < YAW_TOLERANCE
    hold for DOCKED_CONFIRM_TICKS consecutive control ticks, the
    controller declares 'docked' and stops.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ────────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.20   # m — stop this far (forward depth) from tag face
DIST_TOLERANCE = 0.05   # m — ±5 cm counts as "at dock distance"
YAW_TOLERANCE  = 0.12   # rad (~7°) — counts as "aligned"

# REV-9B: reduced from 0.40 → 0.25
KP_DIST = 0.25          # linear gain   (m/s per m error)
# REV-9C: reduced from 0.70 → 0.50
KP_YAW  = 0.50          # angular gain  (rad/s per rad error)
# REV-9A: reduced from 0.10 → 0.06
MAX_VX  = 0.06          # m/s forward cap
# REV-9C: reduced from 0.50 → 0.35
MAX_WZ  = 0.35          # rad/s angular cap

# REV-9D: increased from 3.0 → 5.0
LOST_TIMEOUT = 5.0      # s — hold still if tag unseen this long
DOCK_HZ      = 50       # Hz

DOCKED_CONFIRM_TICKS = 10   # ~0.2 s at 50 Hz

# ── Backup parameters ─────────────────────────────────────────────────────────
BACKUP_SPEED    = 0.05  # m/s
BACKUP_DURATION = 2.0   # s


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command',  self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',             10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self.state         = 'idle'
        self.target_type   = None
        self.last_det      = None
        self.last_det_t    = time.monotonic()
        self._backup_start = None

        self._docked_count = 0

        self._last_status   = ''
        self._last_status_t = 0.0

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev9) ready  '
            f'DOCK_DIST={DOCK_DISTANCE}m  MAX_VX={MAX_VX}  '
            f'KP_DIST={KP_DIST}  KP_YAW={KP_YAW}  LOST_TO={LOST_TIMEOUT}s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'dock_static':
            self.target_type   = 'static'
            self.state         = 'docking'
            self.last_det      = None
            self.last_det_t    = time.monotonic()
            self._docked_count = 0
            self._stop()
            self.get_logger().info('Dock cmd: STATIC')

        elif cmd == 'dock_dynamic':
            # REV-9E: dynamic docking uses the exact same P-controller logic
            # as static — only the target_type filter differs.
            self.target_type   = 'dynamic_dock'
            self.state         = 'docking'
            self.last_det      = None
            self.last_det_t    = time.monotonic()
            self._docked_count = 0
            self._stop()
            self.get_logger().info('Dock cmd: DYNAMIC (same P-controller as static)')

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
            self._pub_status(self.state)
            return

        if self.state in ('idle', 'docked'):
            self._pub_status(self.state)
            return

        if self.state != 'docking':
            return

        # DOCKING ─────────────────────────────────────────────────────────────
        tag_age = time.monotonic() - self.last_det_t

        if self.last_det is None or tag_age > LOST_TIMEOUT:
            self._stop()
            self._docked_count = 0
            if tag_age > LOST_TIMEOUT * 2:
                self.get_logger().warn(
                    f'Tag lost {tag_age:.1f} s', throttle_duration_sec=2.0)
                self._pub_status('lost')
            else:
                self._pub_status('docking')
            return

        tz       = self.last_det['tz']
        yaw_off  = self.last_det['yaw_offset']
        dist_err = tz - DOCK_DISTANCE

        # Require N consecutive ticks inside both tolerances ──────────────────
        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_TOLERANCE:
            self._docked_count += 1
            if self._docked_count >= DOCKED_CONFIRM_TICKS:
                self._stop()
                self.state = 'docked'
                self.get_logger().info(
                    f'DOCKED  tz={tz:.3f} m  yaw={math.degrees(yaw_off):.1f}°')
                self._pub_status('docked')
                return
        else:
            self._docked_count = 0

        # Drive toward tag ────────────────────────────────────────────────────
        vx = KP_DIST * max(0.0, dist_err)   # forward only, no reverse
        wz = -KP_YAW * yaw_off              # turn toward tag centre

        cmd = Twist()
        cmd.linear.x  = min(MAX_VX, vx)
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))
        self.cmd_pub.publish(cmd)

        self._pub_status('docking')
        self.get_logger().info(
            f'[{self.target_type}] tz={tz:.2f}m  yaw={math.degrees(yaw_off):.1f}°  '
            f'vx={cmd.linear.x:.2f}  wz={cmd.angular.z:.2f}  '
            f'confirm={self._docked_count}/{DOCKED_CONFIRM_TICKS}',
            throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_status(self, s: str):
        """Only publish on state change or every 0.5 s heartbeat."""
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
