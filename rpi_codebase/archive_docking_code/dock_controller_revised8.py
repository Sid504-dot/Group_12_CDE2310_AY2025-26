#!/usr/bin/env python3
"""
dock_controller_revised8.py — Runs on RPi
==========================================
Changes vs dock_controller_revised7.py:
  REV-8A  DOCK_DISTANCE increased 0.05 → 0.20 m so the tag stays fully
          in frame during the final approach. At 5 cm a 10 cm tag fills
          most of the frame and corners clip off-screen, causing the
          detector to silently drop the tag and the controller to hang.
  REV-8B  DIST_TOLERANCE widened 0.03 → 0.05 m to match the looser
          stopping distance and reduce oscillation near the target.
  REV-8C  Hysteresis counter added to the docked check. Both tolerances
          must be satisfied for 10 consecutive ticks (~0.2 s at 50 Hz)
          before declaring docked. Prevents false-positive exits caused
          by the proportional controller oscillating through the threshold.
  REV-8D  _pub_status throttled — was publishing at 50 Hz every tick,
          flooding the topic and starving the mission coordinator's
          callback queue. Now only publishes on state change or every
          0.5 s as a heartbeat.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ────────────────────────────────────────────────────────
# REV-8A: increased from 0.05 → 0.20 m so tag stays in frame
DOCK_DISTANCE  = 0.20   # m — stop this far (forward depth) from tag face
# REV-8B: widened from 0.03 → 0.05 m
DIST_TOLERANCE = 0.05   # m — ±5 cm counts as "at dock distance"
YAW_TOLERANCE  = 0.12   # rad (~7°) — counts as "aligned"

KP_DIST = 0.40          # linear gain   (m/s per m error)
KP_YAW  = 0.70          # angular gain  (rad/s per rad error)
MAX_VX  = 0.10          # m/s forward cap
MAX_WZ  = 0.50          # rad/s angular cap

LOST_TIMEOUT = 3.0      # s — hold still if tag unseen this long
DOCK_HZ      = 50       # Hz

# REV-8C: how many consecutive ticks both tolerances must hold before docked
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

        # REV-8C: hysteresis counter
        self._docked_count = 0

        # REV-8D: throttle status publishing
        self._last_status   = ''
        self._last_status_t = 0.0

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (rev8) ready  DOCK_DISTANCE={DOCK_DISTANCE} m')

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
            self.target_type   = 'dynamic_dock'
            self.state         = 'docking'
            self.last_det      = None
            self.last_det_t    = time.monotonic()
            self._docked_count = 0
            self._stop()
            self.get_logger().info('Dock cmd: DYNAMIC')

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

        # REV-8C: require N consecutive ticks inside both tolerances ──────────
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
            self._docked_count = 0   # reset if robot drifts back out

        # Drive toward tag ────────────────────────────────────────────────────
        vx = KP_DIST * max(0.0, dist_err)   # forward only, no reverse
        wz = -KP_YAW * yaw_off              # turn toward tag centre

        cmd = Twist()
        cmd.linear.x  = min(MAX_VX, vx)
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))
        self.cmd_pub.publish(cmd)

        self._pub_status('docking')
        self.get_logger().info(
            f'tz={tz:.2f}m  yaw={math.degrees(yaw_off):.1f}°  '
            f'vx={cmd.linear.x:.2f}  wz={cmd.angular.z:.2f}  '
            f'confirm={self._docked_count}/{DOCKED_CONFIRM_TICKS}',
            throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_status(self, s: str):
        """REV-8D: only publish on state change or every 0.5 s heartbeat."""
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
