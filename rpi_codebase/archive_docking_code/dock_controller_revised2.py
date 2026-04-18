#!/usr/bin/env python3
"""
dock_controller_revised2.py — Runs on RPi
==========================================
Simplest possible docking: when tag detected, drive straight toward it.

Control (both axes always active — no mode switching, no cone):
  vx = KP_DIST * (tz - DOCK_DISTANCE)   forward-only proportional
  wz = -KP_YAW * yaw_offset             angular correction toward tag centre

The robot naturally curves toward the tag and slows as it approaches.
No switching = no oscillation.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ────────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.05   # m — stop this far (forward depth) from tag face
DIST_TOLERANCE = 0.03   # m — ±3 cm counts as "at dock distance"
YAW_TOLERANCE  = 0.12   # rad (~7°) — counts as "aligned"

KP_DIST = 0.40          # linear gain   (m/s per m error)
KP_YAW  = 0.70          # angular gain  (rad/s per rad error)
MAX_VX  = 0.10          # m/s forward cap  (slow and steady)
MAX_WZ  = 0.50          # rad/s angular cap

LOST_TIMEOUT = 3.0      # s — hold still if tag unseen this long
DOCK_HZ      = 50       # Hz — faster than Nav2's 30Hz so dock cmd_vel wins

# ── Backup parameters ─────────────────────────────────────────────────────────
BACKUP_SPEED    = 0.05  # m/s
BACKUP_DURATION = 2.0   # s


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command', self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self.state         = 'idle'
        self.target_type   = None
        self.last_det      = None
        self.last_det_t    = time.monotonic()
        self._backup_start = None

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController ready  DOCK_DISTANCE={DOCK_DISTANCE} m')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'dock_static':
            self.target_type = 'static'
            self.state       = 'docking'
            self.last_det    = None
            self.last_det_t  = time.monotonic()
            self._stop()
            self.get_logger().info('Dock cmd: STATIC')

        elif cmd == 'dock_dynamic':
            self.target_type = 'dynamic_dock'
            self.state       = 'docking'
            self.last_det    = None
            self.last_det_t  = time.monotonic()
            self._stop()
            self.get_logger().info('Dock cmd: DYNAMIC')

        elif cmd == 'backup':
            self._stop()
            self._backup_start = time.monotonic()
            self.state = 'backing_up'
            self.get_logger().info(f'Backup — reversing {BACKUP_DURATION} s')

        elif cmd == 'cancel':
            self.state       = 'idle'
            self.target_type = None
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

    # ── Control tick ─────────────────────────────────────────────────────────

    def _tick(self):
        self._pub_status(self.state)

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
            return

        if self.state != 'docking':
            return

        # DOCKING ─────────────────────────────────────────────────────────────
        tag_age = time.monotonic() - self.last_det_t

        # Tag not visible — hold still and wait
        if self.last_det is None or tag_age > LOST_TIMEOUT:
            self._stop()
            if tag_age > LOST_TIMEOUT * 2:
                self.get_logger().warn(
                    f'Tag lost {tag_age:.1f} s', throttle_duration_sec=2.0)
                self._pub_status('lost')
            return

        tz       = self.last_det['tz']          # forward depth to tag (m)
        yaw_off  = self.last_det['yaw_offset']  # positive = tag is to the right
        dist_err = tz - DOCK_DISTANCE

        # DOCKED check ────────────────────────────────────────────────────────
        if abs(dist_err) < DIST_TOLERANCE and abs(yaw_off) < YAW_TOLERANCE:
            self._stop()
            self.state = 'docked'
            self.get_logger().info(
                f'DOCKED  tz={tz:.3f} m  yaw={math.degrees(yaw_off):.1f}°')
            self._pub_status('docked')
            return

        # Drive toward tag — linear + angular simultaneously, no switching ────
        vx = KP_DIST * max(0.0, dist_err)   # forward only, clamp out reverse
        wz = -KP_YAW * yaw_off              # negative = turn right when tag is right

        cmd = Twist()
        cmd.linear.x  = min(MAX_VX, vx)
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))
        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'tz={tz:.2f}m  yaw={math.degrees(yaw_off):.1f}°  '
            f'vx={cmd.linear.x:.2f}  wz={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _pub_status(self, s: str):
        m = String()
        m.data = s
        self.status_pub.publish(m)


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
