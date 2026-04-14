#!/usr/bin/env python3
"""
dock_controller_revised3.py — Runs on RPi
==========================================
Changes vs dock_controller_revised2.py:
  REV-3A  Search spin REMOVED entirely. If tag not visible, robot holds
          still and waits. No rotation at all unless visually commanded.
  REV-3B  Speeds increased: MAX_VX 0.10->0.18, MAX_WZ 0.6->0.9
          KP_DIST 0.4->0.8 so it reaches 10cm faster.
  REV-3C  After 'docked' is published, robot stays stopped. dock_controller
          does nothing further until a new dock command arrives.
"""

import json
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ──────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.10   # metres from tag face
YAW_TOLERANCE  = 0.06   # radians (~3.4 deg)
DIST_TOLERANCE = 0.05   # metres

# PID gains — REV-3B: increased for faster approach
KP_YAW  = 1.2
KP_DIST = 0.8   # was 0.4

MAX_VX  = 0.18  # m/s — was 0.10
MAX_WZ  = 0.9   # rad/s — was 0.6

LOST_TIMEOUT = 3.0   # seconds before publishing 'lost'
DOCK_HZ      = 15


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command', self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self.state       = 'idle'
        self.target_type = None
        self.last_det    = None
        self.last_det_t  = time.monotonic()

        self.create_timer(1.0 / DOCK_HZ, self._control_tick)
        self.get_logger().info(
            f'DockController (rev3) ready — DOCK_DISTANCE={DOCK_DISTANCE} m, '
            f'MAX_VX={MAX_VX}, MAX_WZ={MAX_WZ}')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'dock_static':
            self.target_type = 'static'
            self.state       = 'docking'
            self.last_det    = None
            self.last_det_t  = time.monotonic()
            self.get_logger().info('Docking command: STATIC target')
        elif cmd == 'dock_dynamic':
            self.target_type = 'dynamic_dock'
            self.state       = 'docking'
            self.last_det    = None
            self.last_det_t  = time.monotonic()
            self.get_logger().info('Docking command: DYNAMIC target')
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

    # =========================================================================
    # Control loop — 15 Hz
    # =========================================================================

    def _control_tick(self):
        status_msg      = String()
        status_msg.data = self.state
        self.status_pub.publish(status_msg)

        if self.state in ('idle', 'docked'):
            # REV-3C: stay stopped, do nothing
            return

        if self.state != 'docking':
            return

        now         = time.monotonic()
        time_no_tag = now - self.last_det_t

        # Tag not visible — REV-3A: hold still, just wait
        if self.last_det is None or time_no_tag > LOST_TIMEOUT:
            self._stop()   # no spin, no search — just freeze
            if time_no_tag > LOST_TIMEOUT * 2:
                self.get_logger().warn(
                    f'Tag lost for {time_no_tag:.1f}s — publishing lost')
                status_msg.data = 'lost'
                self.status_pub.publish(status_msg)
            return

        tag      = self.last_det
        yaw_err  = tag['yaw_offset']
        dist_err = tag['dist'] - DOCK_DISTANCE   # positive = too far

        # Aligned and at correct distance — done
        if abs(yaw_err) < YAW_TOLERANCE and abs(dist_err) < DIST_TOLERANCE:
            self._stop()
            self.state = 'docked'
            self.get_logger().info(
                f'DOCKED — dist={tag["dist"]:.3f} m  yaw={yaw_err:.3f} rad')
            status_msg.data = 'docked'
            self.status_pub.publish(status_msg)
            return

        # Visual servoing — yaw always, linear only when roughly facing tag
        cmd = Twist()
        wz  = -KP_YAW * yaw_err
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))

        if abs(yaw_err) < 0.2:   # within ~11 deg — also correct distance
            vx = KP_DIST * dist_err
            cmd.linear.x = max(-MAX_VX, min(MAX_VX, vx))

        self.cmd_pub.publish(cmd)

    # =========================================================================
    # Helpers
    # =========================================================================

    def _stop(self):
        self.cmd_pub.publish(Twist())


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
