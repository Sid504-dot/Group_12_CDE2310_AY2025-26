#!/usr/bin/env python3
"""
dock_controller_revised1.py — Runs on RPi
==========================================
Changes vs dock_controller.py:
  REV-1A  DOCK_DISTANCE reduced from 0.35 m to 0.10 m — robot stops
          10 cm from the tag face before signalling "docked".
  REV-1B  DIST_TOLERANCE loosened from 0.03 m to 0.05 m — easier to
          settle at close range where small distance errors are larger
          in proportion.

Everything else (PID gains, yaw tolerance, lost timeout) is unchanged.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ──────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.10   # metres from tag face  (REV-1A: was 0.35)
YAW_TOLERANCE  = 0.06   # radians (~3.4°)
DIST_TOLERANCE = 0.05   # metres               (REV-1B: was 0.03)

# PID-ish gains
KP_YAW  = 1.2
KP_DIST = 0.4

MAX_VX  = 0.10   # m/s cap during docking
MAX_WZ  = 0.6    # rad/s cap during docking

LOST_TIMEOUT = 3.0   # seconds without detection before "lost"
DOCK_HZ      = 15    # control loop rate


class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections',
                                 self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command',
                                 self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self.state       = 'idle'
        self.target_type = None      # 'static' | 'dynamic_dock'
        self.last_det    = None
        self.last_det_t  = 0.0

        self.create_timer(1.0 / DOCK_HZ, self._control_tick)
        self.get_logger().info(
            f'DockController (rev1) ready — DOCK_DISTANCE={DOCK_DISTANCE} m')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'dock_static':
            self.target_type = 'static'
            self.state = 'docking'
            self.get_logger().info('Docking command: STATIC target')
        elif cmd == 'dock_dynamic':
            self.target_type = 'dynamic_dock'
            self.state = 'docking'
            self.get_logger().info('Docking command: DYNAMIC target')
        elif cmd == 'cancel':
            self.state = 'idle'
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
        status_msg = String()
        status_msg.data = self.state
        self.status_pub.publish(status_msg)

        if self.state != 'docking':
            return

        now = time.monotonic()

        # Tag lost — slow search spin
        if self.last_det is None or (now - self.last_det_t) > LOST_TIMEOUT:
            cmd = Twist()
            cmd.angular.z = 0.3
            self.cmd_pub.publish(cmd)

            if self.last_det is not None and (now - self.last_det_t) > LOST_TIMEOUT * 2:
                self.get_logger().warn('Tag lost during docking')
                status_msg.data = 'lost'
                self.status_pub.publish(status_msg)
            return

        tag      = self.last_det
        yaw_err  = tag['yaw_offset']          # positive = tag to the right
        dist_err = tag['dist'] - DOCK_DISTANCE # positive = too far away

        # Aligned and at correct distance — done
        if abs(yaw_err) < YAW_TOLERANCE and abs(dist_err) < DIST_TOLERANCE:
            self._stop()
            self.state = 'docked'
            self.get_logger().info(
                f'DOCKED — dist={tag["dist"]:.3f} m, yaw_off={yaw_err:.3f} rad')
            status_msg.data = 'docked'
            self.status_pub.publish(status_msg)
            return

        # Visual servoing
        cmd = Twist()

        wz = -KP_YAW * yaw_err
        wz = max(-MAX_WZ, min(MAX_WZ, wz))
        cmd.angular.z = wz

        if abs(yaw_err) < 0.2:   # ~11 deg — close enough to also fix distance
            vx = KP_DIST * dist_err
            vx = max(-MAX_VX, min(MAX_VX, vx))
            cmd.linear.x = vx

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
