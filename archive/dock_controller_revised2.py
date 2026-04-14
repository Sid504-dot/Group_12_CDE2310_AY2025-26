#!/usr/bin/env python3
"""
dock_controller_revised2.py — Runs on RPi
==========================================
Changes vs dock_controller_revised1.py:
  REV-2A  self.last_det_t initialised to time.monotonic() instead of 0.0.
          In revised1, last_det_t=0.0 meant (now - 0.0) > LOST_TIMEOUT was
          True immediately on the first tick, causing the robot to spin at
          0.3 rad/s before it ever saw a tag. This knocked the robot off
          axis, the tag left the frame, and the coordinator saw 'lost' and
          reset — causing the EXPLORING <-> DOCKING_STATIC flicker.
          Fix: initialise to now() so the lost-timer only starts counting
          from node startup, not from epoch 0.
  REV-2B  DOCK_DISTANCE kept at 0.10 m (10 cm from tag face).
  REV-2C  Stop spinning to search when tag not yet seen (state='docking' but
          last_det is None AND less than LOST_TIMEOUT has passed) — just
          hold position and wait. Only start slow search spin after
          LOST_TIMEOUT seconds with no detection.

Alignment behaviour (confirmed working):
  - yaw_offset from apriltag pose = horizontal angle from camera centre to tag.
  - ang = -KP_YAW * yaw_offset rotates robot until it faces tag head-on
    (perpendicular to the tag face).
  - Linear correction only engages once |yaw_err| < 0.2 rad (~11 deg).
  - Robot will both rotate AND drive to reach: facing tag + 10 cm away.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ──────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.10   # metres from tag face
YAW_TOLERANCE  = 0.06   # radians (~3.4 deg)
DIST_TOLERANCE = 0.05   # metres

# PID-ish gains
KP_YAW  = 1.2
KP_DIST = 0.4

MAX_VX  = 0.10   # m/s cap
MAX_WZ  = 0.6    # rad/s cap

LOST_TIMEOUT = 3.0   # seconds without detection before starting search spin
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
        self.last_det_t  = time.monotonic()   # REV-2A: not 0.0

        self.create_timer(1.0 / DOCK_HZ, self._control_tick)
        self.get_logger().info(
            f'DockController (rev2) ready — DOCK_DISTANCE={DOCK_DISTANCE} m')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'dock_static':
            self.target_type = 'static'
            self.state       = 'docking'
            self.last_det    = None
            self.last_det_t  = time.monotonic()   # reset timer on new command
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

        if self.state != 'docking':
            return

        now          = time.monotonic()
        time_no_tag  = now - self.last_det_t

        # REV-2C: tag not yet seen / recently lost
        if self.last_det is None or time_no_tag > LOST_TIMEOUT:
            if time_no_tag > LOST_TIMEOUT:
                # Slow search spin only after timeout — not immediately
                cmd = Twist()
                cmd.angular.z = 0.25
                self.cmd_pub.publish(cmd)

            if time_no_tag > LOST_TIMEOUT * 2:
                self.get_logger().warn(
                    f'Tag lost for {time_no_tag:.1f}s — publishing lost')
                status_msg.data = 'lost'
                self.status_pub.publish(status_msg)
            return

        tag      = self.last_det
        yaw_err  = tag['yaw_offset']
        dist_err = tag['dist'] - DOCK_DISTANCE   # positive = too far

        # Aligned and at distance — done
        if abs(yaw_err) < YAW_TOLERANCE and abs(dist_err) < DIST_TOLERANCE:
            self._stop()
            self.state = 'docked'
            self.get_logger().info(
                f'DOCKED — dist={tag["dist"]:.3f} m  yaw={yaw_err:.3f} rad')
            status_msg.data = 'docked'
            self.status_pub.publish(status_msg)
            return

        # Visual servoing: yaw always, linear only when roughly aligned
        cmd = Twist()
        wz  = -KP_YAW * yaw_err
        cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))

        if abs(yaw_err) < 0.2:
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
