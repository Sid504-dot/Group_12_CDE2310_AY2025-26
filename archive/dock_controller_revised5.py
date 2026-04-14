#!/usr/bin/env python3
"""
dock_controller_revised5.py — Runs on RPi
==========================================
Control logic:
  - On dock command: 0.5 s settle pause so any residual Nav2 cmd_vel clears.
  - Within CONE_DEG (5°) of tag centre: drive straight forward only, no rotation.
    This gives a clean straight-line approach when roughly aligned.
  - Outside CONE_DEG: rotate in place to face tag, no forward motion.
    This avoids the robot curving/spiralling and ending up sideways.
  - Uses tag['tz'] (forward depth) not tag['dist'] (3-D norm) for distance
    control so camera height offset doesn't make DOCK_DISTANCE unreachable.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Docking parameters ───────────────────────────────────────────────────────
DOCK_DISTANCE  = 0.05   # metres forward depth (tz) to stop from tag face
DIST_TOLERANCE = 0.04   # metres — docked when within this of DOCK_DISTANCE
YAW_TOLERANCE  = 0.06   # radians (~3.4°) — docked when within this of 0

CONE_DEG = 5.0          # degree half-angle of "drive forward" cone
CONE_RAD = math.radians(CONE_DEG)

KP_YAW  = 1.0           # angular gain (rad/s per rad error)
KP_DIST = 0.6           # linear gain  (m/s per m error)
MAX_VX  = 0.15          # m/s forward speed cap
MAX_WZ  = 0.8           # rad/s angular speed cap

LOST_TIMEOUT  = 3.0     # seconds without tag before publishing 'lost'
SETTLE_S      = 0.5     # seconds to hold still after dock command (Nav2 drain)
DOCK_HZ       = 15

# ── Backup parameters ─────────────────────────────────────────────────────────
BACKUP_SPEED    = 0.05   # m/s backward
BACKUP_DURATION = 2.0    # seconds


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
        self._dock_init_t  = None   # time dock command received (for settle period)

        self.create_timer(1.0 / DOCK_HZ, self._control_tick)
        self.get_logger().info(
            f'DockController ready — DOCK_DISTANCE={DOCK_DISTANCE} m  '
            f'CONE={CONE_DEG}°  SETTLE={SETTLE_S}s')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'dock_static':
            self.target_type  = 'static'
            self.state        = 'docking'
            self.last_det     = None
            self.last_det_t   = time.monotonic()
            self._dock_init_t = time.monotonic()
            self._stop()   # immediately kill any residual velocity
            self.get_logger().info('Docking command: STATIC target (settling...)')

        elif cmd == 'dock_dynamic':
            self.target_type  = 'dynamic_dock'
            self.state        = 'docking'
            self.last_det     = None
            self.last_det_t   = time.monotonic()
            self._dock_init_t = time.monotonic()
            self._stop()
            self.get_logger().info('Docking command: DYNAMIC target (settling...)')

        elif cmd == 'backup':
            self._stop()
            self._backup_start = time.monotonic()
            self.state = 'backing_up'
            self.get_logger().info(f'Backup command — reversing {BACKUP_DURATION}s')

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
    # Control loop — DOCK_HZ
    # =========================================================================

    def _control_tick(self):
        status_msg      = String()
        status_msg.data = self.state
        self.status_pub.publish(status_msg)

        # ── BACKING_UP ────────────────────────────────────────────────────────
        if self.state == 'backing_up':
            elapsed = time.monotonic() - self._backup_start
            if elapsed >= BACKUP_DURATION:
                self._stop()
                self.state = 'idle'
                self.get_logger().info('Backup complete')
                status_msg.data = 'backup_done'
                self.status_pub.publish(status_msg)
            else:
                cmd = Twist()
                cmd.linear.x = -BACKUP_SPEED
                self.cmd_pub.publish(cmd)
            return

        if self.state in ('idle', 'docked'):
            return

        if self.state != 'docking':
            return

        # ── SETTLE: hold still for SETTLE_S after dock command ────────────────
        # Nav2's MPPI may still publish cmd_vel for ~1 tick after cancel.
        # Holding zero here drains that before visual servoing starts.
        if self._dock_init_t is not None:
            if time.monotonic() - self._dock_init_t < SETTLE_S:
                self._stop()
                return
            self._dock_init_t = None   # settle done — won't check again

        now         = time.monotonic()
        time_no_tag = now - self.last_det_t

        # Tag not visible — hold still and wait
        if self.last_det is None or time_no_tag > LOST_TIMEOUT:
            self._stop()
            if time_no_tag > LOST_TIMEOUT * 2:
                self.get_logger().warn(
                    f'Tag lost for {time_no_tag:.1f}s', throttle_duration_sec=2.0)
                status_msg.data = 'lost'
                self.status_pub.publish(status_msg)
            return

        tag      = self.last_det
        yaw_err  = tag['yaw_offset']   # positive = tag is to the right
        depth    = tag['tz']           # forward depth (m) — not 3-D norm
        dist_err = depth - DOCK_DISTANCE

        # ── DOCKED check ──────────────────────────────────────────────────────
        if abs(yaw_err) < YAW_TOLERANCE and abs(dist_err) < DIST_TOLERANCE:
            self._stop()
            self.state = 'docked'
            self.get_logger().info(
                f'DOCKED — tz={depth:.3f} m  yaw={math.degrees(yaw_err):.1f}°')
            status_msg.data = 'docked'
            self.status_pub.publish(status_msg)
            return

        # ── CONE-BASED SERVOING ───────────────────────────────────────────────
        #
        #   Within 5° of tag centre → drive straight forward only.
        #     No rotation: avoids wobble and spiralling.
        #
        #   Outside 5° → rotate in place to face tag only.
        #     No forward motion: avoids robot ending up sideways.
        #
        cmd = Twist()

        if abs(yaw_err) <= CONE_RAD:
            # ── ALIGNED: straight forward approach ────────────────────────────
            vx = KP_DIST * dist_err
            cmd.linear.x  = max(-MAX_VX, min(MAX_VX, vx))
            cmd.angular.z = 0.0
            self.get_logger().info(
                f'→ APPROACH  tz={depth:.2f}m  yaw={math.degrees(yaw_err):.1f}°',
                throttle_duration_sec=0.5)
        else:
            # ── MISALIGNED: rotate in place toward tag ────────────────────────
            wz = -KP_YAW * yaw_err
            cmd.angular.z = max(-MAX_WZ, min(MAX_WZ, wz))
            cmd.linear.x  = 0.0
            self.get_logger().info(
                f'↺ ALIGN  yaw={math.degrees(yaw_err):.1f}°  tz={depth:.2f}m',
                throttle_duration_sec=0.5)

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
