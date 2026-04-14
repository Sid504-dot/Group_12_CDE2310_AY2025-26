#!/usr/bin/env python3
"""
dock_controller_revised6.py — Runs on LAPTOP (needs TF + Nav2)
===============================================================
Nav2-based docking: no visual-servoing loop, no oscillation.

Strategy:
  1. Dock command received → enter 'waiting_for_tag'
  2. First tag detection → compute tag position in map frame ONCE
  3. Send a single NavigateToPose goal to Nav2 (robot to 5 cm in front of tag)
  4. Nav2 drives the robot there smoothly (MPPI trajectory, no hunting)
  5. Nav2 success → publish 'docked'

Why this works better than visual servoing:
  - MPPI plans an optimal forward trajectory; it does not oscillate
  - Tag position is fixed in map frame — even if the tag temporarily goes
    out of camera view during approach, the robot still drives to the right place
  - No cmd_vel fighting between Nav2 residual and our own controller

Backup still uses direct cmd_vel (simple, no Nav2 action needed).
"""

import json
import math
import time

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import tf2_ros

# ── Parameters ────────────────────────────────────────────────────────────────
DOCK_DISTANCE   = 0.05    # m — stop when camera is this close to tag face
DOCK_HZ         = 10      # Hz — tick rate (used mainly for backup)
TAG_AGE_LIMIT   = 5.0     # s — reject tag detections older than this
NAV_TIMEOUT_S   = 40.0    # s — abort if Nav2 doesn't finish within this time

BACKUP_SPEED    = 0.05    # m/s backward
BACKUP_DURATION = 2.0     # s


# ── Helpers ───────────────────────────────────────────────────────────────────

def _yaw_from_q(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _q_from_yaw(yaw) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ── Node ──────────────────────────────────────────────────────────────────────

class DockController(Node):

    def __init__(self):
        super().__init__('dock_controller')

        self.create_subscription(String, '/apriltag/detections', self._det_cb, 10)
        self.create_subscription(String, '/mission/dock_command', self._cmd_cb, 10)

        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/dock_status', 10)

        self._nav_client  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # State
        self.state         = 'idle'
        self.target_type   = None
        self.last_det      = None
        self.last_det_t    = 0.0
        self._goal_handle  = None
        self._goal_sent    = False
        self._nav_start_t  = None
        self._backup_start = None

        self.create_timer(1.0 / DOCK_HZ, self._tick)
        self.get_logger().info(
            f'DockController (Nav2-based) ready — DOCK_DISTANCE={DOCK_DISTANCE} m')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd in ('dock_static', 'dock_dynamic'):
            self._cancel_nav()
            self.target_type = 'static' if cmd == 'dock_static' else 'dynamic_dock'
            self.state       = 'waiting_for_tag'
            self.last_det    = None
            self.last_det_t  = time.monotonic()   # timer starts now
            self._goal_sent  = False
            self._stop()
            self.get_logger().info(f'Dock cmd: {cmd} — waiting for tag')

        elif cmd == 'backup':
            self._cancel_nav()
            self._stop()
            self._backup_start = time.monotonic()
            self.state = 'backing_up'
            self.get_logger().info(f'Backup — reversing {BACKUP_DURATION} s')

        elif cmd == 'cancel':
            self._cancel_nav()
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

        # BACKUP ──────────────────────────────────────────────────────────────
        if self.state == 'backing_up':
            if time.monotonic() - self._backup_start >= BACKUP_DURATION:
                self._stop()
                self.state = 'idle'
                self._pub_status('backup_done')
            else:
                t = Twist()
                t.linear.x = -BACKUP_SPEED
                self.cmd_pub.publish(t)
            return

        # NAVIGATING — Nav2 driving to dock position ───────────────────────────
        if self.state == 'navigating':
            # Nav2 result arrives via callback; just check for timeout here
            if self._nav_start_t and time.monotonic() - self._nav_start_t > NAV_TIMEOUT_S:
                self.get_logger().warn('Nav2 dock goal timed out — aborting')
                self._cancel_nav()
                self.state = 'idle'
                self._pub_status('lost')
            return

        # IDLE / DOCKED ────────────────────────────────────────────────────────
        if self.state in ('idle', 'docked'):
            return

        # WAITING_FOR_TAG ─────────────────────────────────────────────────────
        if self.state != 'waiting_for_tag':
            return

        if self._goal_sent:
            return   # goal already in flight

        if self.last_det is None:
            return   # haven't seen the tag yet

        age = time.monotonic() - self.last_det_t
        if age > TAG_AGE_LIMIT:
            self.get_logger().warn(
                f'Last tag detection is {age:.1f} s old — waiting for fresher read',
                throttle_duration_sec=2.0)
            return

        # Tag is fresh — compute map-frame goal and send to Nav2
        goal = self._compute_goal(self.last_det)
        if goal is None:
            return   # TF not ready yet

        self._send_nav_goal(goal)

    # ── Goal computation ──────────────────────────────────────────────────────

    def _compute_goal(self, tag):
        """
        Compute the docking goal in map frame.

        The robot must end up with its camera DOCK_DISTANCE (0.05 m) from
        the tag face.  The base_footprint needs to move (tz - DOCK_DISTANCE)
        metres forward at the bearing toward the tag.  Camera-forward offset
        from base_footprint cancels out of the math.

        Returns (x, y, yaw) in map frame, or None if TF is unavailable.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=1.0)
            return None

        rx  = tf.transform.translation.x
        ry  = tf.transform.translation.y
        rth = _yaw_from_q(tf.transform.rotation)

        tz      = tag['tz']          # camera-to-tag forward depth (m)
        yaw_off = tag['yaw_offset']  # positive = tag is to robot's right

        # Bearing from robot to tag in map frame.
        # yaw_off > 0 (right) means tag is clockwise from heading → bearing = rth - yaw_off
        bearing   = rth - yaw_off
        move_dist = max(0.02, tz - DOCK_DISTANCE)

        gx = rx + move_dist * math.cos(bearing)
        gy = ry + move_dist * math.sin(bearing)

        self.get_logger().info(
            f'Dock goal: ({gx:.2f}, {gy:.2f}) yaw={math.degrees(bearing):.1f}°  '
            f'tz={tz:.2f} m  move={move_dist:.2f} m  '
            f'robot=({rx:.2f},{ry:.2f}) rth={math.degrees(rth):.1f}°')
        return gx, gy, bearing

    def _send_nav_goal(self, goal):
        if not self._nav_client.wait_for_server(timeout_sec=1.5):
            self.get_logger().error('Nav2 action server not available')
            self._pub_status('lost')
            return

        ps = PoseStamped()
        ps.header.frame_id  = 'map'
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = goal[0]
        ps.pose.position.y  = goal[1]
        ps.pose.orientation = _q_from_yaw(goal[2])

        g      = NavigateToPose.Goal()
        g.pose = ps

        self._goal_sent   = True
        self._nav_start_t = time.monotonic()
        self.state        = 'navigating'

        fut = self._nav_client.send_goal_async(g)
        fut.add_done_callback(self._goal_resp_cb)
        self.get_logger().info('NavigateToPose goal sent for docking')

    # ── Nav2 action callbacks ─────────────────────────────────────────────────

    def _goal_resp_cb(self, fut):
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 docking goal REJECTED')
            self._goal_sent = False
            self.state = 'idle'
            self._pub_status('lost')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, fut):
        res = fut.result()
        if res.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('DOCKED — Nav2 reached docking position')
            self._stop()
            self.state = 'docked'
            self._pub_status('docked')
        else:
            self.get_logger().warn(
                f'Nav2 docking goal failed (status={res.status})')
            self.state = 'idle'
            self._pub_status('lost')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _cancel_nav(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._goal_sent   = False
        self._nav_start_t = None

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
