#!/usr/bin/env python3
"""
mission_coordinator_v3.py  (runs on laptop / remote PC)
========================================================
Changes vs mission_coordinator_v2.py:

  V3-A  Removed the static_done gate on dynamic docking.

        OLD (v2): dynamic docking only activates after the full static
        sequence (dock + fire + backup) completes. If the dynamic marker
        is seen first, it is silently ignored until static is done.

        NEW (v3): as soon as ANY marker (static OR dynamic) is detected,
        the robot docks and fires. After completing that sequence it
        resumes navigation and searches for the remaining marker type.

        If both markers are visible at the same time, static is checked
        first (conservative priority order). Change the check order below
        in _tick() / EXPLORING if a different priority is required.

  V3-B  Removed the V2-C "ignoring dynamic tag because static_done=False"
        log. That message is no longer valid now that the gate is gone.

Inherited from v2:
  V2-A  DOCK_LOST_DEBOUNCE_S = 8.0 s
  V2-B  TAG_STALE_S = 1.0 s
  COORD-BUG 1–5 fixes (pause idempotency, split BACKING_UP states, etc.)

State flow (order no longer determined by static-first rule):
  EXPLORING
    → first marker detected (static or dynamic)
  DOCKING_{TYPE}
    → dock_controller servos to target, publishes 'docked'
  FIRING_{TYPE}
    → launcher fires, publishes 'static_done' or 'dynamic_fired_N'
  BACKING_UP_{TYPE}
    → dock_controller reverses, publishes 'backup_done'
  EXPLORING  (completed type ignored, search continues for the other type)
  DONE when both static_done=True and dynamic_done=True
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COORDINATOR_HZ       = 20
DETECTION_RANGE_M    = 1.5    # ignore tags farther than this
DYNAMIC_BALL_COUNT   = 3

DOCK_LOST_DEBOUNCE_S      = 8.0    # 'lost' must persist this long before aborting dock
WAITING_DYNAMIC_TIMEOUT_S = 30.0   # abort if receptacle not seen within this many seconds
DOCK_TIMEOUT_S            = 45.0   # abort dock+fire cycle if no ball launched within this

TAG_STALE_S       = 1.0

TEST_MODE         = False
STATIC_COOLDOWN_S = 10.0


class MissionCoordinator(Node):

    EXPLORING          = 'EXPLORING'
    DOCKING_STATIC     = 'DOCKING_STATIC'
    FIRING_STATIC      = 'FIRING_STATIC'
    BACKING_UP_STATIC  = 'BACKING_UP_STATIC'
    COOLDOWN           = 'COOLDOWN'
    DOCKING_DYNAMIC    = 'DOCKING_DYNAMIC'
    WAITING_DYNAMIC    = 'WAITING_DYNAMIC'
    FIRING_DYNAMIC     = 'FIRING_DYNAMIC'
    BACKING_UP_DYNAMIC = 'BACKING_UP_DYNAMIC'
    DONE               = 'DONE'

    def __init__(self):
        super().__init__('mission_coordinator')

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(String, '/apriltag/detections',   self._det_cb,           10)
        self.create_subscription(String, '/mission/dock_status',   self._dock_status_cb,   10)
        self.create_subscription(String, '/mission/launch_status', self._launch_status_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.nav_cmd_pub    = self.create_publisher(String, '/mission/nav_command',    10)
        self.dock_cmd_pub   = self.create_publisher(String, '/mission/dock_command',   10)
        self.launch_cmd_pub = self.create_publisher(String, '/mission/launch_command', 10)
        self.ignore_pub     = self.create_publisher(String, '/mission/ignore_types',   10)
        self.state_pub      = self.create_publisher(String, '/mission/state',          10)

        # ── Mission state ─────────────────────────────────────────────────────
        self.state        = self.EXPLORING
        self.static_done  = False
        self.dynamic_done = False

        self.dock_status         = 'idle'
        self.launch_status       = ''
        self.dynamic_balls_fired = 0

        self.latest_tags   = []
        self.last_det_time = 0.0

        self._dock_sent         = False
        self._fire_sent         = False
        self._cooldown_t        = 0.0
        self._dock_lost_since   = None
        self._backup_done_latch        = False
        self._waiting_dynamic_start_t  = None
        self._dock_start_t             = None
        self._nav_settle_until         = 0.0

        self.create_timer(1.0 / COORDINATOR_HZ, self._tick)
        mode_str = 'TEST (re-trigger enabled)' if TEST_MODE else 'MISSION'
        self.get_logger().info(
            f'MissionCoordinator v3 started — state=EXPLORING  mode={mode_str}  '
            f'LOST_DEBOUNCE={DOCK_LOST_DEBOUNCE_S}s  TAG_STALE={TAG_STALE_S}s  '
            f'[V3: no static_done gate — first-seen target docked first]')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _det_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.latest_tags   = data.get('tags', [])
            self.last_det_time = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _dock_status_cb(self, msg: String):
        s = msg.data.strip().lower()
        if s == 'lost' and self.dock_status != 'lost':
            self._dock_lost_since = time.monotonic()
        elif s != 'lost':
            self._dock_lost_since = None
        if s == 'backup_done':
            self._backup_done_latch = True
        self.dock_status = s

    def _launch_status_cb(self, msg: String):
        self.launch_status = msg.data.strip().lower()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _pub(self, publisher, text: str):
        m      = String()
        m.data = text
        publisher.publish(m)

    def _pause_nav(self):
        self._pub(self.nav_cmd_pub, 'pause')

    def _resume_nav(self):
        self._pub(self.nav_cmd_pub, 'resume')

    def _set_ignore(self, *types):
        self._pub(self.ignore_pub, ','.join(types))

    def _find_tag(self, tag_type):
        """Return closest in-range tag of the given type, or None if stale."""
        if time.monotonic() - self.last_det_time > TAG_STALE_S:
            return None
        best = None
        for t in self.latest_tags:
            if t['type'] == tag_type and t['dist'] < DETECTION_RANGE_M:
                if best is None or t['dist'] < best['dist']:
                    best = t
        return best

    def _lost_too_long(self) -> bool:
        return (self._dock_lost_since is not None and
                time.monotonic() - self._dock_lost_since > DOCK_LOST_DEBOUNCE_S)

    def _dock_timed_out(self) -> bool:
        return (self._dock_start_t is not None and
                time.monotonic() - self._dock_start_t > DOCK_TIMEOUT_S)

    def _abort_dock_retry(self, label: str):
        self.get_logger().warn(
            f'{label}: dock timeout ({DOCK_TIMEOUT_S}s) — '
            f'cancelling and resuming nav (target NOT marked done, will retry)')
        self._pub(self.dock_cmd_pub, 'cancel')
        self._dock_start_t            = None
        self._dock_lost_since         = None
        self._waiting_dynamic_start_t = None
        self._resume_nav()
        self.state = self.EXPLORING

    def _start_docking(self, tag, target_state: str):
        """Shared helper: pause nav, set settle timer, transition state."""
        self.get_logger().info(
            f'Tag {tag["id"]} ({tag["type"]}) at {tag["dist"]:.2f}m'
            f' — pausing nav, settling 0.5s → {target_state}')
        self._pause_nav()
        self._dock_sent        = False
        self._dock_start_t     = time.monotonic()
        self._nav_settle_until = time.monotonic() + 0.5
        self.state = target_state

    # =========================================================================
    # Main tick — 20 Hz
    # =========================================================================

    def _tick(self):
        self._pub(self.state_pub, self.state)

        # ── EXPLORING ─────────────────────────────────────────────────────────
        if self.state == self.EXPLORING:
            self._dock_sent         = False
            self._fire_sent         = False
            self._backup_done_latch = False

            # V3-A: static and dynamic are checked independently.
            # Static has priority when both are simultaneously visible.
            if not self.static_done:
                tag = self._find_tag('static')
                if tag is not None:
                    self._start_docking(tag, self.DOCKING_STATIC)
                    return

            # V3-A: dynamic check has NO static_done gate (removed from v2)
            if not self.dynamic_done:
                tag = self._find_tag('dynamic_dock')
                if tag is not None:
                    self._start_docking(tag, self.DOCKING_DYNAMIC)
                    return

            if self.static_done and self.dynamic_done:
                self.get_logger().info('All targets complete — DONE')
                self.state = self.DONE

        # ── DOCKING_STATIC ────────────────────────────────────────────────────
        elif self.state == self.DOCKING_STATIC:
            self._pause_nav()

            if not self._dock_sent:
                if time.monotonic() < self._nav_settle_until:
                    return
                self._pub(self.dock_cmd_pub, 'dock_static')
                self._dock_sent = True
                self.get_logger().info('Nav settled — dock_static sent')
                return

            if self._dock_timed_out():
                self._abort_dock_retry('DOCKING_STATIC')
                return

            if self.dock_status == 'docked':
                self.get_logger().info('Docked to static — firing')
                self._pub(self.launch_cmd_pub, 'fire_static')
                self._fire_sent    = True
                self.launch_status = ''
                self.state = self.FIRING_STATIC

            elif self.dock_status == 'lost' and self._lost_too_long():
                self.get_logger().warn(
                    f'Static tag lost >{DOCK_LOST_DEBOUNCE_S}s — resuming nav')
                self._pub(self.dock_cmd_pub, 'cancel')
                self._resume_nav()
                self._dock_lost_since = None
                self.state = self.EXPLORING

        # ── FIRING_STATIC ─────────────────────────────────────────────────────
        elif self.state == self.FIRING_STATIC:
            self._pause_nav()

            if self._dock_timed_out():
                self._abort_dock_retry('FIRING_STATIC')
                return

            if 'static_done' in self.launch_status:
                self.get_logger().info('Static firing complete — backing up')
                self._backup_done_latch = False
                self._dock_start_t      = None
                self._pub(self.dock_cmd_pub, 'backup')
                self.state = self.BACKING_UP_STATIC

        # ── BACKING_UP_STATIC ─────────────────────────────────────────────────
        elif self.state == self.BACKING_UP_STATIC:
            self._pause_nav()

            if self._backup_done_latch:
                self.get_logger().info(
                    'Static backup done — ignoring static tags, resuming navigation')
                self.static_done = True

                if TEST_MODE:
                    self.get_logger().info(f'TEST MODE — cooldown {STATIC_COOLDOWN_S}s')
                    self._cooldown_t = time.monotonic()
                    self._resume_nav()
                    self.state = self.COOLDOWN
                else:
                    self._set_ignore('static')
                    self._resume_nav()
                    self.state = self.EXPLORING

        # ── COOLDOWN (test mode) ──────────────────────────────────────────────
        elif self.state == self.COOLDOWN:
            if time.monotonic() - self._cooldown_t >= STATIC_COOLDOWN_S:
                self.get_logger().info('Cooldown done — ready for next detection')
                self.launch_status = ''
                self.state = self.EXPLORING

        # ── DOCKING_DYNAMIC ───────────────────────────────────────────────────
        elif self.state == self.DOCKING_DYNAMIC:
            self._pause_nav()

            if not self._dock_sent:
                if time.monotonic() < self._nav_settle_until:
                    return
                self._pub(self.dock_cmd_pub, 'dock_dynamic')
                self._dock_sent = True
                self.get_logger().info('Nav settled — dock_dynamic sent')
                return

            if self._dock_timed_out():
                self._abort_dock_retry('DOCKING_DYNAMIC')
                return

            if self.dock_status == 'docked':
                self.get_logger().info('Docked to dynamic — waiting for receptacle')
                self.dynamic_balls_fired      = 0
                self.launch_status            = ''
                self._waiting_dynamic_start_t = time.monotonic()
                self.state = self.WAITING_DYNAMIC

            elif self.dock_status == 'lost' and self._lost_too_long():
                self.get_logger().warn(
                    f'Dynamic tag lost >{DOCK_LOST_DEBOUNCE_S}s — resuming nav')
                self._pub(self.dock_cmd_pub, 'cancel')
                self._resume_nav()
                self._dock_lost_since = None
                self.state = self.EXPLORING

        # ── WAITING_DYNAMIC ───────────────────────────────────────────────────
        elif self.state == self.WAITING_DYNAMIC:
            self._pause_nav()

            if (self._waiting_dynamic_start_t is not None and
                    time.monotonic() - self._waiting_dynamic_start_t > WAITING_DYNAMIC_TIMEOUT_S):
                self.get_logger().warn(
                    f'WAITING_DYNAMIC timeout ({WAITING_DYNAMIC_TIMEOUT_S}s) — '
                    f'no receptacle seen, cancelling and resuming nav')
                self._pub(self.dock_cmd_pub, 'cancel')
                self._waiting_dynamic_start_t = None
                self._resume_nav()
                self.state = self.EXPLORING
                return

            rec_tag = self._find_tag('dynamic_receptacle')
            if rec_tag is not None:
                self.get_logger().info(
                    f'Receptacle detected — firing ball '
                    f'{self.dynamic_balls_fired + 1}/{DYNAMIC_BALL_COUNT}')
                self._pub(self.launch_cmd_pub, 'fire_one')
                self.launch_status = ''
                self.dynamic_balls_fired += 1
                self.state = self.FIRING_DYNAMIC

        # ── FIRING_DYNAMIC ────────────────────────────────────────────────────
        elif self.state == self.FIRING_DYNAMIC:
            self._pause_nav()

            if self._dock_timed_out():
                self._abort_dock_retry('FIRING_DYNAMIC')
                return

            if ('dynamic_fired' in self.launch_status or
                    'dynamic_done' in self.launch_status):
                if self.dynamic_balls_fired >= DYNAMIC_BALL_COUNT:
                    self.get_logger().info('Dynamic firing complete — backing up')
                    self._backup_done_latch = False
                    self._pub(self.dock_cmd_pub, 'backup')
                    self.state = self.BACKING_UP_DYNAMIC
                else:
                    self.launch_status = ''
                    self.state = self.WAITING_DYNAMIC

        # ── BACKING_UP_DYNAMIC ────────────────────────────────────────────────
        elif self.state == self.BACKING_UP_DYNAMIC:
            self._pause_nav()

            if self._backup_done_latch:
                self.get_logger().info(
                    'Dynamic backup done — ignoring dynamic tags, resuming navigation')
                self.dynamic_done = True
                self._set_ignore('dynamic_dock', 'dynamic_receptacle')
                self._resume_nav()
                self.state = self.EXPLORING   # next tick: check if both done → DONE

        # ── DONE ──────────────────────────────────────────────────────────────
        elif self.state == self.DONE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
