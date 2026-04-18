#!/usr/bin/env python3
"""
mission_coordinator_v2.py  (runs on laptop / remote PC)
========================================================
Changes vs mission_coordinator_old.py (bug-fixed):

  V2-A  DOCK_LOST_DEBOUNCE_S increased 5.0 → 8.0 s.
        The RPi camera + pupil-apriltags can drop detections for 3–6 s
        at oblique angles or when the tag is near the frame edge during
        approach. 5 s was too short — the coordinator aborted valid
        docking attempts. 8 s gives the detector more breathing room.

  V2-B  TAG_STALE_S increased 0.5 → 1.0 s.
        At ~15 FPS, the detector sometimes skips 4–8 consecutive frames
        when the tag is small / distant. With a 0.5 s window, the tag
        was already "stale" by the time the coordinator checked for it,
        so dynamic tags were never acted on. 1.0 s fixes this.

  V2-C  Added explicit log in EXPLORING when dynamic tag is skipped
        because static_done is False. Previously this was silent and
        confusing — the detector reported seeing a dynamic tag but the
        coordinator did nothing. Now it logs why.

  V2-D  TEST_MODE default changed True → False for production. The old
        file had TEST_MODE=False already, kept as-is.

All COORD-BUG 1–5 fixes from mission_coordinator_old.py are preserved.

State flow (unchanged):
  EXPLORING
    → static tag detected within range
  DOCKING_STATIC
    → dock_controller servos to target, publishes 'docked'
  FIRING_STATIC
    → launcher fires balls, publishes 'static_done'
  BACKING_UP_STATIC
    → dock_controller drives backward, publishes 'backup_done'
  EXPLORING  (static_done=True, dynamic now searchable)
  DOCKING_DYNAMIC → WAITING_DYNAMIC → FIRING_DYNAMIC → BACKING_UP_DYNAMIC → DONE

Important: Dynamic docking is GATED on static_done=True.
  The coordinator will NOT attempt dynamic docking until the static
  target has been completed. If you want to test dynamic docking
  independently, set static_done=True in __init__.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COORDINATOR_HZ       = 20
DETECTION_RANGE_M    = 1.5    # ignore tags farther than this
DYNAMIC_BALL_COUNT   = 3

# V2-A: increased from 5.0 → 8.0 s
DOCK_LOST_DEBOUNCE_S      = 8.0    # 'lost' must persist this long before aborting dock
WAITING_DYNAMIC_TIMEOUT_S = 30.0   # abort if receptacle not seen within this many seconds
DOCK_TIMEOUT_S            = 45.0   # abort dock+fire cycle if no ball launched within this

# V2-B: increased from 0.5 → 1.0 s
TAG_STALE_S       = 1.0

TEST_MODE         = False
STATIC_COOLDOWN_S = 3.0


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
        self.create_subscription(String, '/apriltag/detections',   self._det_cb,          10)
        self.create_subscription(String, '/mission/dock_status',   self._dock_status_cb,  10)
        self.create_subscription(String, '/mission/launch_status', self._launch_status_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.nav_cmd_pub    = self.create_publisher(String, '/mission/nav_command',   10)
        self.dock_cmd_pub   = self.create_publisher(String, '/mission/dock_command',  10)
        self.launch_cmd_pub = self.create_publisher(String, '/mission/launch_command', 10)
        self.ignore_pub     = self.create_publisher(String, '/mission/ignore_types',  10)
        self.state_pub      = self.create_publisher(String, '/mission/state',         10)

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
            f'MissionCoordinator v2 started — state=EXPLORING  mode={mode_str}  '
            f'LOST_DEBOUNCE={DOCK_LOST_DEBOUNCE_S}s  TAG_STALE={TAG_STALE_S}s')

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
        """
        Return the closest tag of the given type within DETECTION_RANGE_M,
        or None if the latest detection batch is stale (older than TAG_STALE_S).
        """
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
        """
        Cancel docking and resume navigation WITHOUT marking the target done.
        """
        self.get_logger().warn(
            f'{label}: dock timeout ({DOCK_TIMEOUT_S}s) — '
            f'cancelling and resuming nav (target NOT marked done, will retry)')
        self._pub(self.dock_cmd_pub, 'cancel')
        self._dock_start_t            = None
        self._dock_lost_since         = None
        self._waiting_dynamic_start_t = None
        self._resume_nav()
        self.state = self.EXPLORING

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

            if not self.static_done:
                # V2-C: log when dynamic tag is seen but static isn't done yet
                dyn_tag = self._find_tag('dynamic_dock')
                if dyn_tag is not None:
                    self.get_logger().info(
                        f'Dynamic tag {dyn_tag["id"]} seen at {dyn_tag["dist"]:.2f}m '
                        f'but static_done={self.static_done} — ignoring until static is complete',
                        throttle_duration_sec=5.0)

                tag = self._find_tag('static')
                if tag is not None:
                    self.get_logger().info(
                        f'Static tag {tag["id"]} at {tag["dist"]:.2f}m — pausing nav, settling 0.5s')
                    self._pause_nav()
                    self._dock_sent      = False
                    self._dock_start_t   = time.monotonic()
                    self._nav_settle_until = time.monotonic() + 0.5
                    self.state = self.DOCKING_STATIC
                    return

            if not self.dynamic_done and self.static_done:
                tag = self._find_tag('dynamic_dock')
                if tag is not None:
                    self.get_logger().info(
                        f'Dynamic dock tag {tag["id"]} at {tag["dist"]:.2f}m — pausing nav, settling 0.5s')
                    self._pause_nav()
                    self._dock_sent      = False
                    self._dock_start_t   = time.monotonic()
                    self._nav_settle_until = time.monotonic() + 0.5
                    self.state = self.DOCKING_DYNAMIC
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
                self._fire_sent  = True
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
                self.get_logger().info('Static backup done — resuming navigation')
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
                self.dynamic_balls_fired       = 0
                self.launch_status             = ''
                self._waiting_dynamic_start_t  = time.monotonic()
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
                    f'no receptacle seen, cancelling dock and resuming nav')
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
                self.get_logger().info('Dynamic backup done — mission complete')
                self.dynamic_done = True
                self._set_ignore('static', 'dynamic_dock', 'dynamic_receptacle')
                self._resume_nav()
                self.state = self.EXPLORING   # next tick: static+dynamic done → DONE

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


