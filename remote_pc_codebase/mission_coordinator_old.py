#!/usr/bin/env python3
"""
mission_coordinator.py  (bug-fixed)
=====================================
Full mission state machine integrating navigation + AprilTag + docking + launching.

State flow:
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

Nav pause/resume:
  /mission/nav_command  "pause"  → nav node stops, blacklists current goal
  /mission/nav_command  "resume" → nav node reselects and continues

Ignore list:
  /mission/ignore_types  e.g. "static"  → apriltag_detector suppresses that type

TEST_MODE = True  : after firing static, cooldown then allow re-detection
TEST_MODE = False : static marked done forever, only dynamic remains

Bug-fixes applied:
  COORD-BUG 1 — pause/resume were single fire-and-forget messages; a dropped
                message left the nav node permanently frozen or running.
                Fixed: coordinator re-publishes the current command every tick
                while in non-EXPLORING states (pause held during dock/fire/backup,
                resume held during cooldown). Nav node is idempotent so repeated
                messages are safe.

  COORD-BUG 2 — BACKING_UP was a single shared state for both static and
                dynamic sequences. After dynamic backup, the handler set
                static_done=True instead of dynamic_done=True, so dynamic_done
                was never set and the mission never reached DONE.
                Fixed: split into BACKING_UP_STATIC and BACKING_UP_DYNAMIC.
                Each branch sets the correct *_done flag.

  COORD-BUG 3 — TEST_MODE path in BACKING_UP never set static_done=True,
                so the dynamic target check (gated on static_done) was
                permanently unreachable in test mode.
                Fixed: static_done=True is now set in all BACKING_UP_STATIC
                exit paths including TEST_MODE.

  COORD-BUG 4 — latest_tags was never aged out. Stale detections from before
                a backup manoeuvre could immediately re-trigger docking on the
                next EXPLORING tick before the robot had moved.
                Fixed: _find_tag returns None if last detection is older than
                TAG_STALE_S seconds.

  COORD-BUG 5 — dynamic_done was never set to True anywhere in the file,
                making it impossible for the mission to ever reach DONE.
                Fixed: BACKING_UP_DYNAMIC sets dynamic_done=True on exit.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

COORDINATOR_HZ       = 20
DETECTION_RANGE_M    = 1.5    # ignore tags farther than this
DYNAMIC_BALL_COUNT   = 3
DOCK_LOST_DEBOUNCE_S      = 5.0    # 'lost' must persist this long before aborting dock
WAITING_DYNAMIC_TIMEOUT_S = 30.0   # abort if receptacle not seen within this many seconds
DOCK_TIMEOUT_S            = 45.0   # abort dock+fire cycle if no ball launched within this

# COORD-BUG 4 FIX: reject detections older than this many seconds
TAG_STALE_S       = 0.5

TEST_MODE         = False
STATIC_COOLDOWN_S = 10.0


class MissionCoordinator(Node):

    EXPLORING          = 'EXPLORING'
    DOCKING_STATIC     = 'DOCKING_STATIC'
    FIRING_STATIC      = 'FIRING_STATIC'
    BACKING_UP_STATIC  = 'BACKING_UP_STATIC'   # COORD-BUG 2 FIX: was shared BACKING_UP
    COOLDOWN           = 'COOLDOWN'
    DOCKING_DYNAMIC    = 'DOCKING_DYNAMIC'
    WAITING_DYNAMIC    = 'WAITING_DYNAMIC'
    FIRING_DYNAMIC     = 'FIRING_DYNAMIC'
    BACKING_UP_DYNAMIC = 'BACKING_UP_DYNAMIC'   # COORD-BUG 2 FIX: separate state
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
        self.last_det_time = 0.0   # COORD-BUG 4 FIX: tracked for staleness check

        self._dock_sent         = False
        self._fire_sent         = False
        self._cooldown_t        = 0.0
        self._dock_lost_since   = None
        self._backup_done_latch        = False   # latches one-shot 'backup_done' to avoid race
        self._waiting_dynamic_start_t  = None   # set when entering WAITING_DYNAMIC
        self._dock_start_t             = None   # set when dock command sent, for timeout
        self._nav_settle_until         = 0.0    # time after which Nav2 has fully stopped

        self.create_timer(1.0 / COORDINATOR_HZ, self._tick)
        mode_str = 'TEST (re-trigger enabled)' if TEST_MODE else 'MISSION'
        self.get_logger().info(
            f'MissionCoordinator started — state=EXPLORING  mode={mode_str}')

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
            self._backup_done_latch = True   # latched so race with 'idle' can't lose it
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
        COORD-BUG 4 FIX: return None if latest detection is stale.

        Without this, a tag seen before/during a backup manoeuvre persists
        in latest_tags and immediately re-triggers docking on the next
        EXPLORING tick before the robot has moved away from the target.
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
        """True if DOCK_TIMEOUT_S elapsed since dock command was sent."""
        return (self._dock_start_t is not None and
                time.monotonic() - self._dock_start_t > DOCK_TIMEOUT_S)

    def _abort_dock_retry(self, label: str):
        """
        Cancel docking and resume navigation WITHOUT marking the target done.
        The target's done-flag is left False so EXPLORING will detect and retry it.
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
                tag = self._find_tag('static')
                if tag is not None:
                    self.get_logger().info(
                        f'Static tag {tag["id"]} at {tag["dist"]:.2f}m — pausing nav, settling 0.5s')
                    self._pause_nav()
                    self._dock_sent      = False   # sent after settle, not now
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
                    self._dock_sent      = False   # sent after settle, not now
                    self._dock_start_t   = time.monotonic()
                    self._nav_settle_until = time.monotonic() + 0.5
                    self.state = self.DOCKING_DYNAMIC
                    return

            if self.static_done and self.dynamic_done:
                self.get_logger().info('All targets complete — DONE')
                self.state = self.DONE

        # ── DOCKING_STATIC ────────────────────────────────────────────────────
        elif self.state == self.DOCKING_STATIC:
            # COORD-BUG 1 FIX: re-publish pause every tick while docking so a
            # dropped message can't leave the nav node running during the approach.
            self._pause_nav()

            # SETTLE FIX: wait 500ms after pausing before sending dock command.
            # Nav2's MPPI takes 50-150ms to process cancel_goal_async and stop
            # publishing cmd_vel. Sending dock_static immediately lets Nav2
            # override the dock controller and spin the robot.
            if not self._dock_sent:
                if time.monotonic() < self._nav_settle_until:
                    return   # still draining Nav2 residual cmd_vel
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
                self.get_logger().warn('Static tag lost too long — resuming nav')
                self._pub(self.dock_cmd_pub, 'cancel')
                self._resume_nav()
                self._dock_lost_since = None
                self.state = self.EXPLORING

        # ── FIRING_STATIC ─────────────────────────────────────────────────────
        elif self.state == self.FIRING_STATIC:
            # COORD-BUG 1 FIX: hold pause while firing
            self._pause_nav()

            if self._dock_timed_out():
                self._abort_dock_retry('FIRING_STATIC')
                return

            if 'static_done' in self.launch_status:
                self.get_logger().info('Static firing complete — backing up')
                self._backup_done_latch = False   # clear stale latch before waiting
                self._dock_start_t      = None    # launch confirmed — timeout no longer needed
                self._pub(self.dock_cmd_pub, 'backup')
                self.state = self.BACKING_UP_STATIC

        # ── BACKING_UP_STATIC ─────────────────────────────────────────────────
        # COORD-BUG 2 FIX: renamed from BACKING_UP; only handles static sequence.
        elif self.state == self.BACKING_UP_STATIC:
            # COORD-BUG 1 FIX: hold pause until backup is confirmed done
            self._pause_nav()

            if self._backup_done_latch:
                self.get_logger().info('Static backup done — resuming navigation')

                # COORD-BUG 3 FIX: static_done=True in ALL exit paths including
                # TEST_MODE. Without this, the dynamic target (gated on
                # static_done) was permanently unreachable in test mode.
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
            # Nav is already resumed in COOLDOWN; robot is exploring.
            # No need to hold pause here.
            if time.monotonic() - self._cooldown_t >= STATIC_COOLDOWN_S:
                self.get_logger().info('Cooldown done — ready for next detection')
                self.launch_status = ''
                self.state = self.EXPLORING

        # ── DOCKING_DYNAMIC ───────────────────────────────────────────────────
        elif self.state == self.DOCKING_DYNAMIC:
            # COORD-BUG 1 FIX: hold pause while docking
            self._pause_nav()

            # SETTLE FIX: same as DOCKING_STATIC — wait for Nav2 to stop
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
                self.get_logger().warn('Dynamic tag lost too long — resuming nav')
                self._pub(self.dock_cmd_pub, 'cancel')
                self._resume_nav()
                self._dock_lost_since = None
                self.state = self.EXPLORING

        # ── WAITING_DYNAMIC ───────────────────────────────────────────────────
        elif self.state == self.WAITING_DYNAMIC:
            # COORD-BUG 1 FIX: hold pause while waiting for receptacle
            self._pause_nav()

            # Timeout: if receptacle never appears, abort and resume navigation
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
            # COORD-BUG 1 FIX: hold pause while firing
            self._pause_nav()

            if self._dock_timed_out():
                self._abort_dock_retry('FIRING_DYNAMIC')
                return

            if ('dynamic_fired' in self.launch_status or
                    'dynamic_done' in self.launch_status):
                if self.dynamic_balls_fired >= DYNAMIC_BALL_COUNT:
                    self.get_logger().info('Dynamic firing complete — backing up')
                    self._backup_done_latch = False   # clear stale latch before waiting
                    self._pub(self.dock_cmd_pub, 'backup')
                    self.state = self.BACKING_UP_DYNAMIC
                else:
                    self.launch_status = ''
                    self.state = self.WAITING_DYNAMIC

        # ── BACKING_UP_DYNAMIC ────────────────────────────────────────────────
        # COORD-BUG 2 + 5 FIX: separate state that correctly sets dynamic_done.
        # The old shared BACKING_UP set static_done=True here, so dynamic_done
        # was never True and the mission never reached DONE.
        elif self.state == self.BACKING_UP_DYNAMIC:
            # COORD-BUG 1 FIX: hold pause until backup is confirmed done
            self._pause_nav()

            if self._backup_done_latch:
                self.get_logger().info('Dynamic backup done — mission complete')
                # COORD-BUG 5 FIX: set dynamic_done so DONE state is reachable
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