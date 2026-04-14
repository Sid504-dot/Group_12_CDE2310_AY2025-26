#!/usr/bin/env python3
"""
launcher_controller_revised2.py — Runs on RPi
===============================================
Changes vs launcher_controller_revised1.py:
  REV-2A  Prime phase REMOVED entirely. fire_static goes straight to
          3 timed shots with no 8s hold.
  REV-2B  Explicit GPIO LOW after all 3 shots to guarantee solenoid
          is retracted when the sequence ends.

Sequence for fire_static:
  Shot 1: PIN HIGH (FIRE_PULSE_DURATION) -> PIN LOW -> 0.05s gap
  Sleep 1.2s
  Shot 2: PIN HIGH -> PIN LOW -> 0.05s gap
  Sleep 1.2s
  Shot 3: PIN HIGH -> PIN LOW -> 0.05s gap
  PIN LOW (explicit retract guarantee)  <- REV-2B
  Publish 'static_done'

All 3 shots complete within ~2.5s, well inside the 4s mission window.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── GPIO config ──────────────────────────────────────────────────────────────
LAUNCHER_PIN        = 21       # BCM pin

# ── Timing ───────────────────────────────────────────────────────────────────
FIRE_PULSE_DURATION = 0.25     # seconds pin stays HIGH per shot
SHOT_INTERVAL_S     = 1.2      # seconds between shots

BALLS_STATIC  = 3
BALLS_DYNAMIC = 3


class LauncherController(Node):

    def __init__(self):
        super().__init__('launcher_controller')

        self.create_subscription(String, '/mission/launch_command',
                                 self._cmd_cb, 10)
        self.status_pub = self.create_publisher(String, '/mission/launch_status', 10)

        self.firing       = False
        self.balls_fired  = 0
        self._fire_thread = None

        # ── GPIO init ─────────────────────────────────────────────────────────
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(LAUNCHER_PIN, GPIO.OUT, initial=GPIO.LOW)
            self.gpio_ok = True
            self.get_logger().info(f'GPIO pin {LAUNCHER_PIN} ready')
        except Exception as e:
            self.GPIO    = None
            self.gpio_ok = False
            self.get_logger().warn(f'GPIO init failed: {e} — simulation mode')

        self.get_logger().info('LauncherController (rev2) ready — no prime')

    # =========================================================================
    # Helpers
    # =========================================================================

    def _publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)

    # =========================================================================
    # Command callback
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'fire_static':
            if self.firing:
                self.get_logger().warn('Already firing — ignoring')
                return
            self.get_logger().info('FIRE STATIC — 3 direct shots (no prime)')
            self.balls_fired  = 0
            self._fire_thread = threading.Thread(
                target=self._fire_static_sequence, daemon=True)
            self._fire_thread.start()

        elif cmd in ('fire_dynamic', 'fire_one'):
            if self.firing:
                return
            self.get_logger().info('FIRE ONE ball')
            self._fire_thread = threading.Thread(
                target=self._fire_single, daemon=True)
            self._fire_thread.start()

        elif cmd == 'stop':
            self.firing = False
            self._set_pin(False)
            self._publish_status('stopped')

    # =========================================================================
    # Fire sequences
    # =========================================================================

    def _fire_static_sequence(self):
        """3 shots at SHOT_INTERVAL_S intervals. No prime phase (REV-2A)."""
        self.firing = True
        self._publish_status('firing_static')
        t_start = time.monotonic()

        for shot_num in range(1, BALLS_STATIC + 1):
            if not self.firing:
                break

            self._fire_one_ball()
            self.balls_fired += 1
            self.get_logger().info(
                f'Ball {self.balls_fired}/{BALLS_STATIC} fired '
                f'(t+{time.monotonic() - t_start:.2f}s)')
            self._publish_status(f'fired_{self.balls_fired}')

            # Sleep between shots (but not after the last one)
            if shot_num < BALLS_STATIC and self.firing:
                time.sleep(SHOT_INTERVAL_S)

        # REV-2B: explicit LOW to guarantee solenoid retracted
        self._set_pin(False)

        elapsed = time.monotonic() - t_start
        self.get_logger().info(
            f'Static sequence complete — {self.balls_fired} balls in {elapsed:.2f}s')
        self.firing = False
        self._publish_status('static_done')

    def _fire_single(self):
        """Fire exactly one ball — dynamic target."""
        self.firing = True
        self._fire_one_ball()
        self._set_pin(False)   # explicit retract
        self.balls_fired += 1
        self.get_logger().info(f'Single ball fired (total={self.balls_fired})')
        self.firing = False
        self._publish_status(f'dynamic_fired_{self.balls_fired}')
        if self.balls_fired >= BALLS_DYNAMIC:
            self._publish_status('dynamic_done')

    # =========================================================================
    # GPIO helpers
    # =========================================================================

    def _fire_one_ball(self):
        self._set_pin(True)
        time.sleep(FIRE_PULSE_DURATION)
        self._set_pin(False)
        time.sleep(0.05)

    def _set_pin(self, high: bool):
        if self.gpio_ok and self.GPIO is not None:
            self.GPIO.output(LAUNCHER_PIN,
                             self.GPIO.HIGH if high else self.GPIO.LOW)
        else:
            self.get_logger().info(
                f'[SIM] GPIO {LAUNCHER_PIN} -> {"HIGH" if high else "LOW"}')

    # =========================================================================
    # Cleanup
    # =========================================================================

    def destroy_node(self):
        self.firing = False
        self._set_pin(False)
        if self.gpio_ok and self.GPIO is not None:
            try:
                self.GPIO.cleanup(LAUNCHER_PIN)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LauncherController()
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
