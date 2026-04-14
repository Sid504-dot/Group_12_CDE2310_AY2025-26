#!/usr/bin/env python3
"""
launcher_controller_revised1.py — Runs on RPi
===============================================
Changes vs launcher_controller.py:
  REV-1A  Pre-prime sequence added before static 3-shot fire.
          Solenoid is held HIGH for PRIME_DURATION_S (8 s) so the
          mechanism fully pressurises / loads, then briefly LOW for
          PRIME_RETRACT_S (0.5 s) before the timed shot sequence.
  REV-1B  LAUNCHER_PIN changed to 21 (BCM) — matches hardware wiring.
  REV-1C  STATIC_FIRE_DELAYS tightened to [0.0, 1.2, 2.4] so all 3
          shots land within 2.4 s after priming (well under the 4 s
          mission window).

Commands on /mission/launch_command:
  "fire_static"  — prime 8 s -> retract 0.5 s -> 3 timed shots
  "fire_dynamic" — fire 1 ball immediately (no prime)
  "fire_one"     — alias for fire_dynamic
  "stop"         — abort immediately, pin LOW
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── GPIO config ──────────────────────────────────────────────────────────────
LAUNCHER_PIN        = 21       # BCM pin — REV-1B

# ── Timing ───────────────────────────────────────────────────────────────────
PRIME_DURATION_S    = 8.0      # seconds to hold pin HIGH for pre-prime  (REV-1A)
PRIME_RETRACT_S     = 0.5      # seconds LOW after prime before first shot
FIRE_PULSE_DURATION = 0.25     # seconds to pulse pin HIGH per individual shot

# ── Static shot schedule (relative to first shot, NOT to prime start) ────────
STATIC_FIRE_DELAYS  = [0.0, 1.2, 2.4]   # all 3 within 2.4 s <= 4 s limit
BALLS_STATIC        = 3
BALLS_DYNAMIC       = 3


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
            self.get_logger().warn(
                f'GPIO init failed: {e} — running in simulation mode')

        self.get_logger().info('LauncherController (rev1) ready')

    # =========================================================================
    # ROS helpers
    # =========================================================================

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # =========================================================================
    # Command callback
    # =========================================================================

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'fire_static':
            if self.firing:
                self.get_logger().warn('Already firing — ignoring fire_static')
                return
            self.get_logger().info(
                f'FIRE STATIC — prime {PRIME_DURATION_S}s -> retract {PRIME_RETRACT_S}s -> 3 shots')
            self.balls_fired  = 0
            self._fire_thread = threading.Thread(
                target=self._fire_static_sequence, daemon=True)
            self._fire_thread.start()

        elif cmd in ('fire_dynamic', 'fire_one'):
            if self.firing:
                return
            self.get_logger().info('FIRE ONE ball (dynamic / immediate)')
            self._fire_thread = threading.Thread(
                target=self._fire_single, daemon=True)
            self._fire_thread.start()

        elif cmd == 'stop':
            self.firing = False
            self._set_pin(False)
            self._publish_status('stopped')
            self.get_logger().info('Stop command received — pin LOW')

    # =========================================================================
    # Fire sequences  (run in daemon threads)
    # =========================================================================

    def _fire_static_sequence(self):
        """
        REV-1A: prime -> retract -> 3 timed shots.

        Timeline:
          t=0                   PIN HIGH  (solenoid primes)
          t=PRIME_DURATION_S    PIN LOW   (brief retract)
          t=+PRIME_RETRACT_S    shot 1
          t=+1.2s               shot 2
          t=+2.4s               shot 3
        """
        self.firing = True
        self._publish_status('priming')

        # Phase 1: prime
        self.get_logger().info(f'Priming solenoid for {PRIME_DURATION_S}s ...')
        self._set_pin(True)
        time.sleep(PRIME_DURATION_S)

        if not self.firing:          # check for stop command during prime
            self._set_pin(False)
            self._publish_status('stopped')
            return

        # Phase 2: retract
        self.get_logger().info(f'Retracting for {PRIME_RETRACT_S}s ...')
        self._set_pin(False)
        time.sleep(PRIME_RETRACT_S)

        if not self.firing:
            self._publish_status('stopped')
            return

        # Phase 3: timed shots
        self._publish_status('firing_static')
        t_start = time.monotonic()

        for delay in STATIC_FIRE_DELAYS:
            if not self.firing:
                break

            target_t = t_start + delay
            wait     = target_t - time.monotonic()
            if wait > 0:
                time.sleep(wait)

            self._fire_one_ball()
            self.balls_fired += 1
            self.get_logger().info(
                f'Ball {self.balls_fired}/{BALLS_STATIC} fired '
                f'(t+{time.monotonic()-t_start:.2f}s from shot-start)')
            self._publish_status(f'fired_{self.balls_fired}')

        elapsed = time.monotonic() - t_start
        self.get_logger().info(
            f'Static sequence complete — {self.balls_fired} balls in {elapsed:.2f}s')
        self.firing = False
        self._publish_status('static_done')

    def _fire_single(self):
        """Fire exactly one ball — no prime (used for dynamic target)."""
        self.firing = True
        self._fire_one_ball()
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
        """Pulse the solenoid for one ball."""
        self._set_pin(True)
        time.sleep(FIRE_PULSE_DURATION)
        self._set_pin(False)
        time.sleep(0.05)   # brief gap for mechanism reset

    def _set_pin(self, high: bool):
        if self.gpio_ok and self.GPIO is not None:
            self.GPIO.output(LAUNCHER_PIN,
                             self.GPIO.HIGH if high else self.GPIO.LOW)
        else:
            state = 'HIGH' if high else 'LOW'
            self.get_logger().info(f'[SIM] GPIO {LAUNCHER_PIN} -> {state}')

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
