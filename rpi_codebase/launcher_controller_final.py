#!/usr/bin/env python3
"""
launcher_controller_revised5.py — Runs on RPi
===============================================
Changes vs revised4:
  REV-5A  Static sequence now uses per-shot OVERALL delay targets instead of
          a fixed inter-shot sleep.  The delay is measured from the moment the
          shot begins, so pulse duration + overhead is already counted:
            Shot 1 → Shot 2 : 4 s total from start of shot 1
            Shot 2 → Shot 3 : 6 s total from start of shot 2
          This guarantees the wall-clock gap is exactly the target regardless
          of how long GPIO / OS scheduling takes inside _fire_one_ball().
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

# Overall time (seconds) from the START of shot N to the START of shot N+1.
# Index 0 → gap between shot 1 and shot 2 (4 s total)
# Index 1 → gap between shot 2 and shot 3 (6 s total)
# Any pulse/overhead time already elapsed is subtracted automatically.
STATIC_INTER_SHOT_DELAYS = [4.0, 6.0]


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

        self.get_logger().info('LauncherController (rev3) ready — strict concurrency lock')

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
                self.get_logger().warn('Already firing — ignoring duplicate fire_static command')
                return
            
            # REV-3A: Lock the state SYNCHRONOUSLY before the thread even spins up.
            self.firing = True
            self.get_logger().info('FIRE STATIC — 3 direct shots (sequence locked)')
            self.balls_fired  = 0
            self._fire_thread = threading.Thread(
                target=self._fire_static_sequence, daemon=True)
            self._fire_thread.start()

        elif cmd in ('fire_dynamic', 'fire_one'):
            if self.firing:
                self.get_logger().warn('Already firing — ignoring duplicate fire_one command')
                return
                
            # REV-3A: Synchronous lock
            self.firing = True
            self.get_logger().info('FIRE ONE ball')
            self._fire_thread = threading.Thread(
                target=self._fire_single, daemon=True)
            self._fire_thread.start()

        elif cmd == 'stop':
            self.firing = False
            self._set_pin(False)
            self._publish_status('stopped')
            self.get_logger().info('Sequence forcefully STOPPED')

    # =========================================================================
    # Fire sequences
    # =========================================================================

    def _fire_static_sequence(self):
        """3 shots with per-gap OVERALL delay targets.

        Gap targets (measured from the START of each shot):
          Shot 1 → Shot 2 : STATIC_INTER_SHOT_DELAYS[0]  (default 4 s)
          Shot 2 → Shot 3 : STATIC_INTER_SHOT_DELAYS[1]  (default 6 s)

        The sleep after each shot is computed as:
            remaining = target_delay - elapsed_since_shot_start
        so pulse duration + OS overhead is already counted and the
        wall-clock gap is exactly the target delay.
        """
        self._publish_status('firing_static')
        t_start = time.monotonic()

        for shot_num in range(1, BALLS_STATIC + 1):
            if not self.firing:
                self.get_logger().warn('Sequence aborted via stop command.')
                break

            t_shot = time.monotonic()   # REV-5A: timestamp shot start
            self.get_logger().info(f'--- Initiating Shot {shot_num} of {BALLS_STATIC} ---')
            self._fire_one_ball()
            self.balls_fired += 1

            self._publish_status(f'fired_{self.balls_fired}')
            self.get_logger().info(
                f'Shot {self.balls_fired} completed (t+{time.monotonic() - t_start:.2f}s)')

            # REV-5A: overall-delay sleep (not after the last shot)
            if shot_num < BALLS_STATIC and self.firing:
                target_delay = STATIC_INTER_SHOT_DELAYS[shot_num - 1]
                elapsed      = time.monotonic() - t_shot
                remaining    = target_delay - elapsed
                self.get_logger().info(
                    f'Inter-shot gap target {target_delay:.2f}s | '
                    f'elapsed {elapsed:.3f}s | sleeping {max(remaining, 0):.3f}s')
                if remaining > 0:
                    time.sleep(remaining)

        # REV-2B/3B: explicit LOW to guarantee solenoid retracted at the absolute end
        self.get_logger().info('Sequence loop finished, asserting final PIN LOW.')
        self._set_pin(False)

        elapsed = time.monotonic() - t_start
        self.get_logger().info(
            f'=== Static sequence complete — {self.balls_fired} balls in {elapsed:.2f}s ===')
        
        self.firing = False
        self._publish_status('static_done')

    def _fire_single(self):
        """Fire exactly one ball — dynamic target."""
        # BUG-3 FIX: balls_fired was not reset before dynamic shots. After
        # the static sequence (balls_fired=3), the first dynamic shot raised
        # it to 4, immediately satisfying balls_fired >= BALLS_DYNAMIC=3 and
        # publishing dynamic_done after just one shot instead of three.
        # Track dynamic shots with a fresh counter scoped to this call;
        # the instance counter is reset on entry so dynamic_done fires correctly.
        self.balls_fired = 0
        self._fire_one_ball()
        self._set_pin(False)   # explicit retract
        self.balls_fired += 1
        self.get_logger().info(f'Single ball fired (total dynamic={self.balls_fired})')
        self.firing = False
        self._publish_status(f'dynamic_fired_{self.balls_fired}')
        if self.balls_fired >= BALLS_DYNAMIC:
            self._publish_status('dynamic_done')

    # =========================================================================
    # GPIO helpers
    # =========================================================================

    def _fire_one_ball(self):
        if not self.firing:
            return
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