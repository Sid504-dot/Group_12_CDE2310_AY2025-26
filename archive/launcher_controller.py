#!/usr/bin/env python3
"""
launcher_controller.py — Runs on RPi
======================================
Controls the ball launcher mechanism via GPIO.
Subscribes to /mission/launch_command for fire commands.
Publishes /mission/launch_status for feedback.

Commands (on /mission/launch_command):
  "fire_static"           — fire 3 balls with the required timing sequence
  "fire_dynamic"          — fire 1 ball immediately (called per detection)
  "fire_one"              — fire a single ball
  "stop"                  — abort / reset

Timing sequence for Station A:
  The mission says each team has a unique timing delay sequence (released week 7).
  Configure STATIC_FIRE_DELAYS below. All 3 must fire within 4 seconds total.
  Example: [0.0, 1.2, 2.4] means fire at t=0, t=1.2s, t=2.4s after command.

Hardware setup (CHANGE to match your mechanism):
  - LAUNCHER_PIN: GPIO pin connected to launcher relay/servo/motor
  - FIRE_PULSE_DURATION: how long to energise the pin per ball
  - If using a servo, adjust _fire_one_ball() accordingly
"""

import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ── GPIO config ──
LAUNCHER_PIN        = 18          # BCM pin number — CHANGE to your wiring
FIRE_PULSE_DURATION = 0.25        # seconds to hold pin HIGH per ball fire

# ── Timing for Station A (3 balls within 4 sec) ──
# CHANGE to your team's assigned sequence from week 7
STATIC_FIRE_DELAYS  = [0.0, 1.2, 2.4]  # seconds after command to fire each ball

# Total balls
BALLS_STATIC  = 3
BALLS_DYNAMIC = 3   # will fire one-at-a-time on receptacle detection


class LauncherController(Node):
    def __init__(self):
        super().__init__('launcher_controller')

        self.create_subscription(String, '/mission/launch_command',
                                 self._cmd_cb, 10)
        self.status_pub = self.create_publisher(String, '/mission/launch_status', 10)

        self.firing        = False
        self.balls_fired   = 0
        self._fire_thread  = None

        # ── GPIO init ──
        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(LAUNCHER_PIN, GPIO.OUT, initial=GPIO.LOW)
            self.gpio_ok = True
            self.get_logger().info(f'GPIO pin {LAUNCHER_PIN} ready')
        except Exception as e:
            self.GPIO = None
            self.gpio_ok = False
            self.get_logger().warn(
                f'GPIO init failed: {e} — running in simulation mode')

        self.get_logger().info('LauncherController ready')

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _cmd_cb(self, msg: String):
        cmd = msg.data.strip().lower()

        if cmd == 'fire_static':
            if self.firing:
                self.get_logger().warn('Already firing — ignoring')
                return
            self.get_logger().info('FIRE STATIC — 3 balls timed sequence')
            self.balls_fired = 0
            self._fire_thread = threading.Thread(
                target=self._fire_static_sequence, daemon=True)
            self._fire_thread.start()

        elif cmd == 'fire_dynamic' or cmd == 'fire_one':
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

    def _fire_static_sequence(self):
        """Fire 3 balls at the configured delay intervals."""
        self.firing = True
        self._publish_status('firing_static')
        t_start = time.monotonic()

        for i, delay in enumerate(STATIC_FIRE_DELAYS):
            if not self.firing:
                break
            # Wait until the right moment
            target_t = t_start + delay
            now = time.monotonic()
            if target_t > now:
                time.sleep(target_t - now)

            self._fire_one_ball()
            self.balls_fired += 1
            self.get_logger().info(
                f'Ball {self.balls_fired}/{BALLS_STATIC} fired at t+{delay:.2f}s')
            self._publish_status(f'fired_{self.balls_fired}')

        elapsed = time.monotonic() - t_start
        self.get_logger().info(
            f'Static sequence complete — {self.balls_fired} balls in {elapsed:.2f}s')
        self.firing = False
        self._publish_status('static_done')

    def _fire_single(self):
        """Fire exactly one ball."""
        self.firing = True
        self._fire_one_ball()
        self.balls_fired += 1
        self.get_logger().info(f'Single ball fired (total={self.balls_fired})')
        self.firing = False
        self._publish_status(f'dynamic_fired_{self.balls_fired}')

        if self.balls_fired >= BALLS_DYNAMIC:
            self._publish_status('dynamic_done')

    def _fire_one_ball(self):
        """Actuate the launcher mechanism for one ball."""
        self._set_pin(True)
        time.sleep(FIRE_PULSE_DURATION)
        self._set_pin(False)
        time.sleep(0.05)  # tiny gap to let mechanism reset

    def _set_pin(self, high: bool):
        if self.gpio_ok and self.GPIO is not None:
            self.GPIO.output(LAUNCHER_PIN,
                             self.GPIO.HIGH if high else self.GPIO.LOW)
        else:
            state = "HIGH" if high else "LOW"
            self.get_logger().info(f'[SIM] GPIO {LAUNCHER_PIN} → {state}')

    def destroy_node(self):
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
