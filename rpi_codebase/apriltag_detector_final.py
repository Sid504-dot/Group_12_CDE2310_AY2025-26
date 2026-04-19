#!/usr/bin/env python3
"""
apriltag_detector_revised4.py — Runs on RPi
=============================================
Changes vs apriltag_detector_revised3.py:

  REV-4A  Added detection-rate logging. Every 5 seconds, logs how many
          frames were processed and how many contained each tag type.
          Helps diagnose "the dynamic tag isn't being detected" vs
          "it's detected but the coordinator ignores it".

  REV-4B  Added decision_margin filter — tags with margin < 25 are
          rejected. Low-margin detections are ambiguous (the detector
          isn't confident it's a real tag) and produce noisy / wildly
          wrong pose estimates. This prevents false-positive detections
          from triggering the dock controller.

  REV-4C  Log line now includes the tag's decision_margin alongside
          dist and yaw for debugging.

Inherited from REV-3:
  REV-3A  TAG_SIZE_M = 0.10 (10 cm physical tag)
  REV-3B  quad_decimate = 1.0 (no resolution halving)
  REV-3C  quad_sigma = 0.8 (blur for noisy sensor)
  REV-3D  Proper luminance weighting for grayscale conversion
  REV-2A  /mission/ignore_types subscription
  REV-2B  Skips publishing if all tags are in the ignore list

Tag ID conventions:
  STATIC_TAG_IDS         = [0]
  DYNAMIC_DOCK_TAG_IDS   = [25]
  DYNAMIC_RECEPTACLE_IDS = [15]
"""

import math
import time
import numpy as np
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Image

# ── Tag ID config ─────────────────────────────────────────────────────────────
STATIC_TAG_IDS          = [0]
DYNAMIC_DOCK_TAG_IDS    = [25]
DYNAMIC_RECEPTACLE_IDS  = [15]

# ── Camera intrinsics (RPi Camera v2 at 640x480) ─────────────────────────────
CAMERA_PARAMS = (462.0, 462.0, 320.0, 240.0)   # fx, fy, cx, cy

TAG_SIZE_M = 0.08   # physical tag side length in metres

# REV-4B: reject detections below this confidence threshold
MIN_DECISION_MARGIN = 25.0

# REV-4A: detection rate logging interval
RATE_LOG_INTERVAL_S = 5.0


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        # ── Publishers ────────────────────────────────────────────────────────
        self.det_pub = self.create_publisher(String, '/apriltag/detections', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos_profile_sensor_data)

        self.create_subscription(
            String, '/mission/ignore_types', self._ignore_cb, 10)

        self._ignore_types: set = set()

        # ── AprilTag detector ─────────────────────────────────────────────────
        try:
            from pupil_apriltags import Detector
            self.detector = Detector(
                families='tag36h11',
                nthreads=2,
                quad_decimate=1.0,
                quad_sigma=0.8,
                decode_sharpening=0.25,
            )
            self.get_logger().info('AprilTag detector ready (tag36h11)')
        except ImportError:
            self.get_logger().error(
                'pupil-apriltags not installed — pip3 install pupil-apriltags')
            self.detector = None

        # REV-4A: detection rate tracking
        self._rate_window_start = time.monotonic()
        self._frames_processed  = 0
        self._type_counts       = {}   # e.g. {'static': 5, 'dynamic_dock': 2}

        self.get_logger().info(
            f'AprilTagDetector (rev4) started  TAG_SIZE={TAG_SIZE_M}m  '
            f'MIN_MARGIN={MIN_DECISION_MARGIN}')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _ignore_cb(self, msg: String):
        types = {t.strip().lower() for t in msg.data.split(',') if t.strip()}
        if types != self._ignore_types:
            self.get_logger().info(f'Ignoring tag types: {types}')
        self._ignore_types = types

    def _image_cb(self, msg: Image):
        if self.detector is None:
            return

        # Convert ROS Image → numpy ───────────────────────────────────────────
        try:
            enc = msg.encoding
            if enc in ('rgb8', 'bgr8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
                if enc == 'bgr8':
                    frame = frame[:, :, ::-1]
            elif enc == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width))
            else:
                self.get_logger().warn(
                    f'Unsupported encoding: {enc}', throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed: {e}', throttle_duration_sec=5.0)
            return

        # Proper luminance weighting ──────────────────────────────────────────
        if len(frame.shape) == 3:
            gray = (
                0.299 * frame[:, :, 0] +
                0.587 * frame[:, :, 1] +
                0.114 * frame[:, :, 2]
            ).astype(np.uint8)
        else:
            gray = frame

        # ── Detect tags ───────────────────────────────────────────────────────
        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )

        self._frames_processed += 1

        tags_out = []
        for r in results:
            # REV-4B: reject low-confidence detections
            if r.decision_margin < MIN_DECISION_MARGIN:
                self.get_logger().debug(
                    f'Rejected tag {r.tag_id} — margin={r.decision_margin:.1f} '
                    f'< {MIN_DECISION_MARGIN}',
                    throttle_duration_sec=2.0)
                continue

            tag_type = self._classify(r.tag_id)

            # REV-2B: skip ignored types
            if tag_type in self._ignore_types:
                continue

            t    = r.pose_t.flatten()
            dist = float(np.linalg.norm(t))
            yaw  = float(math.atan2(t[0], t[2]))

            tags_out.append({
                'id':         int(r.tag_id),
                'type':       tag_type,
                'cx':         r.center.tolist()[0],
                'cy':         r.center.tolist()[1],
                'dist':       round(dist, 4),
                'yaw_offset': round(yaw, 4),
                'tx':         round(float(t[0]), 4),
                'ty':         round(float(t[1]), 4),
                'tz':         round(float(t[2]), 4),
                'corners':    r.corners.tolist(),
            })

            # REV-4A: count detections per type
            self._type_counts[tag_type] = self._type_counts.get(tag_type, 0) + 1

        if tags_out:
            out_msg      = String()
            out_msg.data = json.dumps({'tags': tags_out, 'stamp': time.time()})
            self.det_pub.publish(out_msg)
            # REV-4C: include margin in log
            det_str = ', '.join(
                f"{t['type']}(id={t['id']},d={t['dist']:.2f}m)"
                for t in tags_out)
            self.get_logger().info(
                f'Detected: {det_str}', throttle_duration_sec=1.0)

        # REV-4A: periodic detection rate log ─────────────────────────────────
        elapsed = time.monotonic() - self._rate_window_start
        if elapsed >= RATE_LOG_INTERVAL_S:
            fps = self._frames_processed / elapsed
            counts_str = ', '.join(
                f'{k}={v}' for k, v in sorted(self._type_counts.items()))
            if not counts_str:
                counts_str = 'none'
            self.get_logger().info(
                f'[RATE] {fps:.1f} FPS over {elapsed:.0f}s  '
                f'detections: {counts_str}')
            self._rate_window_start = time.monotonic()
            self._frames_processed  = 0
            self._type_counts       = {}

    # =========================================================================
    # Helpers
    # =========================================================================

    def _classify(self, tag_id: int) -> str:
        if tag_id in STATIC_TAG_IDS:
            return 'static'
        if tag_id in DYNAMIC_DOCK_TAG_IDS:
            return 'dynamic_dock'
        if tag_id in DYNAMIC_RECEPTACLE_IDS:
            return 'dynamic_receptacle'
        return 'unknown'


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
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
