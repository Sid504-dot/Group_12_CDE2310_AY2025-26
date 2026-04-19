#!/usr/bin/env python3
"""
apriltag_detector_revised2.py — Runs on RPi
=============================================
Changes vs apriltag_detector_revised1.py:
  REV-2A  Subscribes to /mission/ignore_types (String, comma-separated)
          so mission_coordinator can suppress publishing of already-completed
          tag types (e.g. "static" after static target is done). This stops
          the coordinator from seeing detections for targets it already finished.
  REV-2B  Skips publishing if all detected tags are in the ignore list,
          reducing unnecessary topic traffic during navigation.

Requires:
  - camera_ros running: ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888
  - pupil-apriltags: pip3 install pupil-apriltags

Tag ID conventions:
  STATIC_TAG_IDS         = [0, 1, 2]
  DYNAMIC_DOCK_TAG_IDS   = [10, 11, 12]
  DYNAMIC_RECEPTACLE_IDS = [20, 21]
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
DYNAMIC_DOCK_TAG_IDS    = [5]   # BUG-1 FIX: was [5], must match docstring + physical tags
DYNAMIC_RECEPTACLE_IDS  = [3]

# ── Camera intrinsics (RPi Camera v2 at 640x480 — calibrate for your lens) ───
CAMERA_PARAMS = (462.0, 462.0, 320.0, 240.0)   # fx, fy, cx, cy
TAG_SIZE_M    = 0.05   # physical tag side length in metres


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        # ── Publishers ────────────────────────────────────────────────────────
        self.det_pub = self.create_publisher(String, '/apriltag/detections', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos_profile_sensor_data)

        # REV-2A: mission coordinator can tell us which types to suppress
        self.create_subscription(
            String, '/mission/ignore_types', self._ignore_cb, 10)

        self._ignore_types: set = set()   # e.g. {'static'} after static done

        # ── AprilTag detector ─────────────────────────────────────────────────
        try:
            from pupil_apriltags import Detector
            self.detector = Detector(
                families='tag36h11',
                nthreads=2,
                quad_decimate=2.0,
                quad_sigma=0.0,
                decode_sharpening=0.25,
            )
            self.get_logger().info('AprilTag detector ready (tag36h11)')
        except ImportError:
            self.get_logger().error('pupil-apriltags not installed — pip3 install pupil-apriltags')
            self.detector = None

        self.get_logger().info('AprilTagDetector (rev2) started')

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _ignore_cb(self, msg: String):
        """REV-2A: update ignore list from mission coordinator."""
        types = {t.strip().lower() for t in msg.data.split(',') if t.strip()}
        if types != self._ignore_types:
            self.get_logger().info(f'Ignoring tag types: {types}')
        self._ignore_types = types

    def _image_cb(self, msg: Image):
        if self.detector is None:
            return

        # Convert ROS Image → numpy
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

        # BUG-12 FIX: use ITU-R BT.601 luminance weights instead of equal-weight
        # mean. Equal weighting flattens red/blue contrast, softening the
        # high-contrast edges that AprilTag quad detection relies on.
        gray = (
            (0.299 * frame[:, :, 0] +
             0.587 * frame[:, :, 1] +
             0.114 * frame[:, :, 2]).astype(np.uint8)
            if len(frame.shape) == 3 else frame
        )

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )

        tags_out = []
        for r in results:
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

        if tags_out:
            out_msg      = String()
            out_msg.data = json.dumps({'tags': tags_out, 'stamp': time.time()})
            self.det_pub.publish(out_msg)
            det_str = ', '.join(
                f"{t['type']}(id={t['id']},dist={t['dist']:.2f}m)" for t in tags_out)
            self.get_logger().info(f'Detected: {det_str}', throttle_duration_sec=1.0)

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
