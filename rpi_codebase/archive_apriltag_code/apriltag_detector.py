#!/usr/bin/env python3
"""
apriltag_detector.py — Runs on RPi
===================================
Captures frames from RPi Camera v2 via picamera2, detects AprilTags,
publishes detections as custom messages on /apriltag/detections.

Tag ID conventions (configure to match your printed tags):
  STATIC_TAG_IDS  = [0, 1, 2]     — tags placed at/near Station A
  DYNAMIC_TAG_IDS = [10, 11, 12]   — tag on top/side of dynamic target (for docking)
  DYNAMIC_RECEPTACLE_IDS = [20, 21] — tag inside the receptacle hole (for shoot timing)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image

# We'll publish a simple JSON string for detections to avoid custom msg compilation.
# Format: {"tags": [{"id": 0, "type": "static", "cx": 320, "cy": 240,
#           "dist": 0.45, "yaw_offset": 0.12, "corners": [...]}]}
import json

# ── Configure these to match YOUR tag IDs ──
STATIC_TAG_IDS          = [0, 1, 2]
DYNAMIC_DOCK_TAG_IDS    = [10, 11, 12]
DYNAMIC_RECEPTACLE_IDS  = [20, 21]

# Camera intrinsics for RPi Camera v2 at 640x480 (approximate — calibrate yours!)
# fx, fy, cx, cy
CAMERA_PARAMS = (462.0, 462.0, 320.0, 240.0)
TAG_SIZE_M    = 0.05   # physical tag side length in metres — CHANGE to your actual size

DETECT_HZ = 15  # detection rate


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # ── Publishers ──
        self.det_pub  = self.create_publisher(String, '/apriltag/detections', 10)
        self.img_pub  = self.create_publisher(Image,  '/apriltag/image_raw', 1)

        # ── Camera init (picamera2) ──
        try:
            from picamera2 import Picamera2
            self.cam = Picamera2()
            config = self.cam.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"})
            self.cam.configure(config)
            self.cam.start()
            self.get_logger().info('RPi Camera v2 started (640x480)')
        except Exception as e:
            self.get_logger().error(f'Camera init failed: {e}')
            self.cam = None

        # ── AprilTag detector init (pupil-apriltags) ──
        try:
            from pupil_apriltags import Detector
            self.detector = Detector(
                families='tag36h11',
                nthreads=2,
                quad_decimate=2.0,
                quad_sigma=0.0,
                decode_sharpening=0.25,
            )
            self.get_logger().info('AprilTag detector (pupil-apriltags, tag36h11) ready')
        except ImportError:
            self.get_logger().error(
                'pupil-apriltags not installed! Run: pip3 install pupil-apriltags')
            self.detector = None

        self.create_timer(1.0 / DETECT_HZ, self._detect_tick)

    def _classify_tag(self, tag_id):
        if tag_id in STATIC_TAG_IDS:
            return 'static'
        elif tag_id in DYNAMIC_DOCK_TAG_IDS:
            return 'dynamic_dock'
        elif tag_id in DYNAMIC_RECEPTACLE_IDS:
            return 'dynamic_receptacle'
        return 'unknown'

    def _detect_tick(self):
        if self.cam is None or self.detector is None:
            return

        frame = self.cam.capture_array()
        if frame is None:
            return

        # Convert to grayscale for detection
        gray = np.mean(frame[:, :, :3], axis=2).astype(np.uint8)

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )

        tags_out = []
        for r in results:
            # r.pose_t is [3,1] translation: [x_right, y_down, z_forward] in camera frame
            # r.pose_R is [3,3] rotation matrix
            t = r.pose_t.flatten()
            dist = float(np.linalg.norm(t))

            # Horizontal offset angle from camera center (positive = tag is to the right)
            yaw_offset = float(math.atan2(t[0], t[2]))

            tag_type = self._classify_tag(r.tag_id)

            corners = r.corners.tolist()  # [[x,y], [x,y], [x,y], [x,y]]
            center  = r.center.tolist()   # [cx, cy]

            tags_out.append({
                'id':         int(r.tag_id),
                'type':       tag_type,
                'cx':         center[0],
                'cy':         center[1],
                'dist':       round(dist, 4),
                'yaw_offset': round(yaw_offset, 4),
                'tx':         round(float(t[0]), 4),
                'ty':         round(float(t[1]), 4),
                'tz':         round(float(t[2]), 4),
                'corners':    corners,
            })

        if tags_out:
            msg = String()
            msg.data = json.dumps({'tags': tags_out, 'stamp': time.time()})
            self.det_pub.publish(msg)

        # Optionally publish raw image for debugging (low rate)
        # Uncomment if you want to visualize in rqt_image_view
        # self._publish_image(frame)

    def _publish_image(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = frame.shape[0]
        msg.width  = frame.shape[1]
        msg.encoding = 'rgb8'
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.img_pub.publish(msg)

    def destroy_node(self):
        if self.cam is not None:
            try:
                self.cam.stop()
            except Exception:
                pass
        super().destroy_node()


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
