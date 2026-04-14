#!/usr/bin/env python3
"""
apriltag_detector.py — Runs on RPi
===================================
Subscribes to /camera/image_raw from camera_ros node,
detects AprilTags, publishes detections on /apriltag/detections.

Requires:
  - camera_ros running: ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 -p format:=RGB888
  - pupil-apriltags: pip3 install pupil-apriltags

Tag ID conventions (configure to match your printed tags):
  STATIC_TAG_IDS         = [0, 1, 2]     — tags at Station A
  DYNAMIC_DOCK_TAG_IDS   = [10, 11, 12]  — tag on top/side of dynamic target (for docking)
  DYNAMIC_RECEPTACLE_IDS = [20, 21]       — tag inside the receptacle (for shoot timing)
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from sensor_msgs.msg import Image

import json

# ── Configure these to match YOUR tag IDs ──
STATIC_TAG_IDS          = [0, 1, 2]
DYNAMIC_DOCK_TAG_IDS    = [10, 11, 12]
DYNAMIC_RECEPTACLE_IDS  = [20, 21]

# Camera intrinsics for RPi Camera v2 at 640x480 (approximate — calibrate yours!)
# fx, fy, cx, cy
CAMERA_PARAMS = (462.0, 462.0, 320.0, 240.0)
TAG_SIZE_M    = 0.05   # physical tag side length in metres — CHANGE to your actual size


class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # ── Publishers ──
        self.det_pub = self.create_publisher(String, '/apriltag/detections', 10)

        # ── Subscribe to camera image ──
        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, qos_profile_sensor_data)
        self.get_logger().info('Subscribed to /camera/image_raw')

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

    def _classify_tag(self, tag_id):
        if tag_id in STATIC_TAG_IDS:
            return 'static'
        elif tag_id in DYNAMIC_DOCK_TAG_IDS:
            return 'dynamic_dock'
        elif tag_id in DYNAMIC_RECEPTACLE_IDS:
            return 'dynamic_receptacle'
        return 'unknown'

    def _image_cb(self, msg: Image):
        if self.detector is None:
            return

        # Convert ROS Image to numpy array
        try:
            if msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
            elif msg.encoding == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3))
                frame = frame[:, :, ::-1]  # BGR to RGB
            elif msg.encoding == 'mono8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width))
            else:
                self.get_logger().warn(f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=5.0)
                return
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}', throttle_duration_sec=5.0)
            return

        # Convert to grayscale
        if len(frame.shape) == 3:
            gray = np.mean(frame[:, :, :3], axis=2).astype(np.uint8)
        else:
            gray = frame

        results = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )

        tags_out = []
        for r in results:
            t = r.pose_t.flatten()
            dist = float(np.linalg.norm(t))
            yaw_offset = float(math.atan2(t[0], t[2]))
            tag_type = self._classify_tag(r.tag_id)
            corners = r.corners.tolist()
            center = r.center.tolist()

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
            out_msg = String()
            out_msg.data = json.dumps({'tags': tags_out, 'stamp': time.time()})
            self.det_pub.publish(out_msg)
            self.get_logger().info(
                f'Detected {len(tags_out)} tag(s): {[t["id"] for t in tags_out]}',
                throttle_duration_sec=1.0)


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
