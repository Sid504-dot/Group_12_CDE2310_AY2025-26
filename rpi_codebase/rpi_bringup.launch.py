#!/usr/bin/env python3
"""
rpi_bringup.launch.py
=====================
Runs on the Raspberry Pi. Starts:
  1. TurtleBot3 robot bringup   (OpenCR, LiDAR, odometry)
  2. Camera node                (640x480 RGB888)
  3. AprilTag detector
  4. Dock controller            (/cmd_vel → /cmd_vel_dock for twist_mux)
  5. Launcher controller
  6. twist_mux                  (arbitrates /cmd_vel_dock vs /cmd_vel_nav → /cmd_vel)

Usage (RPi terminal):
  source /opt/ros/humble/setup.bash
  ros2 launch rpi_bringup.launch.py

twist_mux priority:
  dock_controller  (100) — wins whenever it is actively publishing
  Nav2 controller  ( 10) — active during exploration
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.expanduser('~/turtlebot3_ws/src/py_pubsub/py_pubsub')
CONFIG_DIR  = os.path.expanduser('~/turtlebot3_ws/src/py_pubsub/config')

TWIST_MUX_YAML = os.path.join(CONFIG_DIR, 'twist_mux.yaml')


def generate_launch_description():
    return LaunchDescription([

        # ── Environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',    '42'),

        # ── 1. TurtleBot3 bringup ─────────────────────────────────────────────
        # Starts OpenCR, LiDAR, odometry publishers.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_bringup'),
                '/launch/robot.launch.py',
            ])
        ),

        # ── 2. Camera node ────────────────────────────────────────────────────
        Node(
            package    = 'camera_ros',
            executable = 'camera_node',
            name       = 'camera_node',
            parameters = [{'width': 640, 'height': 480, 'format': 'RGB888'}],
            output     = 'screen',
        ),

        # ── 3. AprilTag detector ──────────────────────────────────────────────
        # Small delay so the camera node is publishing before detection starts.
        TimerAction(
            period = 3.0,
            actions = [
                ExecuteProcess(
                    cmd    = ['python3',
                              os.path.join(SCRIPT_DIR, 'apriltag_detector_revised2.py')],
                    name   = 'apriltag_detector',
                    output = 'screen',
                ),
            ]
        ),

        # ── 4. Dock controller ────────────────────────────────────────────────
        # Remapped: /cmd_vel → /cmd_vel_dock
        # twist_mux subscribes to /cmd_vel_dock at priority 100.
        ExecuteProcess(
            cmd = [
                'python3',
                os.path.join(SCRIPT_DIR, 'dock_controller_revised7.py'),
                '--ros-args', '-r', '/cmd_vel:=/cmd_vel_dock',
            ],
            name   = 'dock_controller',
            output = 'screen',
        ),

        # ── 5. Launcher controller ────────────────────────────────────────────
        ExecuteProcess(
            cmd    = ['python3',
                      os.path.join(SCRIPT_DIR, 'launcher_controller_revised3.py')],
            name   = 'launcher_controller',
            output = 'screen',
        ),

        # ── 6. twist_mux ──────────────────────────────────────────────────────
        # Subscribes: /cmd_vel_dock (priority 100), /cmd_vel_nav (priority 10)
        # Publishes:  /cmd_vel  → OpenCR
        Node(
            package    = 'twist_mux',
            executable = 'twist_mux',
            name       = 'twist_mux',
            parameters = [TWIST_MUX_YAML],
            remappings = [('/cmd_vel_out', '/cmd_vel')],
            output     = 'screen',
        ),

    ])
