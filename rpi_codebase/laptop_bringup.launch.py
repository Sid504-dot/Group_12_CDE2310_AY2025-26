#!/usr/bin/env python3
"""
laptop_bringup.launch.py
========================
Runs on the laptop. Starts:
  1. Cartographer SLAM          (builds map from LiDAR over network)
  2. Nav2 navigation stack      (planner + controller, publishes to /cmd_vel_nav)
  3. nav2_gap_nav               (frontier explorer, sends goals to Nav2)
  4. mission_coordinator        (state machine sequencing dock/fire/backup)

Usage (laptop terminal):
  source /opt/ros/humble/setup.bash
  source ~/colcon_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  ros2 launch ~/laptop_bringup.launch.py

NOTE — one required YAML change before launching:
  In nav2_params_frontier.yaml, add under controller_server → ros__parameters:
      cmd_vel_topic: /cmd_vel_nav
  This redirects Nav2's output away from /cmd_vel so twist_mux (on RPi) owns it.

Monitor mission state in a separate terminal:
  export ROS_DOMAIN_ID=42 && ros2 topic echo /mission/state
"""

import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# ── Paths ─────────────────────────────────────────────────────────────────────
NAV2_PARAMS = os.path.expanduser(
    '~/colcon_ws/src/auto_nav/config/nav2_params_frontier.yaml')


def generate_launch_description():
    return LaunchDescription([

        # ── Environment ───────────────────────────────────────────────────────
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',    '42'),

        # ── 1. Cartographer SLAM ──────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_cartographer'),
                '/launch/cartographer.launch.py',
            ])
        ),

        # ── 2. Nav2 navigation stack ──────────────────────────────────────────
        # controller_server publishes to /cmd_vel_nav (set in YAML).
        # twist_mux on the RPi subscribes to /cmd_vel_nav over DDS.
        TimerAction(
            period = 5.0,   # wait for Cartographer to publish /map
            actions = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        get_package_share_directory('nav2_bringup'),
                        '/launch/navigation_launch.py',
                    ]),
                    launch_arguments = {
                        'params_file':   NAV2_PARAMS,
                        'use_sim_time':  'false',
                    }.items(),
                ),
            ]
        ),

        # ── 3. nav2_gap_nav (frontier explorer) ───────────────────────────────
        # Sends NavigateToPose goals to Nav2. Does not publish /cmd_vel.
        TimerAction(
            period = 10.0,  # wait for Nav2 to fully initialise
            actions = [
                Node(
                    package    = 'auto_nav',
                    executable = 'sid_working_nav_old',
                    name       = 'nav2_gap_nav',
                    output     = 'screen',
                ),
            ]
        ),

        # ── 4. Mission coordinator ────────────────────────────────────────────
        # Sequences EXPLORING → DOCKING → FIRING → BACKING_UP → DONE.
        # Publishes pause/resume to nav2_gap_nav, dock/fire commands to RPi nodes.
        TimerAction(
            period = 10.0,  # same delay as nav2_gap_nav — both need Nav2 up
            actions = [
                Node(
                    package    = 'auto_nav',
                    executable = 'mission_coordinator_old',
                    name       = 'mission_coordinator',
                    output     = 'screen',
                ),
            ]
        ),

    ])
