#!/usr/bin/env python3
#
# Copyright 2025 KETI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Authors: KETI AI Robot Team

"""
KETI Robot Mode Controller
Manages different operation modes:
- manual: Direct joystick control
- obstacle: Obstacle avoidance (auto stop)
- patrol: Autonomous patrol (square, triangle, circle)
- follower: Follow detected person
- lane: Lane following
- parking: Auto parking
- yolo: YOLO object detection
"""

import subprocess
import signal
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ModeController(Node):
    """Robot mode controller node"""

    def __init__(self):
        super().__init__('mode_controller')

        self.current_mode = 'manual'
        self.mode_processes = []
        self.obstacle_distance = 0.35  # Stop distance in meters

        # Scan data
        self.scan_ranges = []
        self.has_scan = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mode/status', 10)
        self.patrol_cmd_pub = self.create_publisher(String, '/patrol/command', 10)
        self.parking_cmd_pub = self.create_publisher(String, '/parking/command', 10)
        self.lane_enable_pub = self.create_publisher(Bool, '/lane_follower/enable', 10)

        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/robot/mode', self.mode_callback, 10
        )

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Timer for obstacle mode
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Store raw velocity commands
        self.raw_twist = Twist()

        self.get_logger().info('KETI Mode Controller started')
        self.get_logger().info(f'Current mode: {self.current_mode}')
        self.publish_status(f'Mode: {self.current_mode}')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def mode_callback(self, msg):
        """Handle mode change requests"""
        new_mode = msg.data.lower()

        if new_mode == self.current_mode:
            return

        self.get_logger().info(f'Mode change: {self.current_mode} -> {new_mode}')

        # Stop any running mode processes
        self.stop_mode_processes()

        # Stop robot
        self.stop_robot()

        self.current_mode = new_mode
        self.publish_status(f'Mode: {new_mode}')

        # Start new mode
        if new_mode == 'manual':
            pass  # No additional process needed

        elif new_mode == 'obstacle':
            # Start obstacle_avoidance node
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'obstacle_avoidance.py'
            ])

        elif new_mode == 'patrol':
            # Start patrol node
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'patrol.py'
            ])
            # Send start command after node starts
            self.create_timer(1.0, self.start_patrol, one_shot=True)

        elif new_mode == 'follower':
            # Start person follower
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'person_follower.py'
            ])

        elif new_mode == 'lane':
            # Start lane detector and follower
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'lane_detector.py'
            ])
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'lane_follower.py'
            ])
            # Enable lane following
            self.create_timer(1.0, self.enable_lane, one_shot=True)

        elif new_mode == 'parking':
            # Start auto parking
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'auto_parking.py'
            ])
            # Send start command
            self.create_timer(1.0, self.start_parking, one_shot=True)

        elif new_mode == 'yolo':
            # Start YOLO detector
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'yolo_detector.py'
            ])

    def start_patrol(self):
        """Send start command to patrol node"""
        msg = String()
        msg.data = 'start'
        self.patrol_cmd_pub.publish(msg)
        self.get_logger().info('Patrol started')

    def start_parking(self):
        """Send start command to parking node"""
        msg = String()
        msg.data = 'start'
        self.parking_cmd_pub.publish(msg)
        self.get_logger().info('Parking started')

    def enable_lane(self):
        """Enable lane following"""
        msg = Bool()
        msg.data = True
        self.lane_enable_pub.publish(msg)
        self.get_logger().info('Lane following enabled')

    def start_mode_process(self, cmd):
        """Start a mode subprocess"""
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.mode_processes.append(process)
            self.get_logger().info(f'Started mode process: {" ".join(cmd)}')
        except Exception as e:
            self.get_logger().error(f'Failed to start mode process: {e}')

    def stop_mode_processes(self):
        """Stop all mode subprocesses"""
        for process in self.mode_processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.mode_processes = []

        # Disable lane following
        msg = Bool()
        msg.data = False
        self.lane_enable_pub.publish(msg)

    def cmd_vel_raw_callback(self, msg):
        """Store raw velocity commands"""
        self.raw_twist = msg

    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.scan_ranges = msg.ranges
        self.has_scan = True

    def timer_callback(self):
        """Periodic processing based on mode"""
        if self.current_mode == 'obstacle' and self.has_scan:
            self.obstacle_avoidance()
        elif self.current_mode == 'manual':
            # Pass through raw commands
            self.cmd_vel_pub.publish(self.raw_twist)

    def obstacle_avoidance(self):
        """Simple obstacle avoidance"""
        if not self.scan_ranges:
            return

        # Check front 90 degrees
        num_ranges = len(self.scan_ranges)
        front_start = int(num_ranges * 0.375)  # -45 degrees
        front_end = int(num_ranges * 0.625)    # +45 degrees

        # Get minimum distance in front
        front_ranges = self.scan_ranges[front_start:front_end]
        valid_ranges = [r for r in front_ranges if r > 0.01 and r < 10.0]

        if valid_ranges:
            min_distance = min(valid_ranges)
        else:
            min_distance = 10.0

        twist = Twist()

        if min_distance < self.obstacle_distance:
            # Obstacle detected - stop forward motion
            twist.linear.x = 0.0
            twist.angular.z = self.raw_twist.angular.z  # Allow rotation
            self.get_logger().info(
                f'Obstacle at {min_distance:.2f}m - stopping',
                throttle_duration_sec=2.0
            )
        else:
            # No obstacle - pass through commands
            twist = self.raw_twist

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Stop the robot"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Cleanup"""
        self.stop_mode_processes()
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModeController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
