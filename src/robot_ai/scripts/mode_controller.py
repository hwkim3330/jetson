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
- slam: SLAM mapping with cartographer
- nav: Navigation with Nav2

AI Detection modes:
- yolo: YOLO object detection
- body: Body tracking
- gesture: Gesture detection
- person: Person detection
- multi: Multi AI engine

Example commands:
- forward, backward, left, right
- square, triangle, circle
- stop
"""

import subprocess
import signal
import os
import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class ModeController(Node):
    """Robot mode controller node"""

    def __init__(self):
        super().__init__('mode_controller')

        self.current_mode = 'manual'
        self.current_ai = 'off'
        self.mode_processes = []
        self.ai_processes = []
        self.example_thread = None
        self.example_running = False
        self.obstacle_distance = 0.35  # Stop distance in meters

        # Odometry data
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Scan data
        self.scan_ranges = []
        self.has_scan = False

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mode/status', 10)
        self.patrol_cmd_pub = self.create_publisher(String, '/patrol/command', 10)
        self.parking_cmd_pub = self.create_publisher(String, '/parking/command', 10)
        self.lane_enable_pub = self.create_publisher(Bool, '/lane_follower/enable', 10)
        self.gesture_enable_pub = self.create_publisher(Bool, '/gesture_enable', 10)

        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/robot/mode', self.mode_callback, 10
        )

        self.ai_mode_sub = self.create_subscription(
            String, '/robot/ai_mode', self.ai_mode_callback, 10
        )

        self.save_map_sub = self.create_subscription(
            String, '/robot/save_map', self.save_map_callback, 10
        )

        self.load_map_sub = self.create_subscription(
            String, '/robot/load_map', self.load_map_callback, 10
        )

        # Track current loaded map slot
        self.current_map_slot = 'default'

        self.example_sub = self.create_subscription(
            String, '/robot/example', self.example_callback, 10
        )

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            qos_profile=qos_profile_sensor_data
        )

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Timer for obstacle mode
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Store raw velocity commands
        self.raw_twist = Twist()

        self.get_logger().info('KETI Mode Controller started')
        self.get_logger().info(f'Current mode: {self.current_mode}')
        self.publish_status(f'Mode: {self.current_mode}')

        # Auto-start AI detectors after 3 seconds
        self._auto_start_timer = self.create_timer(3.0, self._auto_start_ai)

        # Watchdog: monitor AI processes every 10 seconds
        self._watchdog_timer = self.create_timer(10.0, self._watchdog_check)
        self._ai_auto_started = False

    def _watchdog_check(self):
        """Check if AI processes are alive and restart if crashed"""
        if not self._ai_auto_started:
            return

        import subprocess

        # Check YOLO
        result = subprocess.run(['pgrep', '-f', 'yolo_detector'], capture_output=True)
        if result.returncode != 0:
            self.get_logger().warn('YOLO detector crashed - restarting...')
            self.start_ai_process(['ros2', 'run', 'robot_ai', 'yolo_detector.py'])

        # Check Gesture
        result = subprocess.run(['pgrep', '-f', 'gesture_detector'], capture_output=True)
        if result.returncode != 0:
            self.get_logger().warn('Gesture detector crashed - restarting...')
            self.start_ai_process(['ros2', 'run', 'robot_ai', 'gesture_detector.py'])

    def _auto_start_ai(self):
        """Auto-start YOLO and Gesture detectors on boot"""
        # Cancel timer (one-shot behavior for ROS2 Humble compatibility)
        if self._auto_start_timer:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None

        self.get_logger().info('Auto-starting AI detectors...')

        # Start YOLO detector
        self.start_ai_process(['ros2', 'run', 'robot_ai', 'yolo_detector.py'])
        self.get_logger().info('YOLO detector auto-started')

        # Start Gesture detector
        self.start_ai_process(['ros2', 'run', 'robot_ai', 'gesture_detector.py'])
        self.get_logger().info('Gesture detector auto-started')

        self._ai_auto_started = True

        # Enable gesture control after detectors initialize
        self._gesture_enable_timer = self.create_timer(2.0, self._enable_gesture_control)

    def _enable_gesture_control(self):
        """Enable gesture control for robot movement"""
        # One-shot: cancel timer
        if self._gesture_enable_timer:
            self._gesture_enable_timer.cancel()
            self._gesture_enable_timer = None

        msg = Bool()
        msg.data = True
        self.gesture_enable_pub.publish(msg)
        self.get_logger().info('Gesture control enabled')
        self.publish_status('AI: yolo+gesture (auto)')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny, cosy)

    def mode_callback(self, msg):
        """Handle mode change requests"""
        new_mode = msg.data.lower()

        if new_mode == self.current_mode:
            return

        self.get_logger().info(f'Mode change: {self.current_mode} -> {new_mode}')

        # Stop any running mode processes
        self.stop_mode_processes()

        # Stop example if running
        self.stop_example()

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
            self._patrol_timer = self.create_timer(1.0, self._start_patrol_once)

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
            self._lane_timer = self.create_timer(1.0, self._enable_lane_once)

        elif new_mode == 'parking':
            # Start auto parking
            self.start_mode_process([
                'ros2', 'run', 'robot_ai', 'auto_parking.py'
            ])
            # Send start command
            self._parking_timer = self.create_timer(1.0, self._start_parking_once)

        elif new_mode == 'slam':
            # Start SLAM (cartographer)
            self.start_mode_process([
                'ros2', 'launch', 'robot_slam', 'cartographer.launch.py'
            ])
            self.get_logger().info('SLAM started - drive around to map')

        elif new_mode == 'nav':
            # Start Navigation
            self.start_mode_process([
                'ros2', 'launch', 'robot_navigation', 'navigation2.launch.py'
            ])
            self.get_logger().info('Navigation started')

        elif new_mode == 'explore':
            # Start SLAM + Auto Explore (robot cleaner mode)
            self.start_mode_process([
                'ros2', 'launch', 'robot_slam', 'cartographer.launch.py'
            ])
            self.get_logger().info('SLAM started for auto explore')
            # Start auto explore after SLAM initializes
            self._explore_timer = self.create_timer(2.0, self._start_auto_explore_once)

    def _start_auto_explore_once(self):
        """One-shot wrapper for start_auto_explore"""
        if hasattr(self, '_explore_timer') and self._explore_timer:
            self._explore_timer.cancel()
            self._explore_timer = None
        self.start_auto_explore()

    def start_auto_explore(self):
        """Start auto explore script"""
        try:
            process = subprocess.Popen([
                'ros2', 'run', 'robot_ai', 'auto_explore.py'
            ], preexec_fn=os.setsid)
            self.mode_processes.append(process)
            self.get_logger().info('Auto explore started')
        except Exception as e:
            self.get_logger().error(f'Failed to start auto explore: {e}')

    def save_map_callback(self, msg):
        """Handle map save request - supports slot-based saving

        Message formats:
        - 'save' or 'save:default' -> save to /maps/map.yaml (for Nav2)
        - 'save:0' to 'save:9' -> save to /maps/slot_0/ to /maps/slot_9/
        """
        data = msg.data.lower().strip()

        if not data.startswith('save'):
            return

        # Parse slot number
        parts = data.split(':')
        slot = parts[1] if len(parts) > 1 else 'default'

        map_base_dir = '/home/nvidia/ros2_ws/maps'

        if slot == 'default' or slot == 'save':
            # Default save location for Nav2
            map_dir = map_base_dir
            map_path = f'{map_dir}/map'
        else:
            # Slot-based save
            try:
                slot_num = int(slot)
                if slot_num < 0 or slot_num > 9:
                    self.get_logger().error(f'Invalid slot number: {slot_num}')
                    return
                map_dir = f'{map_base_dir}/slot_{slot_num}'
                map_path = f'{map_dir}/map'
            except ValueError:
                self.get_logger().error(f'Invalid slot: {slot}')
                return

        self.get_logger().info(f'Saving map to {map_path}...')

        # Create directory if not exists
        os.makedirs(map_dir, exist_ok=True)

        # Run map saver
        try:
            result = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path, '--ros-args', '-p', 'save_map_timeout:=10.0'
            ], timeout=15, capture_output=True, text=True)

            if result.returncode == 0:
                self.get_logger().info(f'Map saved to {map_path}.yaml')
                # Also copy to default location for Nav2 if saving to slot
                if slot != 'default' and slot != 'save':
                    import shutil
                    shutil.copy(f'{map_path}.yaml', f'{map_base_dir}/map.yaml')
                    shutil.copy(f'{map_path}.pgm', f'{map_base_dir}/map.pgm')
                    self.get_logger().info(f'Also copied to {map_base_dir}/map.yaml for Nav2')
            else:
                self.get_logger().error(f'Map save failed: {result.stderr}')
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out')
        except Exception as e:
            self.get_logger().error(f'Map save failed: {e}')

    def load_map_callback(self, msg):
        """Handle map load request - for loading saved maps into Nav2

        Message formats:
        - 'load' or 'load:default' -> load from /maps/map.yaml
        - 'load:0' to 'load:9' -> load from /maps/slot_0/ to /maps/slot_9/
        - 'list' -> list available map slots (publishes to /robot/map_slots)
        """
        data = msg.data.lower().strip()

        map_base_dir = '/home/nvidia/ros2_ws/maps'

        if data == 'list':
            # List available map slots
            slots = []
            # Check default
            if os.path.exists(f'{map_base_dir}/map.yaml'):
                slots.append('default')
            # Check slots 0-9
            for i in range(10):
                if os.path.exists(f'{map_base_dir}/slot_{i}/map.yaml'):
                    slots.append(str(i))
            self.get_logger().info(f'Available map slots: {slots}')
            # Publish slot list
            slot_msg = String()
            slot_msg.data = json.dumps({'slots': slots, 'current': self.current_map_slot})
            self.status_pub.publish(slot_msg)
            return

        if not data.startswith('load'):
            return

        # Parse slot number
        parts = data.split(':')
        slot = parts[1] if len(parts) > 1 else 'default'

        if slot == 'default' or slot == 'load':
            map_path = f'{map_base_dir}/map.yaml'
        else:
            try:
                slot_num = int(slot)
                if slot_num < 0 or slot_num > 9:
                    self.get_logger().error(f'Invalid slot number: {slot_num}')
                    return
                map_path = f'{map_base_dir}/slot_{slot_num}/map.yaml'
            except ValueError:
                self.get_logger().error(f'Invalid slot: {slot}')
                return

        # Check if file exists
        if not os.path.exists(map_path):
            self.get_logger().error(f'Map not found: {map_path}')
            return

        self.get_logger().info(f'Loading map from {map_path}')
        self.current_map_slot = slot

        # Copy to default location for Nav2 to use
        if slot != 'default' and slot != 'load':
            import shutil
            slot_dir = f'{map_base_dir}/slot_{slot}'
            shutil.copy(f'{slot_dir}/map.yaml', f'{map_base_dir}/map.yaml')
            shutil.copy(f'{slot_dir}/map.pgm', f'{map_base_dir}/map.pgm')
            self.get_logger().info(f'Map slot {slot} copied to default location')

    def ai_mode_callback(self, msg):
        """Handle AI mode change requests"""
        new_ai = msg.data.lower()

        # Handle toggle commands (yolo_on, yolo_off, gesture_on, gesture_off)
        if new_ai.endswith('_on') or new_ai.endswith('_off'):
            self._handle_ai_toggle(new_ai)
            return

        if new_ai == self.current_ai:
            return

        self.get_logger().info(f'AI mode change: {self.current_ai} -> {new_ai}')

        # Stop any running AI processes
        self.stop_ai_processes()

        self.current_ai = new_ai
        self.publish_status(f'AI: {new_ai}')

        if new_ai == 'off':
            pass  # No AI running

        elif new_ai == 'yolo':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'yolo_detector.py'
            ])

        elif new_ai == 'body':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'body_tracker.py'
            ])

        elif new_ai == 'gesture':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'gesture_detector.py'
            ])

        elif new_ai == 'person':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'person_detector.py'
            ])

        elif new_ai == 'multi':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'multi_ai_engine.py'
            ])

        elif new_ai == 'yolo+gesture':
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'yolo_detector.py'
            ])
            self.start_ai_process([
                'ros2', 'run', 'robot_ai', 'gesture_detector.py'
            ])

    def _handle_ai_toggle(self, command):
        """Handle individual AI toggle commands"""
        import subprocess
        parts = command.rsplit('_', 1)
        ai_type = parts[0]  # yolo or gesture
        action = parts[1]   # on or off

        if action == 'on':
            if ai_type == 'yolo':
                # Check if already running
                result = subprocess.run(['pgrep', '-f', 'yolo_detector'], capture_output=True)
                if result.returncode != 0:
                    self.start_ai_process(['ros2', 'run', 'robot_ai', 'yolo_detector.py'])
                    self.get_logger().info('YOLO detector started')
            elif ai_type == 'gesture':
                result = subprocess.run(['pgrep', '-f', 'gesture_detector'], capture_output=True)
                if result.returncode != 0:
                    self.start_ai_process(['ros2', 'run', 'robot_ai', 'gesture_detector.py'])
                    self.get_logger().info('Gesture detector started')
        else:  # off
            if ai_type == 'yolo':
                subprocess.run(['pkill', '-f', 'yolo_detector'], capture_output=True)
                self.get_logger().info('YOLO detector stopped')
            elif ai_type == 'gesture':
                subprocess.run(['pkill', '-f', 'gesture_detector'], capture_output=True)
                self.get_logger().info('Gesture detector stopped')

    def example_callback(self, msg):
        """Handle example commands from web UI"""
        try:
            data = json.loads(msg.data)
            command = data.get('command', 'stop')
            distance = data.get('distance', 1.0)
            speed = data.get('speed', 0.2)

            self.get_logger().info(f'Example command: {command}, dist={distance}, spd={speed}')

            if command == 'stop':
                self.stop_example()
                self.stop_robot()
                return

            # Stop previous example
            self.stop_example()

            # Start new example in thread
            self.example_running = True
            self.example_thread = threading.Thread(
                target=self.run_example,
                args=(command, distance, speed)
            )
            self.example_thread.start()

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid example command JSON: {e}')

    def run_example(self, command, distance, speed):
        """Run example movement in a thread"""
        try:
            if command == 'forward':
                self.move_distance(distance, speed)
            elif command == 'backward':
                self.move_distance(distance, -speed)
            elif command == 'left':
                self.turn_angle(90.0, speed * 2)
            elif command == 'right':
                self.turn_angle(-90.0, speed * 2)
            elif command == 'square':
                self.move_pattern('square', distance, speed)
            elif command == 'triangle':
                self.move_pattern('triangle', distance, speed)
            elif command == 'circle':
                self.move_pattern('circle', distance, speed)

        except Exception as e:
            self.get_logger().error(f'Example error: {e}')
        finally:
            self.example_running = False
            self.stop_robot()

    def move_distance(self, distance, speed):
        """Move forward/backward by distance"""
        start_x, start_y = self.odom_x, self.odom_y

        twist = Twist()
        twist.linear.x = speed

        while self.example_running:
            dx = self.odom_x - start_x
            dy = self.odom_y - start_y
            traveled = math.sqrt(dx*dx + dy*dy)

            if traveled >= distance:
                break

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()

    def turn_angle(self, angle_deg, angular_speed):
        """Turn by angle in degrees"""
        start_yaw = self.odom_yaw
        target_angle = math.radians(angle_deg)

        twist = Twist()
        twist.angular.z = angular_speed if angle_deg > 0 else -angular_speed

        while self.example_running:
            yaw_diff = self.normalize_angle(self.odom_yaw - start_yaw)

            if abs(yaw_diff) >= abs(target_angle) * 0.95:
                break

            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        self.stop_robot()

    def move_pattern(self, pattern, size, speed):
        """Move in a pattern"""
        if pattern == 'square':
            for i in range(4):
                if not self.example_running:
                    break
                self.move_distance(size, speed)
                self.turn_angle(90.0, speed * 2)

        elif pattern == 'triangle':
            for i in range(3):
                if not self.example_running:
                    break
                self.move_distance(size, speed)
                self.turn_angle(120.0, speed * 2)

        elif pattern == 'circle':
            # Circle: constant linear + angular velocity
            twist = Twist()
            twist.linear.x = speed
            twist.angular.z = speed / (size / 2)  # v = r * omega

            import time
            start_time = time.time()
            circle_duration = 2 * math.pi * (size / 2) / speed  # circumference / speed

            while self.example_running and (time.time() - start_time) < circle_duration:
                self.cmd_vel_pub.publish(twist)
                rclpy.spin_once(self, timeout_sec=0.05)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_example(self):
        """Stop running example"""
        self.example_running = False
        if self.example_thread and self.example_thread.is_alive():
            self.example_thread.join(timeout=1.0)
        self.example_thread = None

    def _start_patrol_once(self):
        """One-shot wrapper for start_patrol"""
        if hasattr(self, '_patrol_timer') and self._patrol_timer:
            self._patrol_timer.cancel()
            self._patrol_timer = None
        self.start_patrol()

    def start_patrol(self):
        """Send start command to patrol node"""
        msg = String()
        msg.data = 'start'
        self.patrol_cmd_pub.publish(msg)
        self.get_logger().info('Patrol started')

    def _start_parking_once(self):
        """One-shot wrapper for start_parking"""
        if hasattr(self, '_parking_timer') and self._parking_timer:
            self._parking_timer.cancel()
            self._parking_timer = None
        self.start_parking()

    def start_parking(self):
        """Send start command to parking node"""
        msg = String()
        msg.data = 'start'
        self.parking_cmd_pub.publish(msg)
        self.get_logger().info('Parking started')

    def _enable_lane_once(self):
        """One-shot wrapper for enable_lane"""
        if hasattr(self, '_lane_timer') and self._lane_timer:
            self._lane_timer.cancel()
            self._lane_timer = None
        self.enable_lane()

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

    def start_ai_process(self, cmd):
        """Start an AI subprocess"""
        try:
            process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.ai_processes.append(process)
            self.get_logger().info(f'Started AI process: {" ".join(cmd)}')
        except Exception as e:
            self.get_logger().error(f'Failed to start AI process: {e}')

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

    def stop_ai_processes(self):
        """Stop all AI subprocesses"""
        for process in self.ai_processes:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                process.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                except Exception:
                    pass
        self.ai_processes = []

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
        # Manual mode: don't publish - let web joystick control directly via rosbridge

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
        self.stop_example()
        self.stop_mode_processes()
        self.stop_ai_processes()
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
