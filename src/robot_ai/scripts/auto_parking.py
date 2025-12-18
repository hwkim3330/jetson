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
# Based on TurtleBot3 example by ROBOTIS CO., LTD.

"""
KETI Robot Automatic Parking Node
- Uses LiDAR to detect parking spot markers (high intensity)
- Performs autonomous parking sequence
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import numpy as np


class AutoParkingNode(Node):

    def __init__(self):
        super().__init__('auto_parking')
        self.get_logger().info('KETI Robot Auto Parking Started')

        # Parameters
        self.declare_parameter('intensity_threshold', 150)
        self.declare_parameter('parking_speed', 0.05)
        self.declare_parameter('rotation_speed', 0.2)
        self.declare_parameter('min_spot_distance', 0.2)

        self.intensity_threshold = self.get_parameter('intensity_threshold').value
        self.parking_speed = self.get_parameter('parking_speed').value
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.min_spot_distance = self.get_parameter('min_spot_distance').value

        # State
        self.scan = None
        self.odom = None
        self.is_running = False
        self.state = 'idle'  # idle, searching, aligning, approaching, complete
        self.spot_angle = None
        self.spot_distance = None
        self.init_yaw = 0.0
        self.current_yaw = 0.0

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(String, '/parking/status', qos)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)
        self.cmd_sub = self.create_subscription(
            String, '/parking/command', self.command_callback, qos)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Send "start" to /parking/command to begin')

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom = msg
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def command_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'start':
            self.start_parking()
        elif cmd == 'stop':
            self.stop_parking()

    def start_parking(self):
        if self.scan is None:
            self.get_logger().warn('No LiDAR data!')
            return
        self.is_running = True
        self.state = 'searching'
        self.get_logger().info('Starting auto parking...')
        self.publish_status('Searching for parking spot')

    def stop_parking(self):
        self.is_running = False
        self.state = 'idle'
        self.stop_robot()
        self.publish_status('Parking stopped')

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def find_parking_spot(self):
        """Find parking spot by detecting high intensity reflectors"""
        if self.scan is None or not hasattr(self.scan, 'intensities'):
            return False

        if len(self.scan.intensities) == 0:
            return False

        # Search for high intensity markers
        spot_indices = []
        for i in range(len(self.scan.intensities)):
            # Skip front and back (only search sides)
            angle_deg = i * 360.0 / len(self.scan.intensities)
            if angle_deg < 30 or angle_deg > 330:
                continue
            if 150 < angle_deg < 210:
                continue

            # Check intensity
            if self.scan.ranges[i] > 0.1 and self.scan.ranges[i] < 2.0:
                intensity_score = (self.scan.intensities[i] ** 2) * self.scan.ranges[i] / 100000
                if intensity_score > self.intensity_threshold:
                    spot_indices.append(i)

        if len(spot_indices) > 10:
            # Find center of detected spot
            center_idx = spot_indices[len(spot_indices) // 2]
            self.spot_angle = center_idx * 360.0 / len(self.scan.ranges) - 180
            self.spot_distance = self.scan.ranges[center_idx]
            return True

        return False

    def timer_callback(self):
        if not self.is_running:
            return

        if self.scan is None or self.odom is None:
            return

        twist = Twist()

        if self.state == 'searching':
            if self.find_parking_spot():
                self.state = 'aligning'
                self.init_yaw = self.current_yaw
                self.get_logger().info(
                    f'Spot found at angle={self.spot_angle:.1f}°, distance={self.spot_distance:.2f}m')
                self.publish_status(f'Spot found, aligning...')
            else:
                # Slowly rotate to search
                twist.angular.z = 0.1
                self.publish_status('Searching...')

        elif self.state == 'aligning':
            # Rotate to face parking spot
            target_yaw = self.normalize_angle(self.init_yaw + math.radians(self.spot_angle))
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)

            if abs(yaw_error) > 0.05:
                twist.angular.z = self.rotation_speed if yaw_error > 0 else -self.rotation_speed
            else:
                self.state = 'approaching'
                self.publish_status('Approaching spot')

        elif self.state == 'approaching':
            # Check front distance
            front_ranges = []
            num_ranges = len(self.scan.ranges)
            front_arc = int(num_ranges * 30 / 360)  # ±30 degrees

            for i in list(range(0, front_arc)) + list(range(num_ranges - front_arc, num_ranges)):
                r = self.scan.ranges[i]
                if 0.05 < r < 5.0:
                    front_ranges.append(r)

            if front_ranges:
                min_front = min(front_ranges)
                if min_front > self.min_spot_distance:
                    twist.linear.x = self.parking_speed
                else:
                    self.state = 'complete'
                    self.is_running = False
                    self.get_logger().info('Parking complete!')
                    self.publish_status('Parking complete!')

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = AutoParkingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
