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
KETI Robot Obstacle Avoidance Node
- Monitors LiDAR data for obstacles
- Filters cmd_vel_raw commands to prevent collisions
- Publishes filtered commands to cmd_vel
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


class ObstacleAvoidanceNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.get_logger().info('KETI Robot Obstacle Avoidance Started')

        # Parameters
        self.declare_parameter('stop_distance', 0.35)  # meters
        self.declare_parameter('warn_distance', 0.6)   # meters
        self.declare_parameter('scan_angle', 90)       # degrees (front arc)
        self.declare_parameter('enabled', True)

        self.stop_distance = self.get_parameter('stop_distance').value
        self.warn_distance = self.get_parameter('warn_distance').value
        self.scan_angle = self.get_parameter('scan_angle').value
        self.enabled = self.get_parameter('enabled').value

        # State
        self.scan_ranges = []
        self.has_scan = False
        self.raw_twist = Twist()
        self.min_distance = float('inf')
        self.obstacle_detected = False

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle/detected', qos)
        self.distance_pub = self.create_publisher(Float32, '/obstacle/distance', qos)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, 'cmd_vel_raw', self.cmd_vel_raw_callback, qos_profile_sensor_data)
        self.enable_sub = self.create_subscription(
            Bool, '/obstacle/enable', self.enable_callback, qos)

        # Timer for processing
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.get_logger().info(f'Stop distance: {self.stop_distance}m, Scan angle: ±{self.scan_angle}°')

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan = True
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def cmd_vel_raw_callback(self, msg):
        self.raw_twist = msg

    def enable_callback(self, msg):
        self.enabled = msg.data
        status = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f'Obstacle avoidance {status}')

    def timer_callback(self):
        if not self.has_scan:
            return

        # Calculate front arc indices
        num_ranges = len(self.scan_ranges)
        if num_ranges == 0:
            return

        # Assuming scan goes from -180 to 180 or 0 to 360
        angle_per_index = 360.0 / num_ranges
        front_indices = int(self.scan_angle / angle_per_index)

        # Get front arc ranges (left and right of center)
        left_start = 0
        left_end = min(front_indices, num_ranges)
        right_start = max(num_ranges - front_indices, 0)
        right_end = num_ranges

        front_ranges = []
        for i in range(left_start, left_end):
            r = self.scan_ranges[i]
            if 0.05 < r < 10.0:  # Valid range
                front_ranges.append(r)
        for i in range(right_start, right_end):
            r = self.scan_ranges[i]
            if 0.05 < r < 10.0:
                front_ranges.append(r)

        if front_ranges:
            self.min_distance = min(front_ranges)
        else:
            self.min_distance = float('inf')

        # Publish distance
        dist_msg = Float32()
        dist_msg.data = float(self.min_distance)
        self.distance_pub.publish(dist_msg)

        # Check for obstacle
        self.obstacle_detected = self.min_distance < self.stop_distance

        # Publish obstacle status
        obstacle_msg = Bool()
        obstacle_msg.data = self.obstacle_detected
        self.obstacle_pub.publish(obstacle_msg)

        # Filter velocity commands
        twist = Twist()
        if self.enabled and self.obstacle_detected:
            # Obstacle detected - stop forward motion but allow rotation
            if self.raw_twist.linear.x > 0:
                twist.linear.x = 0.0
                self.get_logger().warn(
                    f'Obstacle at {self.min_distance:.2f}m! Stopping.',
                    throttle_duration_sec=1.0)
            else:
                twist.linear.x = self.raw_twist.linear.x  # Allow reverse
            twist.angular.z = self.raw_twist.angular.z
        else:
            twist = self.raw_twist

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
