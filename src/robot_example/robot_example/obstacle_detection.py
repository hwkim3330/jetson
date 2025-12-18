#!/usr/bin/env python3
#
# Copyright 2025 KETI
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleDetection(Node):
    """Obstacle detection node for KETI Robot."""

    def __init__(self):
        super().__init__('obstacle_detection')

        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('linear_velocity', 0.2)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.linear_velocity = self.get_parameter('linear_velocity').value

        self.scan_ranges = []
        self.init_scan_state = False

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)

        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_raw_callback,
            qos)

        self.timer = self.create_timer(0.1, self.update_callback)
        self.twist = Twist()

        self.get_logger().info('Obstacle detection node started')
        self.get_logger().info(f'Safety distance: {self.safety_distance}m')

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.init_scan_state = True

    def cmd_vel_raw_callback(self, msg):
        self.twist = msg

    def update_callback(self):
        if not self.init_scan_state:
            return

        twist = Twist()
        obstacle_detected = False

        # Check front 180 degrees for obstacles
        scan_len = len(self.scan_ranges)
        if scan_len > 0:
            # Front region (roughly -90 to +90 degrees)
            front_start = int(scan_len * 0.25)
            front_end = int(scan_len * 0.75)

            for i in range(front_start, front_end):
                if 0.0 < self.scan_ranges[i] < self.safety_distance:
                    obstacle_detected = True
                    break

        if obstacle_detected:
            # Stop forward motion but allow rotation
            twist.linear.x = 0.0
            twist.angular.z = self.twist.angular.z
            self.get_logger().info('Obstacle detected! Stopping.')
        else:
            twist.linear.x = self.twist.linear.x
            twist.angular.z = self.twist.angular.z

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
