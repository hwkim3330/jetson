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
KETI Robot Lane Follower Node
- Subscribes to lane offset from lane_detector
- Controls robot to follow lane center
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool, String


class LaneFollowerNode(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.get_logger().info('KETI Robot Lane Follower Started')

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_gain', 0.003)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('enabled', False)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_angular = self.get_parameter('max_angular').value
        self.enabled = self.get_parameter('enabled').value

        # State
        self.lane_offset = 0.0
        self.has_lane = False

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(String, '/lane_follower/status', qos)

        # Subscribers
        self.offset_sub = self.create_subscription(
            Float32, '/lane/offset', self.offset_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/lane_follower/enable', self.enable_callback, qos)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        self.get_logger().info('Listening on /lane/offset')

    def offset_callback(self, msg):
        self.lane_offset = msg.data
        self.has_lane = True

    def enable_callback(self, msg):
        self.enabled = msg.data
        status = 'enabled' if self.enabled else 'disabled'
        self.get_logger().info(f'Lane follower {status}')
        self.publish_status(f'Lane follower {status}')
        if not self.enabled:
            self.stop_robot()

    def timer_callback(self):
        if not self.enabled or not self.has_lane:
            return

        twist = Twist()

        # Proportional control based on lane offset
        # Negative offset = lane center is left of image center -> turn left
        angular_vel = -self.angular_gain * self.lane_offset
        angular_vel = max(min(angular_vel, self.max_angular), -self.max_angular)

        twist.linear.x = self.linear_speed
        twist.angular.z = angular_vel

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowerNode()
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
