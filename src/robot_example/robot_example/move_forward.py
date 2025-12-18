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

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MoveForward(Node):
    """Move forward node for KETI Robot."""

    def __init__(self):
        super().__init__('move_forward')

        self.declare_parameter('distance', 1.0)
        self.declare_parameter('linear_velocity', 0.2)

        self.target_distance = self.get_parameter('distance').value
        self.linear_velocity = self.get_parameter('linear_velocity').value

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        self.timer = self.create_timer(0.1, self.update_callback)

        self.start_x = 0.0
        self.start_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.init_odom = False
        self.done = False

        self.get_logger().info('Move forward node started')
        self.get_logger().info(f'Target distance: {self.target_distance}m')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if not self.init_odom:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.init_odom = True

    def update_callback(self):
        if not self.init_odom or self.done:
            return

        twist = Twist()

        distance = math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )

        if distance < self.target_distance:
            twist.linear.x = self.linear_velocity
            self.get_logger().info(f'Distance: {distance:.2f}m / {self.target_distance}m')
        else:
            twist.linear.x = 0.0
            self.done = True
            self.get_logger().info('Target distance reached!')

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveForward()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
