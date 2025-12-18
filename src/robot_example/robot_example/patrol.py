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

from tf_transformations import euler_from_quaternion


class Patrol(Node):
    """Patrol node for KETI Robot - moves in a square pattern."""

    def __init__(self):
        super().__init__('patrol')

        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.5)

        self.side_length = self.get_parameter('side_length').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        self.timer = self.create_timer(0.1, self.update_callback)

        # State machine
        self.state = 'FORWARD'
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.init_odom = False
        self.turn_count = 0

        self.get_logger().info('Patrol node started')
        self.get_logger().info(f'Square patrol with {self.side_length}m sides')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        if not self.init_odom:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_yaw = self.current_yaw
            self.init_odom = True

    def update_callback(self):
        if not self.init_odom:
            return

        twist = Twist()

        if self.state == 'FORWARD':
            distance = math.sqrt(
                (self.current_x - self.start_x) ** 2 +
                (self.current_y - self.start_y) ** 2
            )

            if distance < self.side_length:
                twist.linear.x = self.linear_velocity
            else:
                self.state = 'TURN'
                self.start_yaw = self.current_yaw
                self.get_logger().info('Turning...')

        elif self.state == 'TURN':
            yaw_diff = abs(self.current_yaw - self.start_yaw)
            if yaw_diff > math.pi:
                yaw_diff = 2 * math.pi - yaw_diff

            if yaw_diff < math.pi / 2:
                twist.angular.z = self.angular_velocity
            else:
                self.state = 'FORWARD'
                self.start_x = self.current_x
                self.start_y = self.current_y
                self.turn_count += 1
                self.get_logger().info(f'Moving forward (turn {self.turn_count}/4)')

                if self.turn_count >= 4:
                    self.turn_count = 0
                    self.get_logger().info('Completed one patrol cycle')

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Patrol()

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
