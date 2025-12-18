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


class Turn(Node):
    """Turn node for KETI Robot."""

    def __init__(self):
        super().__init__('turn')

        self.declare_parameter('angle', 90.0)  # degrees
        self.declare_parameter('angular_velocity', 0.5)

        angle_deg = self.get_parameter('angle').value
        self.target_angle = math.radians(angle_deg)
        self.angular_velocity = self.get_parameter('angular_velocity').value

        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        self.timer = self.create_timer(0.1, self.update_callback)

        self.start_yaw = 0.0
        self.current_yaw = 0.0
        self.init_odom = False
        self.done = False

        self.get_logger().info('Turn node started')
        self.get_logger().info(f'Target angle: {angle_deg} degrees')

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        if not self.init_odom:
            self.start_yaw = self.current_yaw
            self.init_odom = True

    def update_callback(self):
        if not self.init_odom or self.done:
            return

        twist = Twist()

        yaw_diff = self.current_yaw - self.start_yaw

        # Normalize angle to [-pi, pi]
        while yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        while yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi

        if abs(yaw_diff) < abs(self.target_angle):
            if self.target_angle > 0:
                twist.angular.z = self.angular_velocity
            else:
                twist.angular.z = -self.angular_velocity
            self.get_logger().info(f'Turned: {math.degrees(yaw_diff):.1f} / {math.degrees(self.target_angle):.1f} deg')
        else:
            twist.angular.z = 0.0
            self.done = True
            self.get_logger().info('Target angle reached!')

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Turn()

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
