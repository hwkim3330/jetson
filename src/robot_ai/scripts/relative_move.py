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
KETI Robot Relative Move Node
- Moves robot relative to current position
- x: forward/backward, y: left/right, theta: rotation
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json


class RelativeMoveNode(Node):

    def __init__(self):
        super().__init__('relative_move')
        self.get_logger().info('KETI Robot Relative Move Started')

        # Parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.3)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_theta = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_theta = 0.0
        self.is_moving = False
        self.step = 0  # 0: idle, 1: turn to goal, 2: go straight, 3: final turn
        self.odom_ready = False

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(String, '/relative_move/status', qos)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)
        self.goal_sub = self.create_subscription(
            String, '/relative_move/goal', self.goal_callback, qos)

        # Timer
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info('Listening on /relative_move/goal for JSON: {"x": 1.0, "y": 0.5, "theta": 90}')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny, cosy)
        self.odom_ready = True

    def goal_callback(self, msg):
        if not self.odom_ready:
            self.get_logger().warn('Waiting for odometry...')
            return

        try:
            data = json.loads(msg.data)
            input_x = float(data.get('x', 0.0))
            input_y = float(data.get('y', 0.0))
            input_theta = float(data.get('theta', 0.0))

            # Store start position
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.start_theta = self.current_theta

            # Convert relative to global coordinates
            cos_t = math.cos(self.current_theta)
            sin_t = math.sin(self.current_theta)
            global_x = cos_t * input_x - sin_t * input_y
            global_y = sin_t * input_x + cos_t * input_y

            self.goal_x = self.current_x + global_x
            self.goal_y = self.current_y + global_y
            self.goal_theta = self.normalize_angle(self.current_theta + math.radians(input_theta))

            self.is_moving = True
            self.step = 1

            self.get_logger().info(
                f'Relative move: x={input_x:.2f}, y={input_y:.2f}, theta={input_theta:.1f}°')
            self.publish_status('Moving')

        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f'Invalid goal format: {e}')

    def timer_callback(self):
        if not self.is_moving or not self.odom_ready:
            return

        twist = Twist()

        if self.step == 1:
            # Step 1: Turn towards goal position
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance < 0.02:
                # Skip straight movement if distance is very small
                self.step = 3
                return

            path_theta = math.atan2(dy, dx)
            angle_error = self.normalize_angle(path_theta - self.current_theta)

            if abs(angle_error) > 0.02:
                twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            else:
                self.step = 2

        elif self.step == 2:
            # Step 2: Go straight to goal
            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > 0.02:
                twist.linear.x = self.linear_speed
            else:
                self.step = 3

        elif self.step == 3:
            # Step 3: Final rotation
            angle_error = self.normalize_angle(self.goal_theta - self.current_theta)

            if abs(angle_error) > 0.02:
                twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            else:
                self.step = 0
                self.is_moving = False
                self.get_logger().info('Relative move complete!')
                self.publish_status('Complete')

        self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RelativeMoveNode()
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
