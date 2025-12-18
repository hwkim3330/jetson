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
KETI Robot Absolute Move Node
- Moves robot to absolute position in odometry frame
- Supports both topic-based and interactive input modes
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json


class AbsoluteMoveNode(Node):

    def __init__(self):
        super().__init__('absolute_move')
        self.get_logger().info('KETI Robot Absolute Move Started')

        # Parameters
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('heading_tolerance', 0.02)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value

        # State
        self.goal_position = Point()
        self.goal_heading = 0.0
        self.current_position = Point()
        self.current_heading = 0.0
        self.is_moving = False
        self.state = 'idle'  # idle, moving, rotating

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(String, '/move/status', qos)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)
        self.goal_sub = self.create_subscription(
            String, '/move/goal', self.goal_callback, qos)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info('Listening on /move/goal for JSON: {"x": 1.0, "y": 0.5, "heading": 90}')

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny, cosy)

    def goal_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.goal_position.x = float(data.get('x', 0.0))
            self.goal_position.y = float(data.get('y', 0.0))
            heading_deg = float(data.get('heading', 0.0))
            self.goal_heading = math.radians(heading_deg)
            self.goal_heading = self.normalize_angle(self.goal_heading)

            self.is_moving = True
            self.state = 'moving'
            self.get_logger().info(
                f'Goal set: x={self.goal_position.x:.2f}, y={self.goal_position.y:.2f}, '
                f'heading={heading_deg:.1f}°')
            self.publish_status('Moving to goal')
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f'Invalid goal format: {e}')

    def timer_callback(self):
        if not self.is_moving:
            return

        twist = Twist()

        if self.state == 'moving':
            dx = self.goal_position.x - self.current_position.x
            dy = self.goal_position.y - self.current_position.y
            distance = math.sqrt(dx*dx + dy*dy)

            if distance > self.position_tolerance:
                # Move towards goal
                goal_direction = math.atan2(dy, dx)
                angle_error = self.normalize_angle(goal_direction - self.current_heading)

                # Proportional control
                twist.angular.z = max(min(angle_error * 2.0, self.angular_speed), -self.angular_speed)
                twist.linear.x = min(self.linear_speed * distance, self.linear_speed)

                # Reduce speed if not facing goal
                if abs(angle_error) > 0.5:
                    twist.linear.x *= 0.3

            else:
                # Position reached, rotate to final heading
                self.state = 'rotating'

        elif self.state == 'rotating':
            heading_error = self.normalize_angle(self.goal_heading - self.current_heading)

            if abs(heading_error) > self.heading_tolerance:
                speed = max(min(abs(heading_error) * 1.5, self.angular_speed), 0.1)
                twist.angular.z = speed if heading_error > 0 else -speed
            else:
                # Goal reached
                self.is_moving = False
                self.state = 'idle'
                self.get_logger().info('Goal reached!')
                self.publish_status('Goal reached')

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
    node = AbsoluteMoveNode()
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
