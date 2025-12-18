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
KETI Robot Patrol Node
- Square patrol: Move in a square pattern
- Triangle patrol: Move in a triangle pattern
- Circle patrol: Move in a circle pattern
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')
        self.get_logger().info('KETI Robot Patrol Node Started')

        # Parameters
        self.declare_parameter('pattern', 'square')  # square, triangle, circle
        self.declare_parameter('size', 1.0)  # meters
        self.declare_parameter('speed', 0.2)  # m/s
        self.declare_parameter('repeat', 1)  # number of repetitions

        self.pattern = self.get_parameter('pattern').value
        self.size = self.get_parameter('size').value
        self.speed = self.get_parameter('speed').value
        self.repeat = self.get_parameter('repeat').value

        # State
        self.current_yaw = 0.0
        self.is_running = False

        qos = QoSProfile(depth=10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.status_pub = self.create_publisher(String, '/patrol/status', qos)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos)
        self.cmd_sub = self.create_subscription(
            String, '/patrol/command', self.command_callback, qos)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.state = 'idle'  # idle, forward, turning
        self.target_yaw = 0.0
        self.distance_traveled = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.side_count = 0
        self.loop_count = 0

        self.get_logger().info(f'Pattern: {self.pattern}, Size: {self.size}m, Speed: {self.speed}m/s')

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.state == 'forward':
            dx = x - self.last_x
            dy = y - self.last_y
            self.distance_traveled += math.sqrt(dx*dx + dy*dy)
        self.last_x = x
        self.last_y = y

    def command_callback(self, msg):
        cmd = msg.data.lower()
        if cmd == 'start':
            self.start_patrol()
        elif cmd == 'stop':
            self.stop_patrol()
        elif cmd in ['square', 'triangle', 'circle']:
            self.pattern = cmd
            self.get_logger().info(f'Pattern changed to: {self.pattern}')

    def start_patrol(self):
        if not self.is_running:
            self.is_running = True
            self.state = 'forward'
            self.side_count = 0
            self.loop_count = 0
            self.distance_traveled = 0.0
            self.get_logger().info(f'Starting {self.pattern} patrol')
            self.publish_status(f'Starting {self.pattern} patrol')

    def stop_patrol(self):
        self.is_running = False
        self.state = 'idle'
        self.stop_robot()
        self.get_logger().info('Patrol stopped')
        self.publish_status('Patrol stopped')

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def timer_callback(self):
        if not self.is_running:
            return

        twist = Twist()

        if self.pattern == 'circle':
            # Circle: constant linear + angular velocity
            twist.linear.x = self.speed
            twist.angular.z = self.speed / (self.size / 2)  # v = r * omega
            self.cmd_vel_pub.publish(twist)
            return

        # Square or Triangle
        if self.pattern == 'square':
            sides = 4
            turn_angle = 90.0
        else:  # triangle
            sides = 3
            turn_angle = 120.0

        if self.state == 'forward':
            if self.distance_traveled >= self.size:
                # Reached target distance, start turning
                self.state = 'turning'
                self.target_yaw = self.normalize_angle(
                    self.current_yaw + math.radians(turn_angle))
                self.distance_traveled = 0.0
            else:
                twist.linear.x = self.speed
                self.cmd_vel_pub.publish(twist)

        elif self.state == 'turning':
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            if abs(yaw_error) < 0.05:
                # Turn complete
                self.side_count += 1
                self.publish_status(f'Side {self.side_count}/{sides} complete')

                if self.side_count >= sides:
                    self.loop_count += 1
                    self.side_count = 0
                    if self.loop_count >= self.repeat:
                        self.stop_patrol()
                        self.publish_status('Patrol complete!')
                        return
                    else:
                        self.publish_status(f'Loop {self.loop_count}/{self.repeat} complete')

                self.state = 'forward'
            else:
                twist.angular.z = 0.5 if yaw_error > 0 else -0.5
                self.cmd_vel_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
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
