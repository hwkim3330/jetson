#!/usr/bin/env python3
"""
Auto Explore Node
Simple obstacle avoidance exploration for SLAM mapping
- Goes forward until obstacle detected
- Rotates to find clear path
- Continues exploring
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class ExploreNode(Node):
    def __init__(self):
        super().__init__('explore_node')

        # Parameters
        self.linear_speed = 0.15  # m/s
        self.angular_speed = 0.8  # rad/s
        self.obstacle_dist = 0.45  # meters
        self.side_obstacle_dist = 0.30  # meters

        # State
        self.exploring = False
        self.state = 'forward'  # forward, rotate_left, rotate_right
        self.rotate_time = 0.0
        self.scan_data = None

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.mode_sub = self.create_subscription(
            String, '/robot/mode', self.mode_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Control timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Explore node ready')

    def mode_callback(self, msg):
        if msg.data == 'explore':
            if not self.exploring:
                self.exploring = True
                self.state = 'forward'
                self.get_logger().info('Auto exploration started')
        else:
            if self.exploring:
                self.exploring = False
                self.stop()
                self.get_logger().info('Auto exploration stopped')

    def scan_callback(self, msg):
        self.scan_data = msg

    def get_range_at_angle(self, angle_deg):
        """Get range at specific angle (0=front, positive=left, negative=right)"""
        if not self.scan_data:
            return float('inf')

        # Convert angle to index
        angle_rad = math.radians(angle_deg)

        # LiDAR angles: angle_min to angle_max
        if angle_rad < self.scan_data.angle_min or angle_rad > self.scan_data.angle_max:
            return float('inf')

        idx = int((angle_rad - self.scan_data.angle_min) / self.scan_data.angle_increment)

        if 0 <= idx < len(self.scan_data.ranges):
            r = self.scan_data.ranges[idx]
            if self.scan_data.range_min < r < self.scan_data.range_max:
                return r
        return float('inf')

    def get_min_range(self, start_deg, end_deg):
        """Get minimum range in angle sector"""
        if not self.scan_data:
            return float('inf')

        min_range = float('inf')
        for angle in range(int(start_deg), int(end_deg), 3):
            r = self.get_range_at_angle(angle)
            if r < min_range:
                min_range = r
        return min_range

    def control_loop(self):
        if not self.exploring or not self.scan_data:
            return

        # Check sectors
        front = self.get_min_range(-25, 25)
        front_left = self.get_min_range(25, 60)
        front_right = self.get_min_range(-60, -25)
        left = self.get_min_range(60, 100)
        right = self.get_min_range(-100, -60)

        cmd = Twist()

        if self.state == 'forward':
            # Check for obstacles ahead
            if front < self.obstacle_dist:
                # Obstacle ahead - decide which way to turn
                if left > right:
                    self.state = 'rotate_left'
                    self.rotate_time = 0.0
                else:
                    self.state = 'rotate_right'
                    self.rotate_time = 0.0
                self.get_logger().info(f'Obstacle at {front:.2f}m, turning {"left" if self.state == "rotate_left" else "right"}')
            elif front_left < self.side_obstacle_dist:
                # Obstacle on front-left, veer right slightly
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = -0.3
            elif front_right < self.side_obstacle_dist:
                # Obstacle on front-right, veer left slightly
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = 0.3
            else:
                # Clear path - go forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0

        elif self.state == 'rotate_left':
            self.rotate_time += 0.1
            cmd.angular.z = self.angular_speed

            # Check if path is clear or timeout
            if front > self.obstacle_dist * 1.5 and self.rotate_time > 0.5:
                self.state = 'forward'
                self.get_logger().info('Path clear, moving forward')
            elif self.rotate_time > 4.0:
                # Stuck - try other direction
                self.state = 'rotate_right'
                self.rotate_time = 0.0

        elif self.state == 'rotate_right':
            self.rotate_time += 0.1
            cmd.angular.z = -self.angular_speed

            if front > self.obstacle_dist * 1.5 and self.rotate_time > 0.5:
                self.state = 'forward'
                self.get_logger().info('Path clear, moving forward')
            elif self.rotate_time > 4.0:
                self.state = 'rotate_left'
                self.rotate_time = 0.0

        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
