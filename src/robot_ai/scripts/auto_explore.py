#!/usr/bin/env python3
"""
Auto Explore - Simple frontier exploration for SLAM
Finds open space direction and moves toward it, creating a map automatically.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math


class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')

        # Parameters
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.8  # rad/s
        self.min_distance = 0.4  # minimum distance to obstacle (m)
        self.scan_sectors = 8  # divide scan into sectors

        # State
        self.current_direction = 0  # current best direction (radians)
        self.obstacle_detected = False
        self.last_scan = None

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Control timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Auto Explore started - exploring environment')

    def scan_callback(self, msg):
        self.last_scan = msg

    def find_best_direction(self, scan):
        """Find the direction with most open space"""
        if scan is None:
            return 0, False

        ranges = np.array(scan.ranges)
        n = len(ranges)

        # Replace inf/nan with max range
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        ranges = np.clip(ranges, scan.range_min, scan.range_max)

        # Divide into sectors and find average distance for each
        sector_size = n // self.scan_sectors
        sector_distances = []
        sector_angles = []

        for i in range(self.scan_sectors):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size if i < self.scan_sectors - 1 else n
            sector_ranges = ranges[start_idx:end_idx]
            avg_dist = np.mean(sector_ranges)
            min_dist = np.min(sector_ranges)

            # Angle of sector center
            center_idx = (start_idx + end_idx) // 2
            angle = scan.angle_min + center_idx * scan.angle_increment

            sector_distances.append(avg_dist)
            sector_angles.append(angle)

        # Find sector with maximum average distance
        best_sector = np.argmax(sector_distances)
        best_angle = sector_angles[best_sector]
        best_distance = sector_distances[best_sector]

        # Check if front is blocked
        front_idx = n // 2
        front_range = 30  # check +/- 30 indices around front
        front_distances = ranges[max(0, front_idx-front_range):min(n, front_idx+front_range)]
        front_blocked = np.min(front_distances) < self.min_distance

        return best_angle, front_blocked, best_distance

    def control_loop(self):
        """Main control loop"""
        twist = Twist()

        if self.last_scan is None:
            self.cmd_pub.publish(twist)
            return

        best_angle, front_blocked, best_distance = self.find_best_direction(self.last_scan)

        if front_blocked:
            # Turn toward best direction
            if abs(best_angle) > 0.3:
                twist.angular.z = self.angular_speed if best_angle > 0 else -self.angular_speed
            else:
                # Small angle, just turn a bit
                twist.angular.z = self.angular_speed * 0.5 * np.sign(best_angle)
            twist.linear.x = 0.0
        else:
            # Move forward, slight correction toward best direction
            twist.linear.x = self.linear_speed
            # Gentle steering toward open space
            twist.angular.z = best_angle * 0.3  # proportional steering
            twist.angular.z = np.clip(twist.angular.z, -self.angular_speed * 0.5, self.angular_speed * 0.5)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoExplore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
