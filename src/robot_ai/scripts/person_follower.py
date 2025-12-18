#!/usr/bin/env python3
"""
Person Follower Node for KETI Robot
Follows detected person using proportional control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32
import math


class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # Parameters
        self.declare_parameter('linear_gain', 0.3)       # Speed gain
        self.declare_parameter('angular_gain', 0.8)      # Rotation gain
        self.declare_parameter('max_linear', 0.22)       # Max forward speed
        self.declare_parameter('max_angular', 1.0)       # Max rotation speed
        self.declare_parameter('target_distance', 1.0)   # Desired distance to person (m)
        self.declare_parameter('min_distance', 0.5)      # Stop if closer than this
        self.declare_parameter('lost_timeout', 2.0)      # Stop if person lost for this long
        self.declare_parameter('deadzone', 0.1)          # Ignore small errors

        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.target_distance = self.get_parameter('target_distance').value
        self.min_distance = self.get_parameter('min_distance').value
        self.lost_timeout = self.get_parameter('lost_timeout').value
        self.deadzone = self.get_parameter('deadzone').value

        # State
        self.enabled = False
        self.person_detected = False
        self.person_target = None
        self.person_distance = 2.0
        self.last_detection_time = self.get_clock().now()

        # Subscribers
        self.target_sub = self.create_subscription(
            Point, '/person_target', self.target_callback, 10)
        self.detected_sub = self.create_subscription(
            Bool, '/person_detected', self.detected_callback, 10)
        self.distance_sub = self.create_subscription(
            Float32, '/person_distance', self.distance_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/follow_enable', self.enable_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/follow_active', 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Person Follower initialized')
        self.get_logger().info('Publish True to /follow_enable to start following')

    def target_callback(self, msg):
        """Receive person target position (normalized -1 to 1)"""
        self.person_target = msg
        self.last_detection_time = self.get_clock().now()

    def detected_callback(self, msg):
        """Receive person detection status"""
        self.person_detected = msg.data

    def distance_callback(self, msg):
        """Receive estimated distance to person"""
        self.person_distance = msg.data

    def enable_callback(self, msg):
        """Enable/disable following"""
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info('Following ENABLED')
        else:
            self.get_logger().info('Following DISABLED')
            self.stop_robot()

    def control_loop(self):
        """Main control loop"""
        # Publish status
        status = Bool()
        status.data = self.enabled and self.person_detected
        self.status_pub.publish(status)

        if not self.enabled:
            return

        # Check if person is lost
        time_since_detection = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_since_detection > self.lost_timeout:
            self.get_logger().warn_throttle(self.get_clock(), 2000, 'Person lost, stopping')
            self.stop_robot()
            return

        if not self.person_detected or self.person_target is None:
            self.stop_robot()
            return

        # Calculate control commands
        cmd = Twist()

        # Angular control (turn towards person)
        # person_target.x is negative when person is to the left
        angular_error = -self.person_target.x  # Negate to turn towards person

        if abs(angular_error) > self.deadzone:
            cmd.angular.z = self.angular_gain * angular_error
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))
        else:
            cmd.angular.z = 0.0

        # Linear control (move towards/away from person)
        distance_error = self.person_distance - self.target_distance

        if self.person_distance < self.min_distance:
            # Too close - back up
            cmd.linear.x = -0.1
            self.get_logger().warn_throttle(self.get_clock(), 1000, 'Too close, backing up')
        elif abs(distance_error) > 0.2:
            # Move towards target distance
            cmd.linear.x = self.linear_gain * distance_error
            cmd.linear.x = max(-self.max_linear * 0.5, min(self.max_linear, cmd.linear.x))

            # Slow down when turning
            turn_factor = 1.0 - min(abs(angular_error), 0.5) * 0.6
            cmd.linear.x *= turn_factor
        else:
            cmd.linear.x = 0.0

        # Publish command
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        """Send stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
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
