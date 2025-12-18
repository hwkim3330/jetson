#!/usr/bin/env python3
"""
Body Tracker Node for KETI Robot
Uses color-based tracking as lightweight alternative
Can track person by their clothing color
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, Float32, ColorRGBA
from cv_bridge import CvBridge
import cv2
import numpy as np


class BodyTracker(Node):
    def __init__(self):
        super().__init__('body_tracker')

        # Parameters
        self.declare_parameter('tracking_rate', 15.0)
        self.declare_parameter('min_area', 5000)        # Minimum blob area
        self.declare_parameter('max_area', 200000)      # Maximum blob area
        self.declare_parameter('linear_gain', 0.25)
        self.declare_parameter('angular_gain', 0.7)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 0.8)
        self.declare_parameter('target_area', 30000)    # Target blob size

        self.tracking_rate = self.get_parameter('tracking_rate').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.target_area = self.get_parameter('target_area').value

        # State
        self.bridge = CvBridge()
        self.last_frame = None
        self.enabled = False
        self.tracking = False

        # Target color in HSV (default: red)
        self.target_hsv_low = np.array([0, 100, 100])
        self.target_hsv_high = np.array([10, 255, 255])
        self.target_hsv_low2 = np.array([160, 100, 100])  # Red wraps around
        self.target_hsv_high2 = np.array([180, 255, 255])

        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos)
        self.compressed_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.compressed_callback, qos)
        self.enable_sub = self.create_subscription(
            Bool, '/tracker_enable', self.enable_callback, 10)
        self.color_sub = self.create_subscription(
            ColorRGBA, '/tracker_color', self.color_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(Point, '/tracker_target', 10)
        self.status_pub = self.create_publisher(Bool, '/tracker_active', 10)
        self.debug_pub = self.create_publisher(Image, '/tracker/debug', 10)

        # Timer
        period = 1.0 / self.tracking_rate
        self.timer = self.create_timer(period, self.track_callback)

        self.get_logger().info('Body Tracker initialized')
        self.get_logger().info('Publish True to /tracker_enable to start')
        self.get_logger().info('Default tracking: RED color')

    def image_callback(self, msg):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            pass

    def compressed_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.last_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            pass

    def enable_callback(self, msg):
        self.enabled = msg.data
        if self.enabled:
            self.get_logger().info('Tracking ENABLED')
        else:
            self.get_logger().info('Tracking DISABLED')
            self.stop_robot()

    def color_callback(self, msg):
        """Set target color (RGB 0-1 values)"""
        # Convert RGB to HSV
        r = int(msg.r * 255)
        g = int(msg.g * 255)
        b = int(msg.b * 255)

        color_bgr = np.uint8([[[b, g, r]]])
        color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]

        h, s, v = color_hsv
        margin = 15

        self.target_hsv_low = np.array([max(0, h - margin), 80, 80])
        self.target_hsv_high = np.array([min(180, h + margin), 255, 255])

        # Handle red color wrap-around
        if h < margin:
            self.target_hsv_low2 = np.array([180 - margin + h, 80, 80])
            self.target_hsv_high2 = np.array([180, 255, 255])
        elif h > 180 - margin:
            self.target_hsv_low2 = np.array([0, 80, 80])
            self.target_hsv_high2 = np.array([h + margin - 180, 255, 255])
        else:
            self.target_hsv_low2 = None
            self.target_hsv_high2 = None

        self.get_logger().info(f'Target color set: HSV={h},{s},{v}')

    def track_callback(self):
        """Main tracking loop"""
        status = Bool()
        status.data = self.tracking
        self.status_pub.publish(status)

        if self.last_frame is None:
            return

        frame = self.last_frame.copy()
        h, w = frame.shape[:2]

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask for target color
        mask1 = cv2.inRange(hsv, self.target_hsv_low, self.target_hsv_high)
        if self.target_hsv_low2 is not None:
            mask2 = cv2.inRange(hsv, self.target_hsv_low2, self.target_hsv_high2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.tracking = False
        target_point = Point()
        cmd = Twist()

        if contours:
            # Find largest contour within size limits
            valid_contours = []
            for c in contours:
                area = cv2.contourArea(c)
                if self.min_area < area < self.max_area:
                    valid_contours.append((area, c))

            if valid_contours:
                # Get largest
                valid_contours.sort(key=lambda x: x[0], reverse=True)
                area, largest = valid_contours[0]

                # Get bounding box and center
                x, y, bw, bh = cv2.boundingRect(largest)
                center_x = x + bw // 2
                center_y = y + bh // 2

                # Normalize position
                target_point.x = (center_x - w/2) / (w/2)
                target_point.y = (center_y - h/2) / (h/2)
                target_point.z = float(area)

                self.tracking = True
                self.target_pub.publish(target_point)

                # Draw on debug image
                cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, f'Area: {area}', (x, y-10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Control robot if enabled
                if self.enabled:
                    # Angular control
                    angular_error = -target_point.x
                    if abs(angular_error) > 0.1:
                        cmd.angular.z = self.angular_gain * angular_error
                        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

                    # Linear control based on blob size
                    area_error = (self.target_area - area) / self.target_area
                    if abs(area_error) > 0.2:
                        cmd.linear.x = self.linear_gain * area_error
                        cmd.linear.x = max(-self.max_linear * 0.5, min(self.max_linear, cmd.linear.x))

                        # Slow down when turning
                        turn_factor = 1.0 - min(abs(angular_error), 0.5) * 0.5
                        cmd.linear.x *= turn_factor

        # Send command
        if self.enabled:
            if self.tracking:
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()

        # Publish debug image
        # Draw mask overlay
        debug = frame.copy()
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_color[:, :, 0] = 0  # Blue = 0
        mask_color[:, :, 2] = 0  # Red = 0 (green mask)
        debug = cv2.addWeighted(debug, 0.7, mask_color, 0.3, 0)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except:
            pass

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BodyTracker()
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
