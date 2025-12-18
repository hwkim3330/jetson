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
# Based on TurtleBot3 autorace by ROBOTIS CO., LTD.

"""
KETI Robot Lane Detection Node
- Detects white and yellow lane lines using HSV color filtering
- Publishes lane center offset for lane following
- Optimized for Jetson Orin Nano
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import Twist


class LaneDetectorNode(Node):

    def __init__(self):
        super().__init__('lane_detector')
        self.get_logger().info('KETI Robot Lane Detector Started')

        # Parameters - White lane HSV
        self.declare_parameter('white_h_low', 0)
        self.declare_parameter('white_h_high', 179)
        self.declare_parameter('white_s_low', 0)
        self.declare_parameter('white_s_high', 70)
        self.declare_parameter('white_v_low', 180)
        self.declare_parameter('white_v_high', 255)

        # Parameters - Yellow lane HSV
        self.declare_parameter('yellow_h_low', 15)
        self.declare_parameter('yellow_h_high', 35)
        self.declare_parameter('yellow_s_low', 80)
        self.declare_parameter('yellow_s_high', 255)
        self.declare_parameter('yellow_v_low', 80)
        self.declare_parameter('yellow_v_high', 255)

        # Control parameters
        self.declare_parameter('lane_width', 280)  # expected lane width in pixels
        self.declare_parameter('center_offset', 0)  # offset from center (positive = right)
        self.declare_parameter('enabled', False)

        self.load_parameters()

        # State
        self.bridge = CvBridge()
        self.enabled = self.get_parameter('enabled').value
        self.frame_count = 0
        self.last_center = 320  # default center

        # Publishers
        self.center_pub = self.create_publisher(Float32, '/lane/center', 10)
        self.offset_pub = self.create_publisher(Float32, '/lane/offset', 10)
        self.state_pub = self.create_publisher(UInt8, '/lane/state', 10)
        self.debug_pub = self.create_publisher(CompressedImage, '/lane/debug/compressed', 1)

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.image_callback, qos_profile_sensor_data)

        self.get_logger().info('Subscribed to /camera/image_raw/compressed')

    def load_parameters(self):
        self.white_low = np.array([
            self.get_parameter('white_h_low').value,
            self.get_parameter('white_s_low').value,
            self.get_parameter('white_v_low').value
        ])
        self.white_high = np.array([
            self.get_parameter('white_h_high').value,
            self.get_parameter('white_s_high').value,
            self.get_parameter('white_v_high').value
        ])
        self.yellow_low = np.array([
            self.get_parameter('yellow_h_low').value,
            self.get_parameter('yellow_s_low').value,
            self.get_parameter('yellow_v_low').value
        ])
        self.yellow_high = np.array([
            self.get_parameter('yellow_h_high').value,
            self.get_parameter('yellow_s_high').value,
            self.get_parameter('yellow_v_high').value
        ])
        self.lane_width = self.get_parameter('lane_width').value
        self.center_offset = self.get_parameter('center_offset').value

    def image_callback(self, msg):
        # Process every 3rd frame to reduce CPU load
        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return

        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                return

            height, width = frame.shape[:2]

            # Only analyze bottom half of image (where road is)
            roi = frame[height//2:, :]
            roi_height = roi.shape[0]

            # Convert to HSV
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Detect white and yellow lanes
            white_mask = cv2.inRange(hsv, self.white_low, self.white_high)
            yellow_mask = cv2.inRange(hsv, self.yellow_low, self.yellow_high)

            # Find lane positions
            white_center = self.find_lane_center(white_mask)
            yellow_center = self.find_lane_center(yellow_mask)

            # Determine lane state and center
            # State: 0=none, 1=yellow only, 2=both, 3=white only
            state = UInt8()
            lane_center = None

            white_valid = white_center is not None
            yellow_valid = yellow_center is not None

            if white_valid and yellow_valid:
                lane_center = (white_center + yellow_center) // 2
                state.data = 2
            elif yellow_valid:
                lane_center = yellow_center + self.lane_width // 2
                state.data = 1
            elif white_valid:
                lane_center = white_center - self.lane_width // 2
                state.data = 3
            else:
                lane_center = self.last_center
                state.data = 0

            self.last_center = lane_center
            self.state_pub.publish(state)

            # Calculate offset from image center
            image_center = width // 2 + self.center_offset
            offset = lane_center - image_center

            # Publish results
            center_msg = Float32()
            center_msg.data = float(lane_center)
            self.center_pub.publish(center_msg)

            offset_msg = Float32()
            offset_msg.data = float(offset)
            self.offset_pub.publish(offset_msg)

            # Debug visualization
            debug_frame = roi.copy()
            cv2.line(debug_frame, (lane_center, 0), (lane_center, roi_height), (0, 255, 255), 2)
            cv2.line(debug_frame, (image_center, 0), (image_center, roi_height), (255, 0, 0), 1)

            if yellow_valid:
                cv2.line(debug_frame, (yellow_center, 0), (yellow_center, roi_height), (0, 255, 0), 2)
            if white_valid:
                cv2.line(debug_frame, (white_center, 0), (white_center, roi_height), (255, 255, 255), 2)

            # Add text overlay
            cv2.putText(debug_frame, f'Offset: {offset:.0f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(debug_frame, f'State: {state.data}', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Publish debug image
            _, jpeg = cv2.imencode('.jpg', debug_frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            debug_msg = CompressedImage()
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            debug_msg.format = 'jpeg'
            debug_msg.data = jpeg.tobytes()
            self.debug_pub.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def find_lane_center(self, mask):
        """Find the center x position of a lane from binary mask"""
        # Sum columns in bottom portion
        bottom_half = mask[mask.shape[0]//2:, :]
        histogram = np.sum(bottom_half, axis=0)

        # Find peak
        if np.max(histogram) > 1000:  # minimum threshold
            return int(np.argmax(histogram))
        return None


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
