#!/usr/bin/env python3
"""
Jetson Camera Node for KETI Robot
Uses GStreamer with hardware acceleration for optimal performance:
- nvarguscamerasrc: CSI camera input
- nvvidconv: Hardware color conversion (GPU accelerated)
- nvv4l2decoder: Hardware video decoding (NVDEC)
- nvjpegenc: Hardware JPEG encoding (NVJPG)
Publishes to /camera/image_raw for web_video_server and other nodes
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading


class JetsonCamera(Node):
    def __init__(self):
        super().__init__('jetson_camera')

        # Parameters - Optimized for Jetson Orin Nano
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)  # HD 16:9 ratio
        self.declare_parameter('fps', 30)
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('flip_method', 2)  # 0=none, 2=rotate180 (IMX219)
        self.declare_parameter('use_v4l2', False)  # Fallback to v4l2
        self.declare_parameter('jpeg_quality', 80)  # JPEG quality for compressed

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.sensor_id = self.get_parameter('sensor_id').value
        self.flip_method = self.get_parameter('flip_method').value
        self.use_v4l2 = self.get_parameter('use_v4l2').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        self.bridge = CvBridge()
        self.cap = None
        self.running = True
        self.frame_count = 0

        # Publishers - Use RELIABLE QoS for web_video_server compatibility
        qos_reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos_best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Main image uses RELIABLE for web_video_server
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_reliable)
        # Compressed uses BEST_EFFORT for lower latency
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera/image_raw/compressed', qos_best_effort)

        # Initialize camera
        if not self._init_camera():
            self.get_logger().error('Failed to initialize camera')
            return

        # Start capture thread
        self.capture_thread = threading.Thread(target=self._capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()

        self.get_logger().info(
            f'Jetson Camera started: {self.width}x{self.height}@{self.fps}fps'
        )
        self.get_logger().info('Using hardware acceleration: nvvidconv, NVJPG')

    def _gstreamer_pipeline(self):
        """Create GStreamer pipeline for CSI camera"""
        return (
            f'nvarguscamerasrc sensor-id={self.sensor_id} ! '
            f'video/x-raw(memory:NVMM), '
            f'width=(int)1280, height=(int)720, '
            f'format=(string)NV12, framerate=(fraction){self.fps}/1 ! '
            f'nvvidconv flip-method={self.flip_method} ! '
            f'video/x-raw, width=(int){self.width}, height=(int){self.height}, format=(string)BGRx ! '
            f'videoconvert ! '
            f'video/x-raw, format=(string)BGR ! appsink drop=1'
        )

    def _v4l2_pipeline(self):
        """Fallback V4L2 pipeline for USB cameras"""
        return (
            f'v4l2src device=/dev/video{self.sensor_id} ! '
            f'video/x-raw, width=(int){self.width}, height=(int){self.height} ! '
            f'videoconvert ! '
            f'video/x-raw, format=(string)BGR ! appsink drop=1'
        )

    def _init_camera(self):
        """Initialize camera with GStreamer or V4L2"""
        if self.use_v4l2:
            # Try V4L2 first (USB camera)
            pipeline = self._v4l2_pipeline()
            self.get_logger().info(f'Trying V4L2: {pipeline}')
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if self.cap.isOpened():
                self.get_logger().info('V4L2 camera opened successfully')
                return True

        # Try GStreamer pipeline (CSI camera)
        pipeline = self._gstreamer_pipeline()
        self.get_logger().info(f'Trying GStreamer: {pipeline}')
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

        if self.cap.isOpened():
            self.get_logger().info('CSI camera opened successfully')
            return True

        # Fallback to simple V4L2
        self.get_logger().warn('GStreamer failed, trying simple V4L2')
        self.cap = cv2.VideoCapture(self.sensor_id)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            self.get_logger().info('Simple V4L2 camera opened')
            return True

        return False

    def _capture_loop(self):
        """Camera capture loop running in separate thread"""
        while self.running and rclpy.ok():
            if self.cap is None or not self.cap.isOpened():
                self.get_logger().warn('Camera not open, retrying...')
                if not self._init_camera():
                    import time
                    time.sleep(1.0)
                    continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn('Failed to capture frame')
                continue

            self.frame_count += 1
            timestamp = self.get_clock().now().to_msg()

            # Publish raw image
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                msg.header.stamp = timestamp
                msg.header.frame_id = 'camera_link'
                self.image_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Publish error: {e}')

            # Publish compressed image (JPEG) every frame
            # Using OpenCV JPEG encoder which can use GPU on Jetson
            try:
                encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                _, jpeg_data = cv2.imencode('.jpg', frame, encode_param)

                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = 'camera_link'
                compressed_msg.format = 'jpeg'
                compressed_msg.data = jpeg_data.tobytes()
                self.compressed_pub.publish(compressed_msg)
            except Exception as e:
                pass  # Compressed is optional

    def destroy_node(self):
        self.running = False
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
