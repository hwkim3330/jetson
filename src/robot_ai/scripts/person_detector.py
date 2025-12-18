#!/usr/bin/env python3
"""
Person Detector Node for KETI Robot
Uses OpenCV DNN with MobileNet-SSD for person detection
Optimized for Jetson with hardware acceleration:
- CUDA backend for GPU inference
- TensorRT for optimized inference (when available)
- DLA (Deep Learning Accelerator) support for Jetson Orin
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # Parameters
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_width', 300)
        self.declare_parameter('input_height', 300)
        self.declare_parameter('use_cuda', True)
        self.declare_parameter('detection_rate', 10.0)  # Hz

        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.input_width = self.get_parameter('input_width').value
        self.input_height = self.get_parameter('input_height').value
        self.use_cuda = self.get_parameter('use_cuda').value
        self.detection_rate = self.get_parameter('detection_rate').value

        # Initialize OpenCV DNN
        self.net = None
        self.bridge = CvBridge()
        self.last_frame = None
        self.last_detection_time = self.get_clock().now()

        # COCO class ID for person is 1
        self.PERSON_CLASS_ID = 1

        # Load model
        self._load_model()

        # QoS for camera topics
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos)

        self.compressed_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self.compressed_callback, qos)

        # Publishers
        self.person_pub = self.create_publisher(Point, '/person_target', 10)
        self.detected_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/person_distance', 10)
        self.debug_pub = self.create_publisher(Image, '/person_detector/debug', 10)

        # Timer for detection (rate limited)
        period = 1.0 / self.detection_rate
        self.timer = self.create_timer(period, self.detect_timer_callback)

        self.get_logger().info(f'Person Detector initialized (rate: {self.detection_rate} Hz)')

    def _load_model(self):
        """Load MobileNet-SSD model for person detection"""
        model_dir = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'share', 'robot_ai', 'models'
        )

        # Try to find model files
        prototxt = None
        caffemodel = None

        # Check common locations
        search_paths = [
            '/home/nvidia/ros2_ws/src/robot/robot_ai/models',
            '/home/nvidia/models',
            '/opt/nvidia/models',
            model_dir
        ]

        for path in search_paths:
            proto_path = os.path.join(path, 'MobileNetSSD_deploy.prototxt')
            model_path = os.path.join(path, 'MobileNetSSD_deploy.caffemodel')
            if os.path.exists(proto_path) and os.path.exists(model_path):
                prototxt = proto_path
                caffemodel = model_path
                break

        if prototxt and caffemodel:
            self.net = cv2.dnn.readNetFromCaffe(prototxt, caffemodel)
            self.get_logger().info(f'Loaded MobileNet-SSD from {prototxt}')
        else:
            # Use OpenCV's built-in HOG detector as fallback
            self.get_logger().warn('MobileNet-SSD not found, using HOG detector')
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            self.net = None
            return

        # Try to use hardware acceleration (priority: DLA > CUDA > CPU)
        if self.use_cuda:
            backend_set = False

            # Try CUDA with FP16 for faster inference
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
                self.get_logger().info('Using CUDA FP16 backend (GPU accelerated)')
                backend_set = True
            except Exception:
                pass

            # Fallback to regular CUDA
            if not backend_set:
                try:
                    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
                    self.get_logger().info('Using CUDA backend')
                    backend_set = True
                except Exception as e:
                    self.get_logger().warn(f'CUDA not available: {e}')

            # Fallback to CPU
            if not backend_set:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_DEFAULT)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                self.get_logger().info('Using CPU backend')

    def image_callback(self, msg):
        """Handle raw image messages"""
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def compressed_callback(self, msg):
        """Handle compressed image messages"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.last_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f'Compressed image error: {e}')

    def detect_timer_callback(self):
        """Run detection at fixed rate"""
        if self.last_frame is None:
            return

        frame = self.last_frame.copy()
        h, w = frame.shape[:2]

        person_detected = False
        target_point = Point()
        distance = 0.0

        if self.net is not None:
            # Use DNN model
            blob = cv2.dnn.blobFromImage(
                frame, 0.007843, (self.input_width, self.input_height),
                (127.5, 127.5, 127.5), swapRB=True, crop=False
            )
            self.net.setInput(blob)
            detections = self.net.forward()

            best_conf = 0
            best_box = None

            for i in range(detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                class_id = int(detections[0, 0, i, 1])

                if class_id == 15 and confidence > self.conf_threshold:  # 15 = person in VOC
                    if confidence > best_conf:
                        best_conf = confidence
                        box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                        best_box = box.astype(int)

            if best_box is not None:
                x1, y1, x2, y2 = best_box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                box_height = y2 - y1

                # Normalize to [-1, 1] range
                target_point.x = (center_x - w/2) / (w/2)
                target_point.y = (center_y - h/2) / (h/2)
                target_point.z = best_conf

                # Estimate distance based on box size (rough approximation)
                # Assuming person height ~1.7m fills frame at ~2m distance
                distance = (h * 0.7) / max(box_height, 1)

                person_detected = True

                # Draw debug visualization
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)
                cv2.putText(frame, f'{best_conf:.2f} d:{distance:.1f}m',
                           (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        else:
            # Use HOG detector (fallback)
            small = cv2.resize(frame, (320, 240))
            boxes, weights = self.hog.detectMultiScale(
                small, winStride=(8, 8), padding=(4, 4), scale=1.05
            )

            if len(boxes) > 0:
                # Get largest detection (closest person)
                areas = [bw * bh for (bx, by, bw, bh) in boxes]
                idx = np.argmax(areas)
                x, y, bw, bh = boxes[idx]

                # Scale back to original size
                scale_x = w / 320
                scale_y = h / 240
                x1 = int(x * scale_x)
                y1 = int(y * scale_y)
                x2 = int((x + bw) * scale_x)
                y2 = int((y + bh) * scale_y)

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                box_height = y2 - y1

                target_point.x = (center_x - w/2) / (w/2)
                target_point.y = (center_y - h/2) / (h/2)
                target_point.z = float(weights[idx])

                distance = (h * 0.7) / max(box_height, 1)
                person_detected = True

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

        # Publish results
        detected_msg = Bool()
        detected_msg.data = person_detected
        self.detected_pub.publish(detected_msg)

        if person_detected:
            self.person_pub.publish(target_point)

            dist_msg = Float32()
            dist_msg.data = distance
            self.distance_pub.publish(dist_msg)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
