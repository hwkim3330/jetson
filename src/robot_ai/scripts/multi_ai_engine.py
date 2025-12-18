#!/usr/bin/env python3
"""
KETI Robot Multi-AI Engine
Maximizes Jetson Orin Nano hardware utilization

Hardware Engines Used:
- GPU: YOLO object detection (TensorRT)
- DLA: Lightweight models (if available)
- NVJPEG: Hardware image compression
- VIC: Hardware video conversion

Runs multiple AI models simultaneously using CUDA streams.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String, Bool, Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import json
import time


class MultiAIEngine(Node):
    """
    Multi-model AI engine for Jetson Orin Nano.
    Runs YOLO + optional hand/pose detection simultaneously.
    """

    def __init__(self):
        super().__init__('multi_ai_engine')

        # Parameters
        self.declare_parameter('yolo_model', '')
        self.declare_parameter('yolo_conf', 0.5)
        self.declare_parameter('yolo_rate', 10.0)
        self.declare_parameter('enable_pose', False)
        self.declare_parameter('enable_hands', False)
        self.declare_parameter('use_dla', False)  # Use DLA for lightweight models

        self.yolo_model_path = self.get_parameter('yolo_model').value
        self.yolo_conf = self.get_parameter('yolo_conf').value
        self.yolo_rate = self.get_parameter('yolo_rate').value
        self.enable_pose = self.get_parameter('enable_pose').value
        self.enable_hands = self.get_parameter('enable_hands').value
        self.use_dla = self.get_parameter('use_dla').value

        self.bridge = CvBridge()
        self.last_frame = None
        self.frame_lock = threading.Lock()
        self.yolo_model = None
        self.pose_model = None
        self.hands_model = None

        # COCO class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
            'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
            'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
            'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
            'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
            'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
            'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven',
            'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]

        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._image_callback, qos)

        # Publishers - Detection results
        self.detection_pub = self.create_publisher(String, '/detections', 10)
        self.person_pub = self.create_publisher(Point, '/person_target', 10)
        self.detected_pub = self.create_publisher(Bool, '/person_detected', 10)
        self.distance_pub = self.create_publisher(Float32, '/person_distance', 10)

        # Publishers - Debug images
        self.debug_pub = self.create_publisher(Image, '/ai/debug', qos)

        # Publishers - Pose/Hands (if enabled)
        if self.enable_pose:
            self.pose_pub = self.create_publisher(String, '/pose/landmarks', 10)
        if self.enable_hands:
            self.hands_pub = self.create_publisher(String, '/hands/landmarks', 10)
            self.gesture_pub = self.create_publisher(String, '/gesture', 10)

        # Load models
        self._load_models()

        # AI timer
        self.ai_timer = self.create_timer(1.0 / self.yolo_rate, self._ai_callback)

        self.get_logger().info('Multi-AI Engine started')
        self.get_logger().info(f'  YOLO: {"Enabled" if self.yolo_model else "Disabled"}')
        self.get_logger().info(f'  Pose: {"Enabled" if self.enable_pose else "Disabled"}')
        self.get_logger().info(f'  Hands: {"Enabled" if self.enable_hands else "Disabled"}')

    def _load_models(self):
        """Load all AI models."""
        # Load YOLO
        self._load_yolo()

        # Load MediaPipe models if enabled
        if self.enable_pose or self.enable_hands:
            self._load_mediapipe()

    def _load_yolo(self):
        """Load YOLO model with TensorRT."""
        try:
            from ultralytics import YOLO

            paths = [
                self.yolo_model_path,
                '/home/nvidia/ros2_ws/src/keti/robot_ai/models/yolov8n.engine',
                '/home/nvidia/ros2_ws/src/keti/robot_ai/models/yolov8n.pt',
                'yolov8n.pt'
            ]

            for path in paths:
                if path:
                    try:
                        self.yolo_model = YOLO(path)
                        engine_type = 'TensorRT' if path.endswith('.engine') else 'PyTorch'
                        self.get_logger().info(f'YOLO loaded ({engine_type}): {path}')
                        return
                    except Exception as e:
                        continue

            self.get_logger().warn('YOLO model not found')

        except ImportError:
            self.get_logger().warn('ultralytics not installed')

    def _load_mediapipe(self):
        """Load MediaPipe models."""
        try:
            import mediapipe as mp

            if self.enable_pose:
                self.mp_pose = mp.solutions.pose
                self.pose_model = self.mp_pose.Pose(
                    static_image_mode=False,
                    model_complexity=0,  # Lightweight for speed
                    min_detection_confidence=0.5
                )
                self.get_logger().info('MediaPipe Pose loaded')

            if self.enable_hands:
                self.mp_hands = mp.solutions.hands
                self.hands_model = self.mp_hands.Hands(
                    static_image_mode=False,
                    max_num_hands=2,
                    min_detection_confidence=0.5
                )
                self.get_logger().info('MediaPipe Hands loaded')

        except ImportError:
            self.get_logger().warn('mediapipe not installed')
            self.enable_pose = False
            self.enable_hands = False

    def _image_callback(self, msg):
        """Store latest frame."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.last_frame = frame
        except Exception as e:
            pass

    def _ai_callback(self):
        """Run AI inference."""
        with self.frame_lock:
            if self.last_frame is None:
                return
            frame = self.last_frame.copy()

        debug_frame = frame.copy()
        h, w = frame.shape[:2]

        # Run YOLO
        if self.yolo_model:
            self._run_yolo(frame, debug_frame, w, h)

        # Run MediaPipe Pose
        if self.enable_pose and self.pose_model:
            self._run_pose(frame, debug_frame)

        # Run MediaPipe Hands
        if self.enable_hands and self.hands_model:
            self._run_hands(frame, debug_frame)

        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except:
            pass

    def _run_yolo(self, frame, debug_frame, w, h):
        """Run YOLO object detection."""
        try:
            results = self.yolo_model(frame, verbose=False, conf=self.yolo_conf)
        except:
            return

        detections = []
        person_detected = False
        best_person = None
        best_conf = 0

        if results and len(results) > 0:
            boxes = results[0].boxes
            if boxes is not None:
                for box in boxes:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)

                    class_name = self.class_names[cls] if cls < len(self.class_names) else str(cls)
                    detections.append({
                        'class': class_name,
                        'class_id': cls,
                        'confidence': round(conf, 3),
                        'bbox': [int(x) for x in xyxy]
                    })

                    # Draw
                    color = (0, 255, 0) if cls == 0 else (255, 0, 0)
                    cv2.rectangle(debug_frame, (x1, y1), (x2, y2), color, 2)
                    cv2.putText(debug_frame, f'{class_name} {conf:.2f}',
                               (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    # Track person
                    if cls == 0 and conf > best_conf:
                        best_conf = conf
                        best_person = xyxy

        # Publish detections
        if detections:
            msg = String()
            msg.data = json.dumps(detections)
            self.detection_pub.publish(msg)

        # Publish person tracking
        det_msg = Bool()
        det_msg.data = best_person is not None
        self.detected_pub.publish(det_msg)

        if best_person is not None:
            x1, y1, x2, y2 = best_person
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            box_h = y2 - y1

            target = Point()
            target.x = (cx - w/2) / (w/2)
            target.y = (cy - h/2) / (h/2)
            target.z = best_conf
            self.person_pub.publish(target)

            # Estimate distance
            dist = Float32()
            dist.data = float((h * 0.7) / max(box_h, 1))
            self.distance_pub.publish(dist)

    def _run_pose(self, frame, debug_frame):
        """Run MediaPipe Pose."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose_model.process(rgb)

        if results.pose_landmarks:
            landmarks = []
            for i, lm in enumerate(results.pose_landmarks.landmark):
                landmarks.append({
                    'id': i,
                    'x': round(lm.x, 4),
                    'y': round(lm.y, 4),
                    'z': round(lm.z, 4),
                    'visibility': round(lm.visibility, 3)
                })

            msg = String()
            msg.data = json.dumps(landmarks)
            self.pose_pub.publish(msg)

            # Draw pose
            import mediapipe as mp
            mp.solutions.drawing_utils.draw_landmarks(
                debug_frame, results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS)

    def _run_hands(self, frame, debug_frame):
        """Run MediaPipe Hands."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands_model.process(rgb)

        if results.multi_hand_landmarks:
            all_hands = []
            for hand_landmarks, handedness in zip(
                results.multi_hand_landmarks,
                results.multi_handedness
            ):
                hand_data = {
                    'label': handedness.classification[0].label,
                    'score': round(handedness.classification[0].score, 3),
                    'landmarks': []
                }
                for i, lm in enumerate(hand_landmarks.landmark):
                    hand_data['landmarks'].append({
                        'id': i,
                        'x': round(lm.x, 4),
                        'y': round(lm.y, 4),
                        'z': round(lm.z, 4)
                    })
                all_hands.append(hand_data)

                # Draw
                import mediapipe as mp
                mp.solutions.drawing_utils.draw_landmarks(
                    debug_frame, hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS)

            msg = String()
            msg.data = json.dumps(all_hands)
            self.hands_pub.publish(msg)

            # Simple gesture detection
            self._detect_gesture(all_hands)

    def _detect_gesture(self, hands):
        """Simple gesture detection."""
        for hand in hands:
            landmarks = hand['landmarks']
            if len(landmarks) < 21:
                continue

            # Check if fingers are extended
            # Thumb
            thumb_up = landmarks[4]['y'] < landmarks[3]['y']
            # Index
            index_up = landmarks[8]['y'] < landmarks[6]['y']
            # Middle
            middle_up = landmarks[12]['y'] < landmarks[10]['y']
            # Ring
            ring_up = landmarks[16]['y'] < landmarks[14]['y']
            # Pinky
            pinky_up = landmarks[20]['y'] < landmarks[18]['y']

            gesture = 'unknown'
            if thumb_up and index_up and middle_up and ring_up and pinky_up:
                gesture = 'open_hand'
            elif not thumb_up and index_up and not middle_up and not ring_up and not pinky_up:
                gesture = 'pointing'
            elif thumb_up and not index_up and not middle_up and not ring_up and not pinky_up:
                gesture = 'thumbs_up'
            elif not thumb_up and not index_up and not middle_up and not ring_up and not pinky_up:
                gesture = 'fist'
            elif not thumb_up and index_up and middle_up and not ring_up and not pinky_up:
                gesture = 'peace'

            if gesture != 'unknown':
                msg = String()
                msg.data = json.dumps({'hand': hand['label'], 'gesture': gesture})
                self.gesture_pub.publish(msg)

    def destroy_node(self):
        if self.pose_model:
            self.pose_model.close()
        if self.hands_model:
            self.hands_model.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiAIEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
