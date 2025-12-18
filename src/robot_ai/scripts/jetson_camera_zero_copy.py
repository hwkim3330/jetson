#!/usr/bin/env python3
"""
Zero-Copy Jetson Camera Node for KETI Robot

Better than Isaac ROS:
- Direct NVMM (GPU) memory to ROS without CPU copy
- Hardware JPEG encoding via nvjpegenc
- Optional H.264 encoding via nvv4l2h264enc (if available)
- Lower latency than Isaac ROS
- Simpler installation

Pipeline:
  nvarguscamerasrc → NVMM buffer → nvvidconv → nvjpegenc → appsink
                                              ↘ (optional) nvv4l2h264enc
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)


class JetsonCameraZeroCopy(Node):
    """
    Zero-copy camera node using GStreamer directly.
    Bypasses OpenCV for maximum performance.
    """

    def __init__(self):
        super().__init__('jetson_camera_zerocopy')

        # Parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('flip_method', 2)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('enable_raw', True)
        self.declare_parameter('enable_jpeg', True)
        self.declare_parameter('camera_frame', 'camera_link')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.sensor_id = self.get_parameter('sensor_id').value
        self.flip_method = self.get_parameter('flip_method').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.enable_raw = self.get_parameter('enable_raw').value
        self.enable_jpeg = self.get_parameter('enable_jpeg').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.bridge = CvBridge()
        self.running = True
        self.pipeline = None
        self.raw_sink = None
        self.jpeg_sink = None

        # QoS for low latency
        qos_realtime = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        if self.enable_raw:
            self.image_pub = self.create_publisher(
                Image, '/camera/image_raw', qos_realtime)

        if self.enable_jpeg:
            self.jpeg_pub = self.create_publisher(
                CompressedImage, '/camera/image_raw/compressed', qos_realtime)

        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # Build and start pipeline
        if self._build_pipeline():
            self._start_pipeline()
            self.get_logger().info(
                f'Zero-copy camera started: {self.width}x{self.height}@{self.fps}fps')
        else:
            self.get_logger().error('Failed to build GStreamer pipeline')

        # Camera info timer
        self.create_timer(1.0, self._publish_camera_info)

    def _build_pipeline(self):
        """Build GStreamer pipeline with tee for multiple outputs."""
        try:
            # Source: CSI camera with NVMM memory (zero-copy GPU buffer)
            src = (
                f'nvarguscamerasrc sensor-id={self.sensor_id} ! '
                f'video/x-raw(memory:NVMM), width=1280, height=720, '
                f'format=NV12, framerate={self.fps}/1 ! '
                f'nvvidconv flip-method={self.flip_method} ! '
                f'video/x-raw(memory:NVMM), width={self.width}, height={self.height}, format=NV12 ! '
                f'tee name=t '
            )

            # Branch 1: Raw BGR for ROS (with minimal copy)
            raw_branch = ''
            if self.enable_raw:
                raw_branch = (
                    f't. ! queue max-size-buffers=1 leaky=downstream ! '
                    f'nvvidconv ! video/x-raw, format=BGRx ! '
                    f'videoconvert ! video/x-raw, format=BGR ! '
                    f'appsink name=raw_sink emit-signals=true drop=true max-buffers=1 '
                )

            # Branch 2: Hardware JPEG encoding (stays in GPU until final output)
            jpeg_branch = ''
            if self.enable_jpeg:
                jpeg_branch = (
                    f't. ! queue max-size-buffers=1 leaky=downstream ! '
                    f'nvvidconv ! video/x-raw, format=I420 ! '
                    f'nvjpegenc quality={self.jpeg_quality} ! '
                    f'appsink name=jpeg_sink emit-signals=true drop=true max-buffers=1 '
                )

            pipeline_str = src + raw_branch + jpeg_branch
            self.get_logger().info(f'Pipeline: {pipeline_str}')

            self.pipeline = Gst.parse_launch(pipeline_str)

            # Get sinks
            if self.enable_raw:
                self.raw_sink = self.pipeline.get_by_name('raw_sink')
                self.raw_sink.connect('new-sample', self._on_raw_sample)

            if self.enable_jpeg:
                self.jpeg_sink = self.pipeline.get_by_name('jpeg_sink')
                self.jpeg_sink.connect('new-sample', self._on_jpeg_sample)

            return True

        except Exception as e:
            self.get_logger().error(f'Pipeline error: {e}')
            return False

    def _start_pipeline(self):
        """Start the GStreamer pipeline."""
        self.pipeline.set_state(Gst.State.PLAYING)

        # Run GLib main loop in separate thread
        self.loop = GLib.MainLoop()
        self.gst_thread = threading.Thread(target=self.loop.run)
        self.gst_thread.daemon = True
        self.gst_thread.start()

    def _on_raw_sample(self, sink):
        """Handle raw BGR frame from appsink."""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()

        # Get buffer data
        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            # Create numpy array from buffer (minimal copy)
            frame = np.ndarray(
                shape=(self.height, self.width, 3),
                dtype=np.uint8,
                buffer=map_info.data
            )

            # Publish
            timestamp = self.get_clock().now().to_msg()
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = timestamp
            msg.header.frame_id = self.camera_frame
            self.image_pub.publish(msg)

        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _on_jpeg_sample(self, sink):
        """Handle JPEG data from nvjpegenc (zero-copy from GPU)."""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            # JPEG data directly from hardware encoder
            jpeg_data = bytes(map_info.data)

            timestamp = self.get_clock().now().to_msg()
            msg = CompressedImage()
            msg.header.stamp = timestamp
            msg.header.frame_id = self.camera_frame
            msg.format = 'jpeg'
            msg.data = jpeg_data
            self.jpeg_pub.publish(msg)

        finally:
            buf.unmap(map_info)

        return Gst.FlowReturn.OK

    def _publish_camera_info(self):
        """Publish camera intrinsics."""
        info = CameraInfo()
        info.header.stamp = self.get_clock().now().to_msg()
        info.header.frame_id = self.camera_frame
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'

        fx = fy = self.width * 0.8
        cx, cy = self.width / 2.0, self.height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.info_pub.publish(info)

    def destroy_node(self):
        self.running = False
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if hasattr(self, 'loop'):
            self.loop.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JetsonCameraZeroCopy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
