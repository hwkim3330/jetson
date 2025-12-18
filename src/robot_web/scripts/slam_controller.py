#!/usr/bin/env python3
"""
SLAM Controller Node
Controls Cartographer SLAM start/stop via ROS2 services
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import subprocess
import signal
import os


class SlamController(Node):
    def __init__(self):
        super().__init__('slam_controller')

        self.slam_process = None
        self.slam_running = False

        # Services
        self.start_srv = self.create_service(
            Trigger, '/slam/start', self.start_slam_callback)
        self.stop_srv = self.create_service(
            Trigger, '/slam/stop', self.stop_slam_callback)

        # Status publisher
        self.status_pub = self.create_publisher(Bool, '/slam/status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('SLAM Controller ready')

    def start_slam_callback(self, request, response):
        if self.slam_running:
            response.success = False
            response.message = 'SLAM already running'
            return response

        try:
            # Source ROS2 and launch Cartographer without RViz
            cmd = (
                'source /opt/ros/humble/setup.bash && '
                'source /home/nvidia/ros2_ws/install/setup.bash && '
                'ros2 launch robot_slam cartographer.launch.py use_rviz:=false'
            )

            self.slam_process = subprocess.Popen(
                cmd,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )

            self.slam_running = True
            response.success = True
            response.message = 'SLAM started'
            self.get_logger().info('Cartographer SLAM started')

        except Exception as e:
            response.success = False
            response.message = f'Failed to start SLAM: {str(e)}'
            self.get_logger().error(f'Failed to start SLAM: {e}')

        return response

    def stop_slam_callback(self, request, response):
        if not self.slam_running:
            response.success = False
            response.message = 'SLAM not running'
            return response

        try:
            if self.slam_process:
                # Kill process group
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
                self.slam_process.wait(timeout=5)
                self.slam_process = None

            self.slam_running = False
            response.success = True
            response.message = 'SLAM stopped'
            self.get_logger().info('Cartographer SLAM stopped')

        except Exception as e:
            # Force kill if SIGTERM fails
            try:
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
                self.slam_process = None
                self.slam_running = False
                response.success = True
                response.message = 'SLAM force stopped'
            except:
                response.success = False
                response.message = f'Failed to stop SLAM: {str(e)}'
                self.get_logger().error(f'Failed to stop SLAM: {e}')

        return response

    def publish_status(self):
        # Check if process is still running
        if self.slam_process and self.slam_running:
            poll = self.slam_process.poll()
            if poll is not None:
                self.slam_running = False
                self.slam_process = None
                self.get_logger().warn('SLAM process terminated unexpectedly')

        msg = Bool()
        msg.data = self.slam_running
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Stop SLAM on shutdown
        if self.slam_running and self.slam_process:
            try:
                os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlamController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
