#!/usr/bin/env python3
"""
KETI Robot Web Server
- Serves static files
- Integrates with ROS2 for mode control
- More stable than nginx for embedded systems
"""

import os
import json
import signal
import subprocess
from http.server import HTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class RobotWebHandler(SimpleHTTPRequestHandler):
    """Custom HTTP handler with ROS2 integration"""

    ros_node = None
    mode_pub = None

    def __init__(self, *args, **kwargs):
        # Set the directory to serve files from
        self.directory = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'www'
        )
        super().__init__(*args, directory=self.directory, **kwargs)

    def do_GET(self):
        """Handle GET requests"""
        parsed = urlparse(self.path)

        # API endpoints
        if parsed.path == '/api/mode':
            self.handle_mode_get(parsed)
        elif parsed.path == '/api/status':
            self.handle_status()
        else:
            # Serve static files
            super().do_GET()

    def do_POST(self):
        """Handle POST requests"""
        parsed = urlparse(self.path)

        if parsed.path == '/api/mode':
            self.handle_mode_set()
        elif parsed.path == '/api/slam/start':
            self.handle_slam_start()
        elif parsed.path == '/api/slam/stop':
            self.handle_slam_stop()
        elif parsed.path == '/api/slam/save':
            self.handle_slam_save()
        elif parsed.path == '/api/nav/start':
            self.handle_nav_start()
        elif parsed.path == '/api/nav/stop':
            self.handle_nav_stop()
        else:
            self.send_error(404, 'Not Found')

    def handle_mode_get(self, parsed):
        """Get current mode"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        response = {'mode': getattr(self, 'current_mode', 'manual')}
        self.wfile.write(json.dumps(response).encode())

    def handle_mode_set(self):
        """Set robot mode"""
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)

        try:
            data = json.loads(post_data.decode())
            mode = data.get('mode', 'manual')

            # Publish mode change
            if RobotWebHandler.mode_pub:
                msg = String()
                msg.data = mode
                RobotWebHandler.mode_pub.publish(msg)

            RobotWebHandler.current_mode = mode

            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps({'success': True, 'mode': mode}).encode())

        except Exception as e:
            self.send_error(400, str(e))

    def handle_status(self):
        """Get robot status"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        response = {
            'status': 'ok',
            'mode': getattr(RobotWebHandler, 'current_mode', 'manual'),
            'slam_running': getattr(RobotWebHandler, 'slam_running', False),
            'nav_running': getattr(RobotWebHandler, 'nav_running', False)
        }
        self.wfile.write(json.dumps(response).encode())

    def handle_slam_start(self):
        """Start SLAM"""
        try:
            # Launch SLAM in background
            subprocess.Popen([
                'ros2', 'launch', 'robot_slam', 'slam.launch.py'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            RobotWebHandler.slam_running = True
            self.send_json_response({'success': True, 'message': 'SLAM started'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

    def handle_slam_stop(self):
        """Stop SLAM"""
        try:
            subprocess.run(['pkill', '-f', 'slam.launch.py'], check=False)
            subprocess.run(['pkill', '-f', 'cartographer'], check=False)
            RobotWebHandler.slam_running = False
            self.send_json_response({'success': True, 'message': 'SLAM stopped'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

    def handle_slam_save(self):
        """Save SLAM map"""
        try:
            map_dir = os.path.expanduser('~/ros2_ws/maps')
            os.makedirs(map_dir, exist_ok=True)

            subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', os.path.join(map_dir, 'map')
            ], check=True)

            self.send_json_response({'success': True, 'message': 'Map saved'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

    def handle_nav_start(self):
        """Start Navigation"""
        try:
            subprocess.Popen([
                'ros2', 'launch', 'robot_navigation', 'navigation.launch.py'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            RobotWebHandler.nav_running = True
            self.send_json_response({'success': True, 'message': 'Navigation started'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

    def handle_nav_stop(self):
        """Stop Navigation"""
        try:
            subprocess.run(['pkill', '-f', 'navigation.launch.py'], check=False)
            subprocess.run(['pkill', '-f', 'nav2'], check=False)
            RobotWebHandler.nav_running = False
            self.send_json_response({'success': True, 'message': 'Navigation stopped'})
        except Exception as e:
            self.send_json_response({'success': False, 'error': str(e)})

    def send_json_response(self, data):
        """Send JSON response"""
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(json.dumps(data).encode())

    def do_OPTIONS(self):
        """Handle CORS preflight"""
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def log_message(self, format, *args):
        """Suppress default logging"""
        pass


class WebServerNode(Node):
    """ROS2 Node for web server"""

    def __init__(self):
        super().__init__('robot_web_server')

        # Publishers
        self.mode_pub = self.create_publisher(String, '/robot/mode', 10)
        RobotWebHandler.mode_pub = self.mode_pub
        RobotWebHandler.ros_node = self

        # Services
        self.mode_srv = self.create_service(
            SetBool, '/robot/set_mode_enabled', self.set_mode_callback
        )

        # Start HTTP server in separate thread
        self.port = self.declare_parameter('port', 8888).value
        self.server = HTTPServer(('0.0.0.0', self.port), RobotWebHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()

        self.get_logger().info(f'Web server started on port {self.port}')

    def set_mode_callback(self, request, response):
        """Handle mode enable/disable service"""
        response.success = True
        response.message = 'Mode updated'
        return response

    def destroy_node(self):
        """Cleanup"""
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
