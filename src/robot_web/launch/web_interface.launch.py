#!/usr/bin/env python3
"""
KETI Robot Web Interface Launch File

Launches:
- Robot bringup (motors, sensors, TF)
- Jetson camera node
- Python HTTP Server (port 8888) - web interface
- ROSBridge WebSocket server (port 9090)
- Web Video Server (port 8080)

Access: http://<robot_ip>:8888/
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    robot_ai_dir = get_package_share_directory('robot_ai')
    robot_web_dir = get_package_share_directory('robot_web')
    www_dir = os.path.join(robot_web_dir, 'www')

    return LaunchDescription([
        # Include robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'robot.launch.py')
            )
        ),

        # Include Jetson camera node - HD 16:9 for Orin Nano
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_ai_dir, 'launch', 'camera.launch.py')
            ),
            launch_arguments={
                'width': '1280',
                'height': '720',
                'fps': '30',
            }.items(),
        ),

        # Python HTTP Server for web interface (port 8888)
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8888', '--bind', '0.0.0.0'],
            cwd=www_dir,
            output='screen'
        ),

        # ROSBridge WebSocket Server (port 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': '0.0.0.0',
                'retry_startup_delay': 5.0,
                'send_action_goals_in_new_thread': True,
            }],
            output='screen'
        ),

        # Note: web_video_server removed - camera_node has built-in MJPEG server on port 8080

        # SLAM/Nav Controller - SLAM start/stop, map save/load + Map relay for web
        ExecuteProcess(
            cmd=['python3', '-c', '''
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import Bool, String
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import subprocess, signal, os, glob, time

MAP_DIR = os.path.expanduser("~/maps")

class SlamNavCtrl(Node):
    def __init__(self):
        super().__init__("slam_nav_controller")
        os.makedirs(MAP_DIR, exist_ok=True)
        self.slam_proc = None
        self.nav_proc = None
        self.slam_running = False
        self.nav_running = False

        # SLAM services
        self.create_service(Trigger, "/slam/start", self.slam_start)
        self.create_service(Trigger, "/slam/stop", self.slam_stop)
        self.create_service(Trigger, "/slam/save", self.slam_save)

        # Navigation services
        self.create_service(Trigger, "/nav/start", self.nav_start)
        self.create_service(Trigger, "/nav/stop", self.nav_stop)

        # Map list publisher
        self.map_list_pub = self.create_publisher(String, "/map_list", 10)

        # Status publishers
        self.slam_status_pub = self.create_publisher(Bool, "/slam/status", 10)
        self.nav_status_pub = self.create_publisher(Bool, "/nav/status", 10)

        # Map relay: subscribe with TRANSIENT_LOCAL, republish with default QoS for rosbridge
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map_web", 10)
        self.last_map = None

        # Path relay for web display
        self.path_sub = self.create_subscription(Path, "/plan", self.path_callback, 10)
        self.path_pub = self.create_publisher(Path, "/plan_web", 10)

        # Robot pose for web (from amcl - PoseWithCovarianceStamped)
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, amcl_qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/robot_pose_web", 10)

        # Periodic map republish (for late subscribers)
        self.create_timer(2.0, self.republish_map)
        self.create_timer(1.0, self.tick)
        self.get_logger().info("SLAM/Nav Controller ready. Maps: " + MAP_DIR)

    def map_callback(self, msg):
        self.last_map = msg
        self.map_pub.publish(msg)
        self.get_logger().info(f"Map relayed: {msg.info.width}x{msg.info.height}")

    def path_callback(self, msg):
        self.path_pub.publish(msg)

    def pose_callback(self, msg):
        # Convert PoseWithCovarianceStamped to PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.pose_pub.publish(pose_stamped)

    def republish_map(self):
        if self.last_map and self.nav_running:
            self.map_pub.publish(self.last_map)

    def slam_start(self, req, res):
        if self.slam_running:
            res.success, res.message = False, "Already running"
            return res
        try:
            cmd = "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 launch robot_slam cartographer.launch.py use_rviz:=false"
            self.slam_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.slam_running = True
            res.success, res.message = True, "SLAM started"
            self.get_logger().info("SLAM started")
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def slam_stop(self, req, res):
        if not self.slam_running:
            res.success, res.message = False, "Not running"
            return res
        try:
            os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGTERM)
            self.slam_proc = None
            self.slam_running = False
            res.success, res.message = True, "SLAM stopped"
            self.get_logger().info("SLAM stopped")
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def slam_save(self, req, res):
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            map_name = f"{MAP_DIR}/map_{timestamp}"
            cmd = f"source /opt/ros/humble/setup.bash && ros2 run nav2_map_server map_saver_cli -f {map_name} --ros-args -p save_map_timeout:=10.0"
            result = subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, timeout=15)
            if result.returncode == 0:
                res.success, res.message = True, f"Map saved: map_{timestamp}"
                self.get_logger().info(f"Map saved: {map_name}")
            else:
                res.success, res.message = False, "Save failed - check SLAM is running"
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def nav_start(self, req, res):
        if self.nav_running:
            res.success, res.message = False, "Already running"
            return res
        # Find latest map
        maps = sorted(glob.glob(f"{MAP_DIR}/*.yaml"))
        if not maps:
            res.success, res.message = False, "No maps found"
            return res
        latest_map = maps[-1]
        try:
            cmd = f"source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 launch robot_navigation navigation2.launch.py map:={latest_map} use_rviz:=false"
            self.nav_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash", preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.nav_running = True
            res.success, res.message = True, f"Nav started: {os.path.basename(latest_map)}"
            self.get_logger().info(f"Navigation started with {latest_map}")
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def nav_stop(self, req, res):
        if not self.nav_running:
            res.success, res.message = False, "Not running"
            return res
        try:
            os.killpg(os.getpgid(self.nav_proc.pid), signal.SIGTERM)
            self.nav_proc = None
            self.nav_running = False
            res.success, res.message = True, "Nav stopped"
            self.get_logger().info("Navigation stopped")
        except Exception as e:
            res.success, res.message = False, str(e)
        return res

    def tick(self):
        # Check processes
        if self.slam_proc and self.slam_running and self.slam_proc.poll() is not None:
            self.slam_running = False
            self.slam_proc = None
        if self.nav_proc and self.nav_running and self.nav_proc.poll() is not None:
            self.nav_running = False
            self.nav_proc = None
        # Publish status
        self.slam_status_pub.publish(Bool(data=self.slam_running))
        self.nav_status_pub.publish(Bool(data=self.nav_running))
        # Publish map list
        maps = sorted(glob.glob(f"{MAP_DIR}/*.yaml"))
        map_names = ",".join([os.path.basename(m).replace(".yaml","") for m in maps[-5:]])
        self.map_list_pub.publish(String(data=map_names))

rclpy.init()
rclpy.spin(SlamNavCtrl())
'''],
            output='screen'
        ),

        # AI Mode Controller - manages AI features from web interface
        ExecuteProcess(
            cmd=['python3', '-c', '''
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import subprocess, signal, os

class AIController(Node):
    def __init__(self):
        super().__init__("ai_controller")

        # Process tracking
        self.follow_proc = None
        self.gesture_proc = None
        self.yolo_proc = None
        self.color_proc = None

        # State tracking
        self.follow_enabled = False
        self.gesture_enabled = False
        self.yolo_enabled = False
        self.color_enabled = False

        # Enable subscribers (from web interface)
        self.create_subscription(Bool, "/follow_enable", self.follow_callback, 10)
        self.create_subscription(Bool, "/gesture_enable", self.gesture_callback, 10)
        self.create_subscription(Bool, "/yolo_enable", self.yolo_callback, 10)
        self.create_subscription(Bool, "/tracker_enable", self.color_callback, 10)

        # Status publishers (to web interface)
        self.follow_status_pub = self.create_publisher(Bool, "/follow_status", 10)
        self.gesture_status_pub = self.create_publisher(Bool, "/gesture_status", 10)
        self.yolo_status_pub = self.create_publisher(Bool, "/yolo_status", 10)
        self.color_status_pub = self.create_publisher(Bool, "/color_status", 10)
        self.ai_mode_pub = self.create_publisher(String, "/ai_mode", 10)

        # Cmd_vel publisher for stopping
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Status timer
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info("AI Controller ready")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def kill_all_ai(self):
        """Kill all AI processes before starting new one"""
        for proc, name in [(self.follow_proc, "follow"), (self.gesture_proc, "gesture"),
                          (self.yolo_proc, "yolo"), (self.color_proc, "color")]:
            if proc:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except: pass
        self.follow_proc = self.gesture_proc = self.yolo_proc = self.color_proc = None
        self.follow_enabled = self.gesture_enabled = self.yolo_enabled = self.color_enabled = False
        self.stop_robot()

    def start_follow(self):
        self.kill_all_ai()
        cmd = "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 launch robot_ai person_follow.launch.py"
        self.follow_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                           preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.follow_enabled = True
        self.get_logger().info("Person Follow started")

    def start_gesture(self):
        self.kill_all_ai()
        cmd = "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 run robot_ai gesture_detector.py"
        self.gesture_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                            preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.gesture_enabled = True
        self.get_logger().info("Gesture detector started")

    def start_yolo(self):
        self.kill_all_ai()
        cmd = "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 run robot_ai yolo_detector.py"
        self.yolo_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                         preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.yolo_enabled = True
        self.get_logger().info("YOLO detector started")

    def start_color(self):
        self.kill_all_ai()
        cmd = "source /opt/ros/humble/setup.bash && source /home/nvidia/ros2_ws/install/setup.bash && ros2 launch robot_ai color_follow.launch.py"
        self.color_proc = subprocess.Popen(cmd, shell=True, executable="/bin/bash",
                                          preexec_fn=os.setsid, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.color_enabled = True
        self.get_logger().info("Color tracker started")

    def follow_callback(self, msg):
        if msg.data and not self.follow_enabled:
            self.start_follow()
        elif not msg.data and self.follow_enabled:
            self.kill_all_ai()
            self.get_logger().info("Person Follow stopped")

    def gesture_callback(self, msg):
        if msg.data and not self.gesture_enabled:
            self.start_gesture()
        elif not msg.data and self.gesture_enabled:
            self.kill_all_ai()
            self.get_logger().info("Gesture stopped")

    def yolo_callback(self, msg):
        if msg.data and not self.yolo_enabled:
            self.start_yolo()
        elif not msg.data and self.yolo_enabled:
            self.kill_all_ai()
            self.get_logger().info("YOLO stopped")

    def color_callback(self, msg):
        if msg.data and not self.color_enabled:
            self.start_color()
        elif not msg.data and self.color_enabled:
            self.kill_all_ai()
            self.get_logger().info("Color tracker stopped")

    def publish_status(self):
        # Check if processes are still running
        if self.follow_proc and self.follow_proc.poll() is not None:
            self.follow_enabled = False
            self.follow_proc = None
        if self.gesture_proc and self.gesture_proc.poll() is not None:
            self.gesture_enabled = False
            self.gesture_proc = None
        if self.yolo_proc and self.yolo_proc.poll() is not None:
            self.yolo_enabled = False
            self.yolo_proc = None
        if self.color_proc and self.color_proc.poll() is not None:
            self.color_enabled = False
            self.color_proc = None

        # Publish status
        self.follow_status_pub.publish(Bool(data=self.follow_enabled))
        self.gesture_status_pub.publish(Bool(data=self.gesture_enabled))
        self.yolo_status_pub.publish(Bool(data=self.yolo_enabled))
        self.color_status_pub.publish(Bool(data=self.color_enabled))

        # Publish current mode
        mode = "none"
        if self.follow_enabled: mode = "follow"
        elif self.gesture_enabled: mode = "gesture"
        elif self.yolo_enabled: mode = "yolo"
        elif self.color_enabled: mode = "color"
        self.ai_mode_pub.publish(String(data=mode))

rclpy.init()
rclpy.spin(AIController())
'''],
            output='screen'
        ),

        # Auto Explore Node - simple obstacle avoidance exploration
        ExecuteProcess(
            cmd=['python3', '-c', '''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class ExploreNode(Node):
    def __init__(self):
        super().__init__("explore_node")
        self.linear_speed = 0.15
        self.angular_speed = 0.8
        self.obstacle_dist = 0.45
        self.side_dist = 0.30
        self.exploring = False
        self.state = "forward"
        self.rotate_time = 0.0
        self.scan_data = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/robot/mode", self.mode_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_timer(0.1, self.control)
        self.get_logger().info("Explore node ready")

    def mode_cb(self, msg):
        if msg.data == "explore":
            if not self.exploring:
                self.exploring = True
                self.state = "forward"
                self.get_logger().info("Auto explore started")
        else:
            if self.exploring:
                self.exploring = False
                self.stop()
                self.get_logger().info("Auto explore stopped")

    def scan_cb(self, msg):
        self.scan_data = msg

    def get_min_range(self, start_deg, end_deg):
        if not self.scan_data:
            return float("inf")
        min_r = float("inf")
        for deg in range(int(start_deg), int(end_deg), 3):
            rad = math.radians(deg)
            if rad < self.scan_data.angle_min or rad > self.scan_data.angle_max:
                continue
            idx = int((rad - self.scan_data.angle_min) / self.scan_data.angle_increment)
            if 0 <= idx < len(self.scan_data.ranges):
                r = self.scan_data.ranges[idx]
                if self.scan_data.range_min < r < self.scan_data.range_max and r < min_r:
                    min_r = r
        return min_r

    def control(self):
        if not self.exploring or not self.scan_data:
            return
        front = self.get_min_range(-25, 25)
        front_left = self.get_min_range(25, 60)
        front_right = self.get_min_range(-60, -25)
        left = self.get_min_range(60, 100)
        right = self.get_min_range(-100, -60)
        cmd = Twist()

        if self.state == "forward":
            if front < self.obstacle_dist:
                self.state = "rotate_left" if left > right else "rotate_right"
                self.rotate_time = 0.0
                self.get_logger().info(f"Obstacle at {front:.2f}m, turning")
            elif front_left < self.side_dist:
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = -0.3
            elif front_right < self.side_dist:
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = 0.3
            else:
                cmd.linear.x = self.linear_speed
        elif self.state == "rotate_left":
            self.rotate_time += 0.1
            cmd.angular.z = self.angular_speed
            if front > self.obstacle_dist * 1.5 and self.rotate_time > 0.5:
                self.state = "forward"
            elif self.rotate_time > 4.0:
                self.state = "rotate_right"
                self.rotate_time = 0.0
        elif self.state == "rotate_right":
            self.rotate_time += 0.1
            cmd.angular.z = -self.angular_speed
            if front > self.obstacle_dist * 1.5 and self.rotate_time > 0.5:
                self.state = "forward"
            elif self.rotate_time > 4.0:
                self.state = "rotate_left"
                self.rotate_time = 0.0

        self.cmd_pub.publish(cmd)

    def stop(self):
        self.cmd_pub.publish(Twist())

rclpy.init()
rclpy.spin(ExploreNode())
'''],
            output='screen'
        ),
    ])
