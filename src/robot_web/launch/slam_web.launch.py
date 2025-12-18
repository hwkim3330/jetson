#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    robot_bringup_dir = get_package_share_directory('robot_bringup')
    robot_slam_dir = get_package_share_directory('robot_slam')
    robot_web_dir = get_package_share_directory('robot_web')

    www_dir = os.path.join(robot_web_dir, 'www')

    return LaunchDescription([
        # Include robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_bringup_dir, 'launch', 'robot.launch.py')
            )
        ),

        # Include cartographer SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_slam_dir, 'launch', 'cartographer.launch.py')
            )
        ),

        # ROSBridge WebSocket Server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen'
        ),

        # Web Video Server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8080}],
            output='screen'
        ),

        # HTTP Server
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8888', '--directory', www_dir],
            output='screen'
        ),
    ])
