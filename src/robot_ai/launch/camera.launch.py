#!/usr/bin/env python3
"""
Camera Launch File for KETI Robot
Launches the Jetson camera node with configurable parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments - HD 16:9 defaults for Jetson Orin Nano
    width_arg = DeclareLaunchArgument(
        'width', default_value='1280',
        description='Camera image width'
    )
    height_arg = DeclareLaunchArgument(
        'height', default_value='720',
        description='Camera image height'
    )
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='30',
        description='Camera framerate'
    )
    sensor_id_arg = DeclareLaunchArgument(
        'sensor_id', default_value='0',
        description='Camera sensor ID (0 for CSI, or /dev/videoX number)'
    )
    flip_method_arg = DeclareLaunchArgument(
        'flip_method', default_value='2',
        description='Flip method (0=none, 2=rotate180)'
    )
    use_v4l2_arg = DeclareLaunchArgument(
        'use_v4l2', default_value='false',
        description='Use V4L2 instead of GStreamer CSI'
    )

    # Camera node
    camera_node = Node(
        package='robot_ai',
        executable='jetson_camera.py',
        name='jetson_camera',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'sensor_id': LaunchConfiguration('sensor_id'),
            'flip_method': LaunchConfiguration('flip_method'),
            'use_v4l2': LaunchConfiguration('use_v4l2'),
        }],
        output='screen'
    )

    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        sensor_id_arg,
        flip_method_arg,
        use_v4l2_arg,
        camera_node,
    ])
