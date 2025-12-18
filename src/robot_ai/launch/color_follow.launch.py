#!/usr/bin/env python3
"""
Color-based Body Following Launch File
Lightweight tracking using color detection
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'target_color',
            default_value='red',
            description='Target color to track (red, green, blue, yellow)'
        ),

        # Body Tracker Node
        Node(
            package='robot_ai',
            executable='body_tracker.py',
            name='body_tracker',
            output='screen',
            parameters=[{
                'tracking_rate': 15.0,
                'min_area': 3000,
                'max_area': 200000,
                'linear_gain': 0.25,
                'angular_gain': 0.7,
                'max_linear': 0.18,
                'max_angular': 0.7,
                'target_area': 25000,
            }]
        ),
    ])
