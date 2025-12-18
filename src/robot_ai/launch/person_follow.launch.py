#!/usr/bin/env python3
"""
Person Following Launch File
Launches person detector and follower nodes
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_ai')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim',
            default_value='false',
            description='Use simulation time'
        ),

        # Person Detector Node
        Node(
            package='robot_ai',
            executable='person_detector.py',
            name='person_detector',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.4,
                'input_width': 300,
                'input_height': 300,
                'use_cuda': True,
                'detection_rate': 10.0,
            }]
        ),

        # Person Follower Node
        Node(
            package='robot_ai',
            executable='person_follower.py',
            name='person_follower',
            output='screen',
            parameters=[{
                'linear_gain': 0.3,
                'angular_gain': 0.8,
                'max_linear': 0.2,
                'max_angular': 0.8,
                'target_distance': 1.2,
                'min_distance': 0.5,
                'lost_timeout': 2.0,
            }]
        ),
    ])
