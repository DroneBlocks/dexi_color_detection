#!/usr/bin/env python3
"""
Launch file for the color detection simulator (no camera needed)

Usage:
    # Default simulator
    ros2 launch dexi_color_detection color_detection_sim_launch.py

    # Faster with only certain colors
    ros2 launch dexi_color_detection color_detection_sim_launch.py \
        detection_frequency:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detection_frequency_arg = DeclareLaunchArgument(
        'detection_frequency',
        default_value='2.0',
        description='Simulated detection frequency in Hz'
    )

    detection_probability_arg = DeclareLaunchArgument(
        'detection_probability',
        default_value='0.7',
        description='Probability of detecting a color each frame'
    )

    sim_node = Node(
        package='dexi_color_detection',
        executable='color_detection_simulator.py',
        name='color_detection_simulator',
        output='screen',
        parameters=[{
            'detection_frequency': LaunchConfiguration('detection_frequency'),
            'detection_probability': LaunchConfiguration('detection_probability'),
        }]
    )

    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'DEXI Color Detection SIMULATOR\n',
            '=' * 60, '\n',
            'Publishing simulated detections to: /color_detections\n',
            'No camera required — use for Node-RED / Python testing\n',
            '\n',
            'Monitor:\n',
            '  ros2 topic echo /color_detections\n',
            '=' * 60, '\n',
        ]
    )

    return LaunchDescription([
        detection_frequency_arg,
        detection_probability_arg,
        info_msg,
        sim_node,
    ])
