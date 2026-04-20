#!/usr/bin/env python3
"""
Launch file for DEXI color detection node

Usage:
    # Default (5 Hz, all colors enabled)
    ros2 launch dexi_color_detection color_detection_launch.py

    # Custom frequency and contour threshold
    ros2 launch dexi_color_detection color_detection_launch.py \
        detection_frequency:=10.0 min_contour_area:=1000

    # Disable annotated image (saves CPU)
    ros2 launch dexi_color_detection color_detection_launch.py \
        publish_annotated_image:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detection_frequency_arg = DeclareLaunchArgument(
        'detection_frequency',
        default_value='5.0',
        description='Detection frequency in Hz'
    )

    min_contour_area_arg = DeclareLaunchArgument(
        'min_contour_area',
        default_value='500',
        description='Minimum contour area in pixels to count as detection'
    )

    max_detections_arg = DeclareLaunchArgument(
        'max_detections',
        default_value='10',
        description='Maximum detections per frame'
    )

    publish_annotated_arg = DeclareLaunchArgument(
        'publish_annotated_image',
        default_value='true',
        description='Publish annotated camera image with bounding boxes'
    )

    annotated_quality_arg = DeclareLaunchArgument(
        'annotated_jpeg_quality',
        default_value='75',
        description='JPEG quality for annotated image output (1-100)'
    )

    color_detection_node = Node(
        package='dexi_color_detection',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        parameters=[{
            'detection_frequency': LaunchConfiguration('detection_frequency'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'max_detections': LaunchConfiguration('max_detections'),
            'publish_annotated_image': LaunchConfiguration('publish_annotated_image'),
            'annotated_jpeg_quality': LaunchConfiguration('annotated_jpeg_quality'),
        }],
        remappings=[
            ('/cam0/image_raw/compressed', '/cam0/image_raw/compressed'),
            ('/color_detections', '/color_detections'),
        ]
    )

    info_msg = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            'DEXI Color Detection Node\n',
            '=' * 60, '\n',
            'Subscribing to: /cam0/image_raw/compressed\n',
            'Publishing to:  /color_detections\n',
            'Annotated feed: /color_detections/image/compressed\n',
            '\n',
            'Parameters:\n',
            '  detection_frequency:    ', LaunchConfiguration('detection_frequency'), ' Hz\n',
            '  min_contour_area:       ', LaunchConfiguration('min_contour_area'), ' px\n',
            '  max_detections:         ', LaunchConfiguration('max_detections'), '\n',
            '  publish_annotated_image:', LaunchConfiguration('publish_annotated_image'), '\n',
            '\n',
            'Monitor detections:\n',
            '  ros2 topic echo /color_detections\n',
            '\n',
            'Node-RED: subscribe to /color_detections via rosbridge\n',
            '=' * 60, '\n',
        ]
    )

    return LaunchDescription([
        detection_frequency_arg,
        min_contour_area_arg,
        max_detections_arg,
        publish_annotated_arg,
        annotated_quality_arg,
        info_msg,
        color_detection_node,
    ])
