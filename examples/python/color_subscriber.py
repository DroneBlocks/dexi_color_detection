#!/usr/bin/env python3
"""
Example: Simple color detection subscriber

Prints detected colors to the console. Use this as a starting point
for your own color-reactive missions.

Usage:
    python3 color_subscriber.py
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import ColorDetectionArray


class ColorSubscriber(Node):
    def __init__(self):
        super().__init__('color_subscriber')
        self.create_subscription(
            ColorDetectionArray,
            '/color_detections',
            self.on_detections,
            10
        )
        self.get_logger().info('Listening for color detections on /color_detections ...')

    def on_detections(self, msg):
        if not msg.detections:
            return

        for det in msg.detections:
            self.get_logger().info(
                f'{det.color_name}: confidence={det.confidence:.0%} '
                f'center=({det.center_x:.2f}, {det.center_y:.2f}) '
                f'pixels={det.pixel_count}'
            )


def main():
    rclpy.init()
    rclpy.spin(ColorSubscriber())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
