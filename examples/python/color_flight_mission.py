#!/usr/bin/env python3
"""
Example: Fly toward a detected color using the offboard manager

Subscribes to /color_detections and sends velocity commands to center
the drone on a target color. Uses the DEXI offboard nav command interface.

Usage:
    # Make sure offboard_manager and color_detection_node are running, then:
    python3 color_flight_mission.py

    # Change target color:
    python3 color_flight_mission.py --color blue
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import ColorDetectionArray, OffboardNavCommand
import argparse


class ColorFlightMission(Node):
    def __init__(self, target_color='red'):
        super().__init__('color_flight_mission')
        self.target_color = target_color

        self.nav_pub = self.create_publisher(
            OffboardNavCommand,
            '/dexi/offboard_manager',
            10
        )

        self.create_subscription(
            ColorDetectionArray,
            '/color_detections',
            self.on_detections,
            10
        )

        self.get_logger().info(f'Color Flight Mission ready — tracking "{target_color}"')
        self.get_logger().info('Will send velocity commands to center on target')

    def on_detections(self, msg):
        # Find our target color in the detections
        target = None
        for det in msg.detections:
            if det.color_name == self.target_color:
                target = det
                break

        if target is None:
            return

        # How far off-center is the detection? (0.5, 0.5) = centered
        error_x = target.center_x - 0.5  # positive = target is right
        error_y = target.center_y - 0.5  # positive = target is below

        # Simple proportional control — scale to low velocity
        # Positive fly_right = drone moves right, positive fly_forward = forward
        # Map camera x-error to left/right, camera y-error to up/down
        if abs(error_x) > 0.1:
            direction = 'fly_right' if error_x > 0 else 'fly_left'
            cmd = OffboardNavCommand()
            cmd.command = direction
            cmd.param1 = min(abs(error_x) * 0.5, 0.3)  # velocity m/s, capped
            self.nav_pub.publish(cmd)
            self.get_logger().info(
                f'{self.target_color} at ({target.center_x:.2f}, {target.center_y:.2f}) '
                f'-> {direction} {cmd.param1:.2f} m/s')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='red', help='Color to track')
    # Parse only known args so ROS2 args don't cause errors
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = ColorFlightMission(target_color=args.color)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
