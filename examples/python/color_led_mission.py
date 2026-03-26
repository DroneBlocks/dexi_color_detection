#!/usr/bin/env python3
"""
Example: Set LED ring color based on detected color

Subscribes to /color_detections and calls the LED service
to match the drone's LED ring to whatever color the camera sees.

Usage:
    # Make sure color_detection_node and led_service are running, then:
    python3 color_led_mission.py
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import ColorDetectionArray
from dexi_interfaces.srv import LEDRingColor


class ColorLedMission(Node):
    def __init__(self):
        super().__init__('color_led_mission')
        self.led = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')
        self.led.wait_for_service(timeout_sec=5.0)

        self.create_subscription(
            ColorDetectionArray,
            '/color_detections',
            self.on_detections,
            10
        )

        self._current_color = None
        self.set_led('white')
        self.get_logger().info('Color LED Mission ready — watching /color_detections')

    def set_led(self, color):
        if color == self._current_color:
            return
        req = LEDRingColor.Request()
        req.color = color
        self.led.call_async(req)
        self._current_color = color

    def on_detections(self, msg):
        if msg.detections:
            # Use the largest detection (first one — sorted by area)
            color = msg.detections[0].color_name
            self.set_led(color)
            self.get_logger().info(f'Detected {color} — LED set to {color}')
        else:
            self.set_led('white')


def main():
    rclpy.init()
    node = ColorLedMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_led('white')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
