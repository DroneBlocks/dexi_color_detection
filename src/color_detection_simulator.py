#!/usr/bin/env python3
"""
Color detection simulator for desktop development and testing.
Publishes simulated ColorDetectionArray messages without requiring
a camera or OpenCV processing.

Useful for:
  - Testing Node-RED flows that subscribe to /color_detections
  - Testing Python subscriber scripts
  - Developing LED reactions to color events without hardware
"""

import rclpy
from rclpy.node import Node
from dexi_interfaces.msg import ColorDetection as ColorDetectionMsg, ColorDetectionArray
import time
import random
import math


class ColorDetectionSimulator(Node):
    """Simulator node that publishes realistic color detection data"""

    def __init__(self):
        super().__init__('color_detection_simulator')

        # Parameters
        self.declare_parameter('detection_frequency', 2.0)
        self.declare_parameter('detection_probability', 0.7)
        self.declare_parameter('max_colors_per_frame', 2)
        self.declare_parameter('enable_movement', True)
        self.declare_parameter('colors', ['red', 'orange', 'yellow', 'green', 'blue', 'pink'])

        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.detection_probability = self.get_parameter('detection_probability').value
        self.max_colors = self.get_parameter('max_colors_per_frame').value
        self.enable_movement = self.get_parameter('enable_movement').value
        self.available_colors = self.get_parameter('colors').value

        # Publisher
        self.detection_pub = self.create_publisher(
            ColorDetectionArray,
            '/color_detections',
            10
        )

        # Timer
        timer_period = 1.0 / max(self.detection_frequency, 0.1)
        self.timer = self.create_timer(timer_period, self.publish_simulated)

        # Simulation state — persistent "objects" that drift around the frame
        self.objects = []
        self.frame_count = 0
        self.start_time = time.time()

        self.get_logger().info('Color Detection Simulator initialized')
        self.get_logger().info(f'Publishing to: /color_detections')
        self.get_logger().info(f'Frequency: {self.detection_frequency} Hz')
        self.get_logger().info(f'Colors: {", ".join(self.available_colors)}')

    def publish_simulated(self):
        self.frame_count += 1
        now = time.time()
        elapsed = now - self.start_time

        msg = ColorDetectionArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_sim'
        msg.timestamp = now

        # Randomly spawn / despawn objects
        if random.random() < 0.1 and len(self.objects) < self.max_colors:
            self.objects.append({
                'color': random.choice(self.available_colors),
                'cx': random.uniform(0.2, 0.8),
                'cy': random.uniform(0.2, 0.8),
                'size': random.uniform(0.05, 0.2),
                'dx': random.uniform(-0.005, 0.005),
                'dy': random.uniform(-0.005, 0.005),
                'phase': random.uniform(0, 2 * math.pi),
                'ttl': random.randint(20, 100),
            })

        # Age out old objects
        self.objects = [o for o in self.objects if o['ttl'] > 0]

        if random.random() > self.detection_probability:
            # No detection this frame
            self.detection_pub.publish(msg)
            return

        for obj in self.objects:
            obj['ttl'] -= 1

            if self.enable_movement:
                obj['cx'] += obj['dx'] + 0.01 * math.sin(elapsed * 0.5 + obj['phase'])
                obj['cy'] += obj['dy'] + 0.008 * math.cos(elapsed * 0.3 + obj['phase'])
                obj['cx'] = max(0.05, min(0.95, obj['cx']))
                obj['cy'] = max(0.05, min(0.95, obj['cy']))

            half = obj['size'] / 2
            det = ColorDetectionMsg()
            det.color_name = obj['color']
            det.confidence = random.uniform(0.4, 0.95)
            det.bbox = [
                max(0.0, obj['cx'] - half),
                max(0.0, obj['cy'] - half),
                min(1.0, obj['cx'] + half),
                min(1.0, obj['cy'] + half),
            ]
            det.center_x = obj['cx']
            det.center_y = obj['cy']
            det.pixel_count = int(obj['size'] * obj['size'] * 640 * 480)
            msg.detections.append(det)

        self.detection_pub.publish(msg)

        if msg.detections and self.frame_count % 20 == 0:
            colors = [d.color_name for d in msg.detections]
            self.get_logger().info(f'[sim] Frame {self.frame_count}: {", ".join(colors)}')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
