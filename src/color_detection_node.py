#!/usr/bin/env python3
"""
HSV color detection ROS2 node for DEXI

Subscribes to compressed camera images, detects colors using configurable
HSV ranges, and publishes results on /color_detections as ColorDetectionArray.

Works with:
  - Node-RED via rosbridge (subscribe to /color_detections)
  - Python scripts (from dexi_interfaces.msg import ColorDetectionArray)
  - ros2 topic echo /color_detections
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from dexi_interfaces.msg import ColorDetection as ColorDetectionMsg, ColorDetectionArray
import cv2
import math
import numpy as np
import time


# BGR colors for drawing annotations
DRAW_COLORS = {
    'red':    (0, 0, 255),
    'orange': (0, 140, 255),
    'yellow': (0, 220, 255),
    'green':  (0, 200, 0),
    'blue':   (255, 0, 0),
    'pink':   (200, 0, 200),
}

DEFAULT_BGR = (200, 200, 200)


class ColorDetectionNode(Node):
    """ROS2 node for HSV color detection on camera images"""

    def __init__(self):
        super().__init__('color_detection_node')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('detection_frequency', 5.0)
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('max_detections', 10)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('annotated_jpeg_quality', 75)

        self.detection_frequency = self.get_parameter('detection_frequency').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.max_detections = self.get_parameter('max_detections').value
        self.publish_annotated = self.get_parameter('publish_annotated_image').value
        self.jpeg_quality = self.get_parameter('annotated_jpeg_quality').value

        # ── Load color definitions from parameters ────────────────
        self.colors = self._load_color_params()

        # ── Publishers ────────────────────────────────────────────
        self.detection_pub = self.create_publisher(
            ColorDetectionArray,
            '/color_detections',
            10
        )

        self.annotated_pub = None
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                CompressedImage,
                '/color_detections/image/compressed',
                10
            )

        # ── Subscriber ────────────────────────────────────────────
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/cam0/image_raw/compressed',
            self.image_callback,
            10
        )

        # ── Rate limiting ─────────────────────────────────────────
        self.last_detection_time = 0.0
        self.min_detection_interval = 1.0 / max(self.detection_frequency, 0.1)

        # ── Stats ─────────────────────────────────────────────────
        self.frame_count = 0
        self.detection_count = 0

        color_names = [name for name, cfg in self.colors.items() if cfg['enabled']]
        self.get_logger().info('DEXI Color Detection node initialized')
        self.get_logger().info(f'Subscribing to: /cam0/image_raw/compressed')
        self.get_logger().info(f'Publishing to:  /color_detections')
        if self.publish_annotated:
            self.get_logger().info(f'Annotated feed: /color_detections/image/compressed')
        self.get_logger().info(f'Detection freq: {self.detection_frequency} Hz')
        self.get_logger().info(f'Min contour:    {self.min_contour_area} px')
        self.get_logger().info(f'Colors enabled: {", ".join(color_names)}')

    def _load_color_params(self):
        """Load color HSV ranges from ROS2 parameters"""
        defaults = {
            'red':    {'h_min': 0, 'h_max': 10, 's_min': 120, 's_max': 255,
                       'v_min': 70, 'v_max': 255, 'h_min2': 170, 'h_max2': 179, 'enabled': True},
            'orange': {'h_min': 11, 'h_max': 25, 's_min': 150, 's_max': 255,
                       'v_min': 100, 'v_max': 255, 'enabled': True},
            'yellow': {'h_min': 26, 'h_max': 34, 's_min': 120, 's_max': 255,
                       'v_min': 100, 'v_max': 255, 'enabled': True},
            'green':  {'h_min': 35, 'h_max': 85, 's_min': 60, 's_max': 255,
                       'v_min': 50, 'v_max': 255, 'enabled': True},
            'blue':   {'h_min': 95, 'h_max': 130, 's_min': 80, 's_max': 255,
                       'v_min': 50, 'v_max': 255, 'enabled': True},
            'pink':   {'h_min': 145, 'h_max': 169, 's_min': 80, 's_max': 255,
                       'v_min': 50, 'v_max': 255, 'enabled': True},
        }

        colors = {}
        for name, d in defaults.items():
            prefix = f'colors.{name}'
            for key, val in d.items():
                self.declare_parameter(f'{prefix}.{key}', val)

            cfg = {}
            for key in d:
                cfg[key] = self.get_parameter(f'{prefix}.{key}').value

            # Build numpy arrays for cv2.inRange
            cfg['lower'] = np.array([cfg['h_min'], cfg['s_min'], cfg['v_min']])
            cfg['upper'] = np.array([cfg['h_max'], cfg['s_max'], cfg['v_max']])
            if 'h_min2' in cfg and cfg.get('h_min2') is not None:
                cfg['lower2'] = np.array([cfg['h_min2'], cfg['s_min'], cfg['v_min']])
                cfg['upper2'] = np.array([cfg['h_max2'], cfg['s_max'], cfg['v_max']])

            colors[name] = cfg

        return colors

    def image_callback(self, msg: CompressedImage):
        """Process incoming camera frame for color detection"""
        now = time.time()
        if now - self.last_detection_time < self.min_detection_interval:
            return

        self.last_detection_time = now
        self.frame_count += 1

        # Decode compressed image
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed image')
            return

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_area = h * w

        detections = []

        for name, cfg in self.colors.items():
            if not cfg['enabled']:
                continue

            # Primary HSV mask
            mask = cv2.inRange(hsv, cfg['lower'], cfg['upper'])

            # Secondary hue range (for red wrapping around 180)
            if 'lower2' in cfg:
                mask2 = cv2.inRange(hsv, cfg['lower2'], cfg['upper2'])
                mask = cv2.bitwise_or(mask, mask2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue

            # Process contours above the area threshold, sorted largest first
            valid = [(c, cv2.contourArea(c)) for c in contours
                     if cv2.contourArea(c) >= self.min_contour_area]
            valid.sort(key=lambda x: x[1], reverse=True)

            for contour, area in valid:
                if len(detections) >= self.max_detections:
                    break

                x, y, bw, bh = cv2.boundingRect(contour)

                # Principal-axis orientation in image frame.
                # 0 = vertical line (top-to-bottom), clockwise positive,
                # normalized to [-pi/2, pi/2] (line direction is 180-deg ambiguous).
                if len(contour) >= 5:
                    [vx, vy, _, _] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
                    orientation_rad = float(math.atan2(float(vx), float(vy)))
                    if orientation_rad > math.pi / 2:
                        orientation_rad -= math.pi
                    elif orientation_rad < -math.pi / 2:
                        orientation_rad += math.pi
                else:
                    orientation_rad = 0.0

                det = ColorDetectionMsg()
                det.color_name = name
                det.confidence = min(float(area) / frame_area * 20.0, 1.0)
                det.bbox = [
                    float(x) / w,
                    float(y) / h,
                    float(x + bw) / w,
                    float(y + bh) / h,
                ]
                det.center_x = float(x + bw / 2) / w
                det.center_y = float(y + bh / 2) / h
                det.pixel_count = int(area)
                det.orientation_rad = orientation_rad
                detections.append(det)

                # Draw annotation on frame
                if self.publish_annotated:
                    bgr = DRAW_COLORS.get(name, DEFAULT_BGR)
                    cv2.rectangle(frame, (x, y), (x + bw, y + bh), bgr, 2)
                    label = f'{name} {det.confidence:.0%}'
                    cv2.putText(frame, label, (x, y - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, bgr, 2)

        # ── Publish detections ────────────────────────────────────
        det_msg = ColorDetectionArray()
        det_msg.header = msg.header
        det_msg.header.frame_id = msg.header.frame_id or 'camera'
        det_msg.detections = detections
        det_msg.timestamp = now
        self.detection_pub.publish(det_msg)

        if detections:
            self.detection_count += 1

        # ── Publish annotated image ──────────────────────────────
        if self.publish_annotated and self.annotated_pub is not None:
            # Draw status bar
            if detections:
                found = sorted(set(d.color_name for d in detections))
                status = 'DETECTED: ' + ', '.join(found)
                status_color = DRAW_COLORS.get(found[0], DEFAULT_BGR)
            else:
                status = 'Scanning...'
                status_color = (150, 150, 150)
            cv2.rectangle(frame, (0, 0), (w, 30), (15, 15, 15), -1)
            cv2.putText(frame, status, (8, 21),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2)

            _, buf = cv2.imencode('.jpg', frame,
                                  [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            img_msg = CompressedImage()
            img_msg.header = msg.header
            img_msg.format = 'jpeg'
            img_msg.data = buf.tobytes()
            self.annotated_pub.publish(img_msg)

        # Periodic stats logging
        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Processed {self.frame_count} frames, '
                f'{self.detection_count} with detections')


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
