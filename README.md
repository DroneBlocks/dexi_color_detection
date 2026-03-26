# dexi_color_detection

HSV color detection ROS2 package for DEXI. Subscribes to compressed camera images and publishes color detections that can be consumed by Python scripts, Node-RED flows, or any ROS2 subscriber.

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cam0/image_raw/compressed` | `sensor_msgs/CompressedImage` | Subscribe | Camera input |
| `/color_detections` | `dexi_interfaces/ColorDetectionArray` | Publish | Detected colors with bounding boxes |
| `/color_detections/image/compressed` | `sensor_msgs/CompressedImage` | Publish | Annotated camera image |

## Quick Start

```bash
# Run the detection node (requires camera publishing on /cam0/image_raw/compressed)
ros2 launch dexi_color_detection color_detection_launch.py

# Or run the simulator (no camera needed, publishes fake detections)
ros2 launch dexi_color_detection color_detection_sim_launch.py

# Monitor detections
ros2 topic echo /color_detections
```

## ColorDetection Message

Each detection contains:

```
string color_name     # "red", "blue", "green", etc.
float32 confidence    # 0.0 to 1.0
float32[4] bbox       # [x1, y1, x2, y2] normalized 0-1
float32 center_x      # Center of detection, normalized
float32 center_y      # Center of detection, normalized
uint32 pixel_count    # Contour area in pixels
```

## Node-RED

With rosbridge running (`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`):

1. Add a **ros2-subscriber** node, set topic to `/color_detections`
2. Parse `msg.payload.detections[0].color_name` in a function node
3. Call `/dexi/led_service/set_led_ring_color` or trigger any other action

See `examples/node_red/color_to_led_flow.json` for an importable flow.

## Python Examples

```bash
# Set LED to match detected color
python3 examples/python/color_led_mission.py

# Print detections to console
python3 examples/python/color_subscriber.py

# Fly toward a colored target
python3 examples/python/color_flight_mission.py --color red
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `detection_frequency` | `5.0` | Hz — detection rate |
| `min_contour_area` | `500` | Minimum pixel area to count as detection |
| `max_detections` | `10` | Max detections per frame |
| `publish_annotated_image` | `true` | Publish annotated image feed |
| `colors.<name>.h_min/h_max` | varies | HSV hue range |
| `colors.<name>.s_min/s_max` | varies | HSV saturation range |
| `colors.<name>.v_min/v_max` | varies | HSV value range |
| `colors.<name>.enabled` | `true` | Enable/disable this color |

## Adding Custom Colors

Add to `config/color_detection_params.yaml`:

```yaml
colors:
  purple:
    h_min: 130
    h_max: 145
    s_min: 80
    s_max: 255
    v_min: 50
    v_max: 255
    enabled: true
```

Then launch with the config:

```bash
ros2 launch dexi_color_detection color_detection_launch.py \
    --ros-args --params-file config/color_detection_params.yaml
```

## Dependencies

- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `dexi_interfaces` (for `ColorDetection` / `ColorDetectionArray` messages)
- `python3-opencv` (`cv2`)
- `numpy`
