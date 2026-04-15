// HSV color detection ROS2 node for DEXI (C++ port).
//
// Subscribes to compressed camera images, detects colors using configurable
// HSV ranges, and publishes results on /color_detections as a
// dexi_interfaces/ColorDetectionArray.
//
// This is a straight port of the original Python node
// (see git history for color_detection_node.py). It preserves the exact
// ROS interface contract: topic names, message types, parameter names,
// parameter defaults, and detection semantics. The motivation for the
// port is CPU load on the Raspberry Pi CM5, where the Python node was
// burning ~16% of a core and contributing to thermal throttling.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <dexi_interfaces/msg/color_detection.hpp>
#include <dexi_interfaces/msg/color_detection_array.hpp>

namespace dexi_color_detection
{

struct ColorConfig
{
  bool enabled{true};
  int h_min{0};
  int h_max{0};
  int s_min{0};
  int s_max{255};
  int v_min{0};
  int v_max{255};
  bool has_second_range{false};
  int h_min2{0};
  int h_max2{0};
};

// BGR colors for drawing annotations, matches the Python DRAW_COLORS dict.
static const std::map<std::string, cv::Scalar> kDrawColors = {
  {"red",    cv::Scalar(  0,   0, 255)},
  {"orange", cv::Scalar(  0, 140, 255)},
  {"yellow", cv::Scalar(  0, 220, 255)},
  {"green",  cv::Scalar(  0, 200,   0)},
  {"blue",   cv::Scalar(255,   0,   0)},
  {"pink",   cv::Scalar(200,   0, 200)},
};

static const cv::Scalar kDefaultBgr(200, 200, 200);

static cv::Scalar draw_color_for(const std::string & name)
{
  auto it = kDrawColors.find(name);
  return (it != kDrawColors.end()) ? it->second : kDefaultBgr;
}

class ColorDetectionNode : public rclcpp::Node
{
public:
  ColorDetectionNode()
  : rclcpp::Node("color_detection_node"),
    last_detection_time_(0.0),
    frame_count_(0),
    detection_count_(0)
  {
    // ── Top-level parameters ──────────────────────────────────────
    detection_frequency_ = this->declare_parameter<double>("detection_frequency", 5.0);
    min_contour_area_ = this->declare_parameter<int>("min_contour_area", 500);
    max_detections_ = this->declare_parameter<int>("max_detections", 10);
    publish_annotated_ = this->declare_parameter<bool>("publish_annotated_image", true);
    jpeg_quality_ = this->declare_parameter<int>("annotated_jpeg_quality", 75);

    min_detection_interval_ = 1.0 / std::max(detection_frequency_, 0.1);

    // ── Color definitions ─────────────────────────────────────────
    load_color_params();

    // ── Publishers ────────────────────────────────────────────────
    detection_pub_ = this->create_publisher<dexi_interfaces::msg::ColorDetectionArray>(
      "/color_detections", 10);

    if (publish_annotated_) {
      annotated_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/color_detections/image/compressed", 10);
    }

    // ── Subscriber ────────────────────────────────────────────────
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/cam0/image_raw/compressed", 10,
      std::bind(&ColorDetectionNode::image_callback, this, std::placeholders::_1));

    // ── Stats / info ──────────────────────────────────────────────
    std::string enabled_names;
    for (const auto & kv : colors_) {
      if (kv.second.enabled) {
        if (!enabled_names.empty()) {
          enabled_names += ", ";
        }
        enabled_names += kv.first;
      }
    }

    RCLCPP_INFO(this->get_logger(), "DEXI Color Detection node initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to: /cam0/image_raw/compressed");
    RCLCPP_INFO(this->get_logger(), "Publishing to:  /color_detections");
    if (publish_annotated_) {
      RCLCPP_INFO(this->get_logger(), "Annotated feed: /color_detections/image/compressed");
    }
    RCLCPP_INFO(this->get_logger(), "Detection freq: %.2f Hz", detection_frequency_);
    RCLCPP_INFO(this->get_logger(), "Min contour:    %d px", min_contour_area_);
    RCLCPP_INFO(this->get_logger(), "Colors enabled: %s", enabled_names.c_str());
  }

private:
  // Use a vector to preserve the insertion order used by the Python version.
  // The Python node iterated the colors dict in insertion order
  // (red, orange, yellow, green, blue, pink).
  std::vector<std::pair<std::string, ColorConfig>> colors_;

  double detection_frequency_;
  int min_contour_area_;
  int max_detections_;
  bool publish_annotated_;
  int jpeg_quality_;

  double min_detection_interval_;
  double last_detection_time_;
  uint64_t frame_count_;
  uint64_t detection_count_;

  rclcpp::Publisher<dexi_interfaces::msg::ColorDetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr annotated_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  void declare_color(
    const std::string & name,
    int h_min, int h_max, int s_min, int s_max,
    int v_min, int v_max, bool enabled,
    bool has_second_range = false, int h_min2 = 0, int h_max2 = 0)
  {
    const std::string prefix = "colors." + name + ".";

    ColorConfig cfg;
    cfg.h_min = this->declare_parameter<int>(prefix + "h_min", h_min);
    cfg.h_max = this->declare_parameter<int>(prefix + "h_max", h_max);
    cfg.s_min = this->declare_parameter<int>(prefix + "s_min", s_min);
    cfg.s_max = this->declare_parameter<int>(prefix + "s_max", s_max);
    cfg.v_min = this->declare_parameter<int>(prefix + "v_min", v_min);
    cfg.v_max = this->declare_parameter<int>(prefix + "v_max", v_max);
    cfg.enabled = this->declare_parameter<bool>(prefix + "enabled", enabled);

    if (has_second_range) {
      cfg.has_second_range = true;
      cfg.h_min2 = this->declare_parameter<int>(prefix + "h_min2", h_min2);
      cfg.h_max2 = this->declare_parameter<int>(prefix + "h_max2", h_max2);
    }

    colors_.emplace_back(name, cfg);
  }

  void load_color_params()
  {
    // Defaults match color_detection_node.py exactly.
    declare_color("red",      0,  10, 120, 255,  70, 255, true, /*has_second=*/true, 170, 179);
    declare_color("orange",  11,  25, 150, 255, 100, 255, true);
    declare_color("yellow",  26,  34, 120, 255, 100, 255, true);
    declare_color("green",   35,  85,  60, 255,  50, 255, true);
    declare_color("blue",    95, 130,  80, 255,  50, 255, true);
    declare_color("pink",   145, 169,  80, 255,  50, 255, true);
  }

  static double wall_time_sec()
  {
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    const double now = wall_time_sec();
    if (now - last_detection_time_ < min_detection_interval_) {
      return;
    }
    last_detection_time_ = now;
    frame_count_++;

    // Decode compressed image (OpenCV returns BGR).
    cv::Mat raw(1, static_cast<int>(msg->data.size()), CV_8UC1,
      const_cast<uint8_t *>(msg->data.data()));
    cv::Mat frame = cv::imdecode(raw, cv::IMREAD_COLOR);
    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image");
      return;
    }

    const int h = frame.rows;
    const int w = frame.cols;
    const double frame_area = static_cast<double>(h) * static_cast<double>(w);

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    dexi_interfaces::msg::ColorDetectionArray det_msg;
    det_msg.header = msg->header;
    if (det_msg.header.frame_id.empty()) {
      det_msg.header.frame_id = "camera";
    }
    det_msg.timestamp = now;

    std::vector<std::string> detected_names;  // for status bar (preserve order of append)

    for (const auto & kv : colors_) {
      const std::string & name = kv.first;
      const ColorConfig & cfg = kv.second;
      if (!cfg.enabled) {
        continue;
      }

      cv::Mat mask;
      cv::inRange(hsv,
        cv::Scalar(cfg.h_min, cfg.s_min, cfg.v_min),
        cv::Scalar(cfg.h_max, cfg.s_max, cfg.v_max),
        mask);

      if (cfg.has_second_range) {
        cv::Mat mask2;
        cv::inRange(hsv,
          cv::Scalar(cfg.h_min2, cfg.s_min, cfg.v_min),
          cv::Scalar(cfg.h_max2, cfg.s_max, cfg.v_max),
          mask2);
        cv::bitwise_or(mask, mask2, mask);
      }

      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      if (contours.empty()) {
        continue;
      }

      // Collect contours that meet the area threshold, sorted largest first.
      std::vector<std::pair<double, size_t>> valid;
      valid.reserve(contours.size());
      for (size_t i = 0; i < contours.size(); ++i) {
        const double area = cv::contourArea(contours[i]);
        if (area >= static_cast<double>(min_contour_area_)) {
          valid.emplace_back(area, i);
        }
      }
      std::sort(valid.begin(), valid.end(),
        [](const auto & a, const auto & b) { return a.first > b.first; });

      for (const auto & va : valid) {
        if (static_cast<int>(det_msg.detections.size()) >= max_detections_) {
          break;
        }

        const double area = va.first;
        const cv::Rect bbox = cv::boundingRect(contours[va.second]);

        dexi_interfaces::msg::ColorDetection det;
        det.color_name = name;
        det.confidence = static_cast<float>(std::min(area / frame_area * 20.0, 1.0));
        det.bbox = {
          static_cast<float>(bbox.x) / static_cast<float>(w),
          static_cast<float>(bbox.y) / static_cast<float>(h),
          static_cast<float>(bbox.x + bbox.width) / static_cast<float>(w),
          static_cast<float>(bbox.y + bbox.height) / static_cast<float>(h),
        };
        det.center_x = static_cast<float>(bbox.x + bbox.width / 2.0) / static_cast<float>(w);
        det.center_y = static_cast<float>(bbox.y + bbox.height / 2.0) / static_cast<float>(h);
        det.pixel_count = static_cast<uint32_t>(area);

        det_msg.detections.push_back(det);

        if (publish_annotated_) {
          const cv::Scalar color = draw_color_for(name);
          cv::rectangle(frame, bbox, color, 2);
          char label[64];
          std::snprintf(label, sizeof(label), "%s %d%%",
            name.c_str(), static_cast<int>(std::round(det.confidence * 100.0f)));
          cv::putText(frame, label, cv::Point(bbox.x, bbox.y - 8),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }

        detected_names.push_back(name);
      }
    }

    detection_pub_->publish(det_msg);

    if (!det_msg.detections.empty()) {
      detection_count_++;
    }

    // ── Publish annotated image ──────────────────────────────────
    if (publish_annotated_ && annotated_pub_) {
      std::string status;
      cv::Scalar status_color;

      if (!det_msg.detections.empty()) {
        // Sorted unique color names, matches Python's sorted(set(...)).
        std::vector<std::string> uniq = detected_names;
        std::sort(uniq.begin(), uniq.end());
        uniq.erase(std::unique(uniq.begin(), uniq.end()), uniq.end());

        status = "DETECTED: ";
        for (size_t i = 0; i < uniq.size(); ++i) {
          if (i != 0) {
            status += ", ";
          }
          status += uniq[i];
        }
        status_color = draw_color_for(uniq.front());
      } else {
        status = "Scanning...";
        status_color = cv::Scalar(150, 150, 150);
      }

      cv::rectangle(frame, cv::Point(0, 0), cv::Point(w, 30),
        cv::Scalar(15, 15, 15), -1);
      cv::putText(frame, status, cv::Point(8, 21),
        cv::FONT_HERSHEY_SIMPLEX, 0.65, status_color, 2);

      std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      std::vector<uint8_t> buf;
      if (cv::imencode(".jpg", frame, buf, encode_params)) {
        sensor_msgs::msg::CompressedImage img_msg;
        img_msg.header = msg->header;
        img_msg.format = "jpeg";
        img_msg.data = std::move(buf);
        annotated_pub_->publish(img_msg);
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to JPEG-encode annotated image");
      }
    }

    if (frame_count_ % 100 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Processed %lu frames, %lu with detections",
        static_cast<unsigned long>(frame_count_),
        static_cast<unsigned long>(detection_count_));
    }
  }
};

}  // namespace dexi_color_detection

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dexi_color_detection::ColorDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
