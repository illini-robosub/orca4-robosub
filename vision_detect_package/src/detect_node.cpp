// =============================================================================
// YOLO detection node
//
// Listens to the left color image from orca_zed_publisher, runs a YOLO model
// on it (TensorRT, on the GPU), draws a box around each detection and publishes:
//   ~/image       the left image with boxes drawn on it (bgr8)
//   ~/detections  every detection: class, confidence, box center (x, y) and size
//   ~/center      the box center (x, y) of the most confident detection
// All coordinates are in pixels of the left image (x = column, y = row).
//
// The model must be an .onnx file exported from Ultralytics, e.g.
//   yolo export model=best.pt format=onnx simplify=True dynamic=False imgsz=640
// (command from the ZED sample README). On first start the .onnx is turned
// into a TensorRT .engine file, which takes a few minutes and is then reused.
//
// API references:
//   - cv_bridge: /opt/ros/humble/include/cv_bridge/cv_bridge/cv_bridge.h
//   - Detection3D / DetectionArray: robosub_msgs/msg in this repo
//   - geometry_msgs/PointStamped: /opt/ros/humble/share/geometry_msgs/msg
// =============================================================================
#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <robosub_msgs/msg/detection3_d.hpp>
#include <robosub_msgs/msg/detection_array.hpp>

#include "vision_detect_package/yolo_detector.hpp"

using geometry_msgs::msg::PointStamped;
using robosub_msgs::msg::Detection3D;
using robosub_msgs::msg::DetectionArray;
using sensor_msgs::msg::Image;
namespace fs = std::filesystem;

class DetectNode : public rclcpp::Node
{
public:
  DetectNode()
  : Node("vision_detect")
  {
    // Default model: <install>/share/vision_detect_package/weights/best.onnx
    const std::string weights_dir =
      ament_index_cpp::get_package_share_directory("vision_detect_package") + "/weights/";

    // Settings (can be changed in config/detect_node.yaml)
    // An empty string means "use the default"
    onnx_path_ = declare_parameter<std::string>("onnx_path", "");
    if (onnx_path_.empty()) {onnx_path_ = weights_dir + "best.onnx";}
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    if (engine_path_.empty()) {  // same name as the .onnx, ending in .engine
      engine_path_ = fs::path(onnx_path_).replace_extension(".engine").string();
    }
    const std::string image_topic =
      declare_parameter<std::string>("image_topic", "/zed_publisher/left/image_rect_color");
    conf_threshold_ = declare_parameter<double>("conf_threshold", 0.5);
    nms_threshold_ = declare_parameter<double>("nms_threshold", 0.7);  // Ultralytics default
    fp16_ = declare_parameter<bool>("fp16", true);
    // Class names in training order. The .engine file doesn't store them,
    // so they have to be listed here (copy them from the dataset's data.yaml).
    class_names_ = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{});

    loadModel();

    // Keep only the newest frame. If YOLO is slower than the camera, old
    // frames get dropped instead of piling up and adding delay.
    // Best effort works with both reliable and best-effort publishers.
    image_sub_ = create_subscription<Image>(
      image_topic, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&DetectNode::imageCallback, this, std::placeholders::_1));

    // "~/" puts the topic under the node name, e.g. /vision_detect/image
    image_pub_ = create_publisher<Image>("~/image", 2);
    detections_pub_ = create_publisher<DetectionArray>("~/detections", 10);
    center_pub_ = create_publisher<PointStamped>("~/center", 10);

    RCLCPP_INFO(get_logger(), "Listening on %s", image_topic.c_str());
  }

private:
  // Load the TensorRT engine, building it from the .onnx first if needed
  void loadModel()
  {
    // Send TensorRT's messages to the ROS log
    auto log = [this](nvinfer1::ILogger::Severity s, const char * msg) {
        using S = nvinfer1::ILogger::Severity;
        if (s <= S::kERROR) {
          RCLCPP_ERROR(get_logger(), "[TensorRT] %s", msg);
        } else if (s == S::kWARNING) {
          RCLCPP_WARN(get_logger(), "[TensorRT] %s", msg);
        } else if (s == S::kINFO) {
          RCLCPP_DEBUG(get_logger(), "[TensorRT] %s", msg);  // very chatty, hidden by default
        }
      };
    auto detector = std::make_unique<vision_detect::YoloDetector>(log);

    // No engine yet: build one from the .onnx (only happens once)
    if (!fs::exists(engine_path_)) {
      if (!fs::exists(onnx_path_)) {
        RCLCPP_WARN(get_logger(), "No model at %s, publishing images without detections",
          onnx_path_.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Building TensorRT engine from %s. This takes a few minutes...",
        onnx_path_.c_str());
      if (!detector->buildEngine(onnx_path_, engine_path_, fp16_)) {
        RCLCPP_ERROR(get_logger(), "Engine build failed, publishing images without detections");
        return;
      }
      RCLCPP_INFO(get_logger(), "Saved engine to %s", engine_path_.c_str());
    }

    if (!detector->loadEngine(engine_path_)) {
      RCLCPP_ERROR(get_logger(), "Could not load %s (delete it to rebuild from the .onnx)",
        engine_path_.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "Loaded %s: input %dx%d, %d classes", engine_path_.c_str(),
      detector->inputWidth(), detector->inputHeight(), detector->numClasses());
    if (!class_names_.empty() &&
      static_cast<int>(class_names_.size()) != detector->numClasses())
    {
      RCLCPP_WARN(get_logger(), "class_names has %zu names but the model has %d classes",
        class_names_.size(), detector->numClasses());
    }
    detector_ = std::move(detector);
  }

  // Name of a class, or "class_<number>" if no name was given for it
  std::string className(int id) const
  {
    if (id >= 0 && id < static_cast<int>(class_names_.size())) {
      return class_names_[id];
    }
    return "class_" + std::to_string(id);
  }

  void imageCallback(const Image::ConstSharedPtr & msg)
  {
    // The ZED sends bgra8 (4 channels). Ask cv_bridge for bgr8 (3 channels),
    // which is what the detector and OpenCV drawing expect.
    cv::Mat img;
    try {
      img = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge: %s", e.what());
      return;
    }

    DetectionArray detections;
    detections.header = msg->header;  // same time stamp and frame as the camera image

    if (detector_) {
      for (const auto & d : detector_->detect(img, conf_threshold_, nms_threshold_)) {
        Detection3D det;
        det.header = msg->header;
        det.class_id = className(d.class_id);
        det.confidence = d.confidence;
        det.bbox_x = (d.x1 + d.x2) / 2.f;  // middle of the box
        det.bbox_y = (d.y1 + d.y2) / 2.f;
        det.bbox_width = d.x2 - d.x1;
        det.bbox_height = d.y2 - d.y1;
        det.depth = -1.f;  // not filled in yet (could come from the point cloud)
        det.depth_valid = false;
        detections.detections.push_back(det);
        draw(img, det);
      }
    }
    detections_pub_->publish(detections);

    // Center of the most confident detection, for nodes that want one target
    if (!detections.detections.empty()) {
      const auto & best = *std::max_element(
        detections.detections.begin(), detections.detections.end(),
        [](const Detection3D & a, const Detection3D & b) {return a.confidence < b.confidence;});
      PointStamped center;
      center.header = msg->header;
      center.point.x = best.bbox_x;
      center.point.y = best.bbox_y;
      center.point.z = 0.0;  // unused: this is a 2D pixel position
      center_pub_->publish(center);
    }

    // Image with boxes drawn on it (skip the copy if nobody is watching)
    if (image_pub_->get_subscription_count() > 0) {
      image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg());
    }
  }

  // Box, center dot and "label confidence (x, y)" text, all in green
  void draw(cv::Mat & img, const Detection3D & det)
  {
    const cv::Scalar green(0, 255, 0);  // OpenCV colors are (blue, green, red)
    const int cx = static_cast<int>(det.bbox_x), cy = static_cast<int>(det.bbox_y);
    const int x1 = static_cast<int>(det.bbox_x - det.bbox_width / 2);
    const int y1 = static_cast<int>(det.bbox_y - det.bbox_height / 2);
    cv::rectangle(img, cv::Rect(x1, y1, static_cast<int>(det.bbox_width),
      static_cast<int>(det.bbox_height)), green, 2);
    cv::circle(img, cv::Point(cx, cy), 4, green, cv::FILLED);
    char label[128];
    std::snprintf(label, sizeof(label), "%s %.2f (%d, %d)", det.class_id.c_str(),
      det.confidence, cx, cy);
    cv::putText(img, label, cv::Point(x1, std::max(15, y1 - 5)),  // keep text inside the image
      cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1, cv::LINE_AA);
  }

  // Settings
  std::string onnx_path_, engine_path_;
  double conf_threshold_, nms_threshold_;
  bool fp16_;
  std::vector<std::string> class_names_;

  std::unique_ptr<vision_detect::YoloDetector> detector_;  // null = no model loaded

  rclcpp::Subscription<Image>::SharedPtr image_sub_;
  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<DetectionArray>::SharedPtr detections_pub_;
  rclcpp::Publisher<PointStamped>::SharedPtr center_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DetectNode>());
  rclcpp::shutdown();
  return 0;
}
