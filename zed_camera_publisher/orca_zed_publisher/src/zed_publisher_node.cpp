// =============================================================================
// ZED camera publisher node
//
// Opens a ZED camera directly with the ZED SDK (no zed-ros2-wrapper needed)
// and publishes:
//   - an XYZ point cloud with a matching color for every point
//   - the left color image (same size as the point cloud, so pixels line up)
//   - depth image, confidence map, camera calibration info
//   - IMU, magnetometer, barometer and temperatures (if the camera has them)
//   - visual odometry (optional)
//
// API references: ZED SDK 5.0 header /usr/local/zed/include/sl/Camera.hpp and
// the official zed-ros2-wrapper (zed_camera_component.cpp).
// =============================================================================

#include <atomic>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <sl/Camera.hpp>  // ZED SDK

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// Shorter names for the message types we use a lot
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::FluidPressure;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;
using sensor_msgs::msg::Temperature;

namespace
{

// The ZED reports gyro data in degrees, ROS wants radians
constexpr double kDegToRad = M_PI / 180.0;

// Lets the user pick the resolution by name in the config file (e.g. "HD1200")
const std::map<std::string, sl::RESOLUTION> kResolutions = {
  {"HD4K", sl::RESOLUTION::HD4K}, {"QHDPLUS", sl::RESOLUTION::QHDPLUS},
  {"HD2K", sl::RESOLUTION::HD2K}, {"HD1080", sl::RESOLUTION::HD1080},
  {"HD1200", sl::RESOLUTION::HD1200}, {"HD1536", sl::RESOLUTION::HD1536},
  {"HD720", sl::RESOLUTION::HD720}, {"SVGA", sl::RESOLUTION::SVGA},
  {"VGA", sl::RESOLUTION::VGA}, {"AUTO", sl::RESOLUTION::AUTO},
};

// Lets the user pick the depth algorithm by name (e.g. "NEURAL_LIGHT")
const std::map<std::string, sl::DEPTH_MODE> kDepthModes = {
  {"PERFORMANCE", sl::DEPTH_MODE::PERFORMANCE}, {"QUALITY", sl::DEPTH_MODE::QUALITY},
  {"ULTRA", sl::DEPTH_MODE::ULTRA}, {"NEURAL_LIGHT", sl::DEPTH_MODE::NEURAL_LIGHT},
  {"NEURAL", sl::DEPTH_MODE::NEURAL}, {"NEURAL_PLUS", sl::DEPTH_MODE::NEURAL_PLUS},
};

// Look up a name in one of the tables above; give a clear error if it's misspelled
template<typename T>
T lookup(const std::map<std::string, T> & table, const std::string & key, const char * what)
{
  auto it = table.find(key);
  if (it == table.end()) {
    throw std::runtime_error(std::string("Unknown ") + what + ": '" + key + "'");
  }
  return it->second;
}

// Copy a ZED image/measure (sl::Mat) into a ROS message's byte buffer.
// We copy row by row because the ZED may add padding bytes at the end of each
// row, while ROS messages expect rows packed tightly together.
void copyRows(sl::Mat & mat, std::vector<uint8_t> & out, size_t row_bytes)
{
  const size_t h = mat.getHeight();
  const size_t src_step = mat.getStepBytes(sl::MEM::CPU);  // bytes per row incl. padding
  const uint8_t * src = mat.getPtr<sl::uchar1>(sl::MEM::CPU);
  out.resize(row_bytes * h);
  for (size_t r = 0; r < h; ++r) {
    std::memcpy(out.data() + r * row_bytes, src + r * src_step, row_bytes);
  }
}

}  // namespace

class ZedPublisherNode : public rclcpp::Node
{
public:
  ZedPublisherNode()
  : Node("zed_publisher")
  {
    declareParameters();        // read settings from the config file
    openCamera();               // connect to the ZED
    createPublishers();         // set up all the ROS topics
    publishStaticTransforms();  // tell TF how the camera frames relate

    // IMU and other sensors update faster than the images (up to ~400 Hz),
    // so they get their own timer instead of being tied to the image rate
    if (enable_sensors_) {
      auto period = std::chrono::duration<double>(1.0 / std::max(1.0, sensor_rate_));
      sensor_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ZedPublisherNode::publishSensors, this));
    }

    // zed_.grab() waits for each new frame, so run it in its own thread
    // so it never blocks the ROS timers above
    running_ = true;
    grab_thread_ = std::thread(&ZedPublisherNode::grabLoop, this);
  }

  ~ZedPublisherNode() override
  {
    // Stop the grab thread first, then release the camera
    running_ = false;
    if (grab_thread_.joinable()) {
      grab_thread_.join();
    }
    if (tracking_enabled_) {
      zed_.disablePositionalTracking();
    }
    zed_.close();
  }

private:
  // ---------------------------------------------------------------------------
  // Setup
  // ---------------------------------------------------------------------------

  // Read every setting (with a default value) from the ROS parameters
  void declareParameters()
  {
    // Which camera to open
    serial_number_ = declare_parameter<int>("serial_number", 0);     // 0 = first camera found
    svo_file_ = declare_parameter<std::string>("svo_file", "");      // play a recording instead

    // Camera / depth settings
    resolution_ = declare_parameter<std::string>("resolution", "AUTO");
    fps_ = declare_parameter<int>("fps", 30);
    depth_mode_ = declare_parameter<std::string>("depth_mode", "NEURAL_LIGHT");
    depth_min_ = declare_parameter<double>("depth_min", 0.3);        // meters
    depth_max_ = declare_parameter<double>("depth_max", 20.0);       // meters
    confidence_threshold_ = declare_parameter<int>("confidence_threshold", 95);
    texture_confidence_threshold_ = declare_parameter<int>("texture_confidence_threshold", 100);
    // Shrink the output by this factor (2 = half width and half height)
    downscale_ = std::max<int64_t>(1, declare_parameter<int>("downscale", 2));

    // Optional extras
    publish_right_ = declare_parameter<bool>("publish_right", false);
    enable_sensors_ = declare_parameter<bool>("enable_sensors", true);
    sensor_rate_ = declare_parameter<double>("sensor_rate", 200.0);  // Hz
    enable_tracking_ = declare_parameter<bool>("enable_tracking", false);
    publish_odom_tf_ = declare_parameter<bool>("publish_odom_tf", false);

    // TF frame names
    camera_frame_ = declare_parameter<std::string>("camera_frame", "zed_left_camera_frame");
    left_optical_frame_ =
      declare_parameter<std::string>("left_optical_frame", "zed_left_camera_optical_frame");
    right_optical_frame_ =
      declare_parameter<std::string>("right_optical_frame", "zed_right_camera_optical_frame");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "zed_odom");

    // true = "best effort" delivery (faster, may drop messages)
    sensor_qos_ = declare_parameter<bool>("sensor_qos", false);
  }

  // Connect to the ZED and read its calibration
  void openCamera()
  {
    sl::InitParameters init;
    init.camera_resolution = lookup(kResolutions, resolution_, "resolution");
    init.camera_fps = fps_;
    init.depth_mode = lookup(kDepthModes, depth_mode_, "depth_mode");
    init.coordinate_units = sl::UNIT::METER;
    // Use the standard ROS axes: x forward, y left, z up
    init.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init.depth_minimum_distance = static_cast<float>(depth_min_);
    init.depth_maximum_distance = static_cast<float>(depth_max_);
    if (!svo_file_.empty()) {
      init.input.setFromSVOFile(sl::String(svo_file_.c_str()));
    } else if (serial_number_ > 0) {
      init.input.setFromSerialNumber(static_cast<unsigned int>(serial_number_));
    }

    RCLCPP_INFO(get_logger(), "Opening ZED camera (%s, %s @ %d fps, depth %s)...",
      svo_file_.empty() ? "live" : svo_file_.c_str(), resolution_.c_str(), fps_,
      depth_mode_.c_str());
    sl::ERROR_CODE err = zed_.open(init);
    if (err != sl::ERROR_CODE::SUCCESS) {
      throw std::runtime_error(std::string("Failed to open ZED camera: ") +
              sl::toString(err).c_str());
    }

    // Settings applied on every grab(): pixels below these confidence
    // levels get no depth (they become NaN)
    runtime_.confidence_threshold = confidence_threshold_;
    runtime_.texture_confidence_threshold = texture_confidence_threshold_;

    // Work out the output size (camera size divided by downscale)
    auto info = zed_.getCameraInformation();
    const auto & full = info.camera_configuration.resolution;
    out_res_ = sl::Resolution(full.width / downscale_, full.height / downscale_);
    RCLCPP_INFO(get_logger(), "Opened %s (serial %u, firmware %u). Camera %dx%d, output %dx%d",
      sl::toString(info.camera_model).c_str(), info.serial_number,
      info.camera_configuration.firmware_version, full.width, full.height,
      out_res_.width, out_res_.height);

    // Get the lens calibration already scaled to our output size.
    // Images from the ZED are rectified, so there is no lens distortion left.
    auto calib = zed_.getCameraInformation(out_res_).camera_configuration.calibration_parameters;
    baseline_ = calib.getCameraBaseline();  // distance between left and right lens (m)
    left_info_ = makeCameraInfo(calib.left_cam, left_optical_frame_, 0.0);
    right_info_ = makeCameraInfo(calib.right_cam, right_optical_frame_, baseline_);

    // Visual odometry: the ZED tracks its own motion from images + IMU
    if (enable_tracking_) {
      sl::PositionalTrackingParameters tracking;
      err = zed_.enablePositionalTracking(tracking);
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Positional tracking failed to start: %s",
          sl::toString(err).c_str());
      } else {
        tracking_enabled_ = true;
      }
    }
  }

  // Build a ROS CameraInfo message (lens focal length, center, etc.)
  CameraInfo makeCameraInfo(
    const sl::CameraParameters & cam, const std::string & frame, double baseline)
  {
    CameraInfo ci;
    ci.header.frame_id = frame;
    ci.width = out_res_.width;
    ci.height = out_res_.height;
    ci.distortion_model = "plumb_bob";
    ci.d = {0.0, 0.0, 0.0, 0.0, 0.0};  // no distortion (already rectified)
    // K = intrinsic matrix: focal lengths (fx, fy) and image center (cx, cy)
    ci.k = {cam.fx, 0.0, cam.cx, 0.0, cam.fy, cam.cy, 0.0, 0.0, 1.0};
    // R = identity for rectified images
    ci.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    // P = projection matrix. For the right camera, Tx = -fx * baseline
    // (from the sensor_msgs/CameraInfo docs; left camera uses baseline 0)
    ci.p = {cam.fx, 0.0, cam.cx, -cam.fx * baseline, 0.0, cam.fy, cam.cy, 0.0, 0.0, 0.0, 1.0, 0.0};
    return ci;
  }

  // Create every ROS topic this node publishes.
  // "~/" puts the topic under the node's name, e.g. /zed_publisher/point_cloud
  void createPublishers()
  {
    // Keep only the 2 newest messages: big point clouds shouldn't pile up
    rclcpp::QoS qos = sensor_qos_ ? rclcpp::SensorDataQoS() : rclcpp::QoS(rclcpp::KeepLast(2));

    cloud_pub_ = create_publisher<PointCloud2>("~/point_cloud", qos);
    left_image_pub_ = create_publisher<Image>("~/left/image_rect_color", qos);
    left_info_pub_ = create_publisher<CameraInfo>("~/left/camera_info", qos);
    depth_pub_ = create_publisher<Image>("~/depth/depth_registered", qos);
    confidence_pub_ = create_publisher<Image>("~/confidence/confidence_map", qos);
    if (publish_right_) {
      right_image_pub_ = create_publisher<Image>("~/right/image_rect_color", qos);
      right_info_pub_ = create_publisher<CameraInfo>("~/right/camera_info", qos);
    }
    if (enable_sensors_) {
      imu_pub_ = create_publisher<Imu>("~/imu/data", qos);
      mag_pub_ = create_publisher<MagneticField>("~/imu/mag", qos);
      pressure_pub_ = create_publisher<FluidPressure>("~/atm_press", qos);
      temp_imu_pub_ = create_publisher<Temperature>("~/temperature/imu", qos);
      temp_left_pub_ = create_publisher<Temperature>("~/temperature/left", qos);
      temp_right_pub_ = create_publisher<Temperature>("~/temperature/right", qos);
    }
    if (tracking_enabled_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", qos);
      if (publish_odom_tf_) {
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      }
    }
  }

  // Images use the "optical" axes (z forward, x right, y down) while the point
  // cloud uses normal robot axes (x forward, y left, z up). These fixed
  // transforms tell TF/RViz how to convert between them.
  void publishStaticTransforms()
  {
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    auto make = [this](const std::string & child, double y_offset) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = camera_frame_;
        t.child_frame_id = child;
        t.transform.translation.y = y_offset;
        // Standard rotation from robot axes to optical axes (REP-103)
        t.transform.rotation.x = -0.5;
        t.transform.rotation.y = 0.5;
        t.transform.rotation.z = -0.5;
        t.transform.rotation.w = 0.5;
        return t;
      };
    // The right lens is one baseline to the right (negative y) of the left lens
    static_tf_->sendTransform({make(left_optical_frame_, 0.0),
        make(right_optical_frame_, -baseline_)});
  }

  // ---------------------------------------------------------------------------
  // Main loop: grab a frame, then publish everything from it
  // ---------------------------------------------------------------------------
  void grabLoop()
  {
    // Reused buffers so we don't reallocate memory every frame
    sl::Mat xyz, left, right, depth, confidence;

    while (running_ && rclcpp::ok()) {
      // Wait for the next frame and compute depth
      sl::ERROR_CODE err = zed_.grab(runtime_);
      if (err == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
        zed_.setSVOPosition(0);  // recording finished: restart it
        continue;
      }
      if (err != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "grab() failed: %s",
          sl::toString(err).c_str());
        continue;
      }

      // Stamp every message with the time the image was actually captured
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(
        static_cast<int64_t>(zed_.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds()));

      // For each topic: skip the work entirely if nobody is subscribed

      // Point cloud: x, y, z + color for every pixel
      if (cloud_pub_->get_subscription_count() > 0) {
        zed_.retrieveMeasure(xyz, sl::MEASURE::XYZBGRA, sl::MEM::CPU, out_res_);
        header.frame_id = camera_frame_;
        cloud_pub_->publish(toPointCloud(xyz, header));
      }

      // Everything below is an image, so it uses the optical frame
      header.frame_id = left_optical_frame_;

      // Color image (BGRA, 4 bytes per pixel). Same size as the point cloud,
      // so pixel (u, v) here is point (u, v) in the cloud.
      if (left_image_pub_->get_subscription_count() > 0) {
        zed_.retrieveImage(left, sl::VIEW::LEFT, sl::MEM::CPU, out_res_);
        left_image_pub_->publish(toImage(left, header, "bgra8", 4));
      }
      // Depth: distance in meters for each pixel (float, NaN = unknown)
      if (depth_pub_->get_subscription_count() > 0) {
        zed_.retrieveMeasure(depth, sl::MEASURE::DEPTH, sl::MEM::CPU, out_res_);
        depth_pub_->publish(toImage(depth, header, "32FC1", 4));
      }
      // Confidence: how unsure the depth is per pixel (1 = sure, 100 = unsure)
      if (confidence_pub_->get_subscription_count() > 0) {
        zed_.retrieveMeasure(confidence, sl::MEASURE::CONFIDENCE, sl::MEM::CPU, out_res_);
        confidence_pub_->publish(toImage(confidence, header, "32FC1", 4));
      }
      // Camera calibration, sent alongside every image
      left_info_.header = header;
      left_info_pub_->publish(left_info_);

      // Optional right camera image
      if (publish_right_) {
        header.frame_id = right_optical_frame_;
        if (right_image_pub_->get_subscription_count() > 0) {
          zed_.retrieveImage(right, sl::VIEW::RIGHT, sl::MEM::CPU, out_res_);
          right_image_pub_->publish(toImage(right, header, "bgra8", 4));
        }
        right_info_.header = header;
        right_info_pub_->publish(right_info_);
      }

      // Optional visual odometry
      if (tracking_enabled_) {
        publishOdometry(header.stamp);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Conversions from ZED data to ROS messages
  // ---------------------------------------------------------------------------

  // Point cloud layout (same as the official zed-ros2-wrapper):
  //   bytes 0-3: x, 4-7: y, 8-11: z (float meters), 12-15: color packed as "rgb"
  // It is "organized": height x width like an image, so it lines up with the
  // color image. Pixels without depth are NaN.
  std::unique_ptr<PointCloud2> toPointCloud(sl::Mat & xyz, const std_msgs::msg::Header & header)
  {
    auto msg = std::make_unique<PointCloud2>();
    msg->header = header;
    msg->height = xyz.getHeight();
    msg->width = xyz.getWidth();
    msg->is_bigendian = false;
    msg->is_dense = false;  // false because some points are NaN
    msg->point_step = 16;   // 4 floats per point
    msg->row_step = msg->point_step * msg->width;

    // Describe the 4 fields so RViz/PCL know how to read each point
    const char * names[] = {"x", "y", "z", "rgb"};
    for (uint32_t i = 0; i < 4; ++i) {
      PointField f;
      f.name = names[i];
      f.offset = 4 * i;
      f.datatype = PointField::FLOAT32;
      f.count = 1;
      msg->fields.push_back(f);
    }
    copyRows(xyz, msg->data, msg->row_step);
    return msg;
  }

  // Wrap a ZED image/measure in a ROS Image message
  std::unique_ptr<Image> toImage(
    sl::Mat & mat, const std_msgs::msg::Header & header, const std::string & encoding,
    uint32_t bytes_per_pixel)
  {
    auto msg = std::make_unique<Image>();
    msg->header = header;
    msg->height = mat.getHeight();
    msg->width = mat.getWidth();
    msg->encoding = encoding;
    msg->is_bigendian = false;
    msg->step = msg->width * bytes_per_pixel;  // bytes per row
    copyRows(mat, msg->data, msg->step);
    return msg;
  }

  // Publish where the camera thinks it is relative to where it started
  void publishOdometry(const rclcpp::Time & stamp)
  {
    sl::Pose pose;
    // Only publish when tracking is healthy
    if (zed_.getPosition(pose, sl::REFERENCE_FRAME::WORLD) != sl::POSITIONAL_TRACKING_STATE::OK) {
      return;
    }
    sl::Translation t = pose.getTranslation();  // position (m)
    sl::Orientation q = pose.getOrientation();  // rotation (quaternion)

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = camera_frame_;
    odom.pose.pose.position.x = t.tx;
    odom.pose.pose.position.y = t.ty;
    odom.pose.pose.position.z = t.tz;
    odom.pose.pose.orientation.x = q.ox;
    odom.pose.pose.orientation.y = q.oy;
    odom.pose.pose.orientation.z = q.oz;
    odom.pose.pose.orientation.w = q.ow;
    // Velocity: first 3 = linear, last 3 = angular
    odom.twist.twist.linear.x = pose.twist[0];
    odom.twist.twist.linear.y = pose.twist[1];
    odom.twist.twist.linear.z = pose.twist[2];
    odom.twist.twist.angular.x = pose.twist[3];
    odom.twist.twist.angular.y = pose.twist[4];
    odom.twist.twist.angular.z = pose.twist[5];
    // Uncertainty (6x6 matrices, same layout in the ZED SDK and ROS)
    for (size_t i = 0; i < 36; ++i) {
      odom.pose.covariance[i] = pose.pose_covariance[i];
      odom.twist.covariance[i] = pose.twist_covariance[i];
    }
    odom_pub_->publish(odom);

    // Optionally also publish it as a TF transform (odom -> camera)
    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = camera_frame_;
      tf.transform.translation.x = t.tx;
      tf.transform.translation.y = t.ty;
      tf.transform.translation.z = t.tz;
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ---------------------------------------------------------------------------
  // IMU / magnetometer / barometer / temperature (runs on its own timer)
  // Each one is only published if this camera model actually has it
  // (the ZED X has an IMU only; the ZED 2/2i also has mag + barometer).
  // ---------------------------------------------------------------------------
  void publishSensors()
  {
    sl::SensorsData sd;
    if (zed_.getSensorsData(sd, sl::TIME_REFERENCE::CURRENT) != sl::ERROR_CODE::SUCCESS) {
      return;
    }

    // IMU: only publish if this is a new reading (timestamp changed)
    if (sd.imu.is_available && sd.imu.timestamp.getNanoseconds() != last_imu_ns_) {
      last_imu_ns_ = sd.imu.timestamp.getNanoseconds();
      Imu imu;
      imu.header.stamp = rclcpp::Time(static_cast<int64_t>(last_imu_ns_));
      imu.header.frame_id = camera_frame_;
      // Orientation estimated by the ZED's onboard filter
      sl::Orientation q = sd.imu.pose.getOrientation();
      imu.orientation.x = q.ox;
      imu.orientation.y = q.oy;
      imu.orientation.z = q.oz;
      imu.orientation.w = q.ow;
      // Gyro: ZED gives deg/s, ROS wants rad/s
      imu.angular_velocity.x = sd.imu.angular_velocity.x * kDegToRad;
      imu.angular_velocity.y = sd.imu.angular_velocity.y * kDegToRad;
      imu.angular_velocity.z = sd.imu.angular_velocity.z * kDegToRad;
      // Accelerometer: already m/s^2
      imu.linear_acceleration.x = sd.imu.linear_acceleration.x;
      imu.linear_acceleration.y = sd.imu.linear_acceleration.y;
      imu.linear_acceleration.z = sd.imu.linear_acceleration.z;
      // Uncertainty (3x3). Degree-based ones get converted to radians,
      // the same way the official zed-ros2-wrapper does it.
      for (size_t i = 0; i < 9; ++i) {
        imu.orientation_covariance[i] = sd.imu.pose_covariance.r[i] * kDegToRad * kDegToRad;
        imu.angular_velocity_covariance[i] =
          sd.imu.angular_velocity_covariance.r[i] * kDegToRad * kDegToRad;
        imu.linear_acceleration_covariance[i] = sd.imu.linear_acceleration_covariance.r[i];
      }
      imu_pub_->publish(imu);

      // Temperatures (°C) at the IMU and next to each image sensor
      using Loc = sl::SensorsData::TemperatureData::SENSOR_LOCATION;
      publishTemperature(sd, Loc::IMU, temp_imu_pub_, imu.header);
      publishTemperature(sd, Loc::ONBOARD_LEFT, temp_left_pub_, imu.header);
      publishTemperature(sd, Loc::ONBOARD_RIGHT, temp_right_pub_, imu.header);
    }

    // Magnetometer (compass)
    if (sd.magnetometer.is_available &&
      sd.magnetometer.timestamp.getNanoseconds() != last_mag_ns_)
    {
      last_mag_ns_ = sd.magnetometer.timestamp.getNanoseconds();
      MagneticField mag;
      mag.header.stamp = rclcpp::Time(static_cast<int64_t>(last_mag_ns_));
      mag.header.frame_id = camera_frame_;
      // ZED gives microtesla, ROS wants tesla
      mag.magnetic_field.x = sd.magnetometer.magnetic_field_calibrated.x * 1e-6;
      mag.magnetic_field.y = sd.magnetometer.magnetic_field_calibrated.y * 1e-6;
      mag.magnetic_field.z = sd.magnetometer.magnetic_field_calibrated.z * 1e-6;
      mag_pub_->publish(mag);
    }

    // Barometer (air pressure inside the camera)
    if (sd.barometer.is_available && sd.barometer.timestamp.getNanoseconds() != last_baro_ns_) {
      last_baro_ns_ = sd.barometer.timestamp.getNanoseconds();
      FluidPressure p;
      p.header.stamp = rclcpp::Time(static_cast<int64_t>(last_baro_ns_));
      p.header.frame_id = camera_frame_;
      p.fluid_pressure = sd.barometer.pressure * 100.0;  // hPa -> Pa
      pressure_pub_->publish(p);
    }
  }

  // Publish one temperature reading if the camera has a sensor at that spot
  void publishTemperature(
    sl::SensorsData & sd, sl::SensorsData::TemperatureData::SENSOR_LOCATION loc,
    const rclcpp::Publisher<Temperature>::SharedPtr & pub, const std_msgs::msg::Header & header)
  {
    float celsius = 0.0f;
    if (sd.temperature.get(loc, celsius) != sl::ERROR_CODE::SUCCESS || std::isnan(celsius)) {
      return;  // no sensor here
    }
    Temperature t;
    t.header = header;
    t.temperature = celsius;
    pub->publish(t);
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  // Settings (from parameters)
  int64_t serial_number_;
  std::string svo_file_, resolution_, depth_mode_;
  int fps_;
  double depth_min_, depth_max_;
  int confidence_threshold_, texture_confidence_threshold_;
  int64_t downscale_;
  bool publish_right_, enable_sensors_, enable_tracking_, publish_odom_tf_, sensor_qos_;
  double sensor_rate_;
  std::string camera_frame_, left_optical_frame_, right_optical_frame_, odom_frame_;

  // Camera state
  sl::Camera zed_;
  sl::RuntimeParameters runtime_;
  sl::Resolution out_res_;          // size of everything we publish
  float baseline_ = 0.0f;           // left-to-right lens distance (m)
  bool tracking_enabled_ = false;
  CameraInfo left_info_, right_info_;
  uint64_t last_imu_ns_ = 0, last_mag_ns_ = 0, last_baro_ns_ = 0;  // skip repeat readings

  // Threads / timers
  std::atomic<bool> running_{false};
  std::thread grab_thread_;
  rclcpp::TimerBase::SharedPtr sensor_timer_;

  // Publishers
  rclcpp::Publisher<PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Image>::SharedPtr left_image_pub_, right_image_pub_, depth_pub_,
    confidence_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr left_info_pub_, right_info_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<Temperature>::SharedPtr temp_imu_pub_, temp_left_pub_, temp_right_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    // Runs until Ctrl+C; the grab thread does the camera work
    rclcpp::spin(std::make_shared<ZedPublisherNode>());
  } catch (const std::exception & e) {
    // e.g. camera not plugged in, or a misspelled parameter
    RCLCPP_FATAL(rclcpp::get_logger("zed_publisher"), "%s", e.what());
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}
