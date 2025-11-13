#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

// Your existing message types
#include "orca_msgs/msg/motion.hpp"

using std::placeholders::_1;

class PoseFromArduSubNode : public rclcpp::Node {
public:
  PoseFromArduSubNode() : Node("pose_from_ardusub_node") {
    // Parameters (frame ids and topic selection)
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("odom_topic", "/mavros/local_position/odom"); // or /pose

    get_parameter("base_frame", base_frame_);
    get_parameter("odom_frame", odom_frame_);
    get_parameter("odom_topic", odom_topic_);

    motion_pub_ = create_publisher<orca_msgs::msg::Motion>("ardusub_pose_estimate", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Choose ONE of these subscribers. Default: nav_msgs/Odometry.
    if (odom_topic_.find("odom") != std::string::npos) {
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
          odom_topic_, rclcpp::QoS(10),
          std::bind(&PoseFromArduSubNode::odom_cb, this, _1));
      RCLCPP_INFO(get_logger(), "Subscribing to nav_msgs/Odometry: %s", odom_topic_.c_str());
    } else {
      pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          odom_topic_, rclcpp::QoS(10),
          std::bind(&PoseFromArduSubNode::pose_cb, this, _1));
      RCLCPP_INFO(get_logger(), "Subscribing to geometry_msgs/PoseStamped: %s", odom_topic_.c_str());
    }
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // MAVROS publishes ENU. Z is "up" (not depth). ArduSub’s EKF already fused depth sensor.
    const auto &p = msg->pose.pose.position;
    const auto &q = msg->pose.pose.orientation;

    // Republish in your custom message
    orca_msgs::msg::Motion motion;
    motion.header = msg->header;                   // keep EKF timestamp & frame (usually "map")
    motion.header.frame_id = odom_frame_;          // normalize to your chosen odom frame
    motion.pose.position.x = p.x;
    motion.pose.position.y = p.y;
    motion.pose.position.z = p.z;                  // keep ENU convention (z up)
    motion.pose.orientation = q;
    motion_pub_->publish(motion);

    // Broadcast TF: odom -> base_link
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = p.x;
    tf_msg.transform.translation.y = p.y;
    tf_msg.transform.translation.z = p.z;
    tf_msg.transform.rotation = q;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    const auto &p = msg->pose.position;
    const auto &q = msg->pose.orientation;

    orca_msgs::msg::Motion motion;
    motion.header = msg->header;
    motion.header.frame_id = odom_frame_;
    motion.pose.position.x = p.x;
    motion.pose.position.y = p.y;
    motion.pose.position.z = p.z;
    motion.pose.orientation = q;
    motion_pub_->publish(motion);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = p.x;
    tf_msg.transform.translation.y = p.y;
    tf_msg.transform.translation.z = p.z;
    tf_msg.transform.rotation = q;
    tf_broadcaster_->sendTransform(tf_msg);
  }

  // Params
  std::string base_frame_;
  std::string odom_frame_;
  std::string odom_topic_;

  // IO
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<orca_msgs::msg::Motion>::SharedPtr motion_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseFromArduSubNode>());
  rclcpp::shutdown();
  return 0;
}