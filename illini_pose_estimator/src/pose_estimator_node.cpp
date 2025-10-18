// pose_estimator_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "orca_msgs/msg/motion.hpp"
#include "orca_shared/util.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using std::placeholders::_1;

class PoseEstimatorNode : public rclcpp::Node
{
public:
  PoseEstimatorNode()
      : Node("pose_estimator_node")
  {
    // Declare parameters (frame ids)
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("odom_frame", "odom");

    get_parameter("base_frame", base_frame_);
    get_parameter("odom_frame", odom_frame_);

    rclcpp::QoS qos(rclcpp::QoS(10).best_effort().durability_volatile());

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/mavros/imu/data", qos, std::bind(&PoseEstimatorNode::imu_callback, this, _1));

    // For barometer, update topic as needed
    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        "/mavros/imu/static_pressure", qos, std::bind(&PoseEstimatorNode::baro_callback, this, _1));

    // Publisher (Odometry or Pose)
    motion_pub_ = create_publisher<orca_msgs::msg::Motion>("pose_estimate", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Timer to run pose estimation at fixed rate
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PoseEstimatorNode::timer_callback, this));
    
    last_time_ = now();
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
    has_imu_ = true;
  }

  void baro_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    last_baro_ = *msg;
    has_baro_ = true;
  }

  void timer_callback()
  {
    if (!(has_imu_ && has_baro_)) return;
    rclcpp::Time now_time = now();
    double dt;
    if (first_update_) {
      dt = 0.0;
      first_update_ = false;
    } else {
      dt = (now_time - last_time_).seconds();
    }
    last_time_ = now_time;

    // Get orientation
    const auto &q = last_imu_.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 R(quat);

    // Body-frame acceleration
    tf2::Vector3 accel_body(
      last_imu_.linear_acceleration.x,
      last_imu_.linear_acceleration.y,
      last_imu_.linear_acceleration.z);

    // Rotate to world frame and subtract gravity
    // tf2::Vector3 accel_world = R * accel_body;

    // R * accel_body ==> applies (world-> body) rotation to body frame vector
    // should do (body -> world) instead?
    tf2::Vector3 accel_world = R.transpose() * accel_body;  

    // accel_world.setZ(accel_world.getZ() - 9.80665);
    tf2::Vector3 gravity_world(0.0, 0.0, 9.80665);
    accel_world = accel_world - gravity_world;


    //Smooth the acceleration values to avoid noise and make things more stable/consistent
    const double alpha = 0.17;
    smoothed_accel_.setX(alpha * accel_world.getX() + (1 - alpha) * smoothed_accel_.getX());
    smoothed_accel_.setY(alpha * accel_world.getY() + (1 - alpha) * smoothed_accel_.getY());
    smoothed_accel_.setZ(alpha * accel_world.getZ() + (1 - alpha) * smoothed_accel_.getZ());

    std::cout<<"accel_x1 : "<<accel_world.getX()<<std::endl;
    std::cout<<"accel_y1 : "<<accel_world.getY()<<std::endl;
    std::cout<<"accel_z1 : "<<accel_world.getZ()<<std::endl;
    //Ignore noisy acceleration values
    if (std::abs(accel_world.getX()) < 0.123) accel_world.setX(0.0);
    if (std::abs(accel_world.getY()) < 0.123) accel_world.setY(0.0);
    if (std::abs(accel_world.getZ()) < 0.123) accel_world.setZ(0.0);

    // Integrate velocity and position
    velocity_ += accel_world * dt;
    const double velo_decay = 0.9;
    velocity_ *= velo_decay;

    //Ignore noisy velocity values
    if (std::abs(velocity_.getX()) < 0.09) velocity_.setX(0.0);
    if (std::abs(velocity_.getY()) < 0.09) velocity_.setY(0.0);
    if (std::abs(velocity_.getZ()) < 0.09) velocity_.setZ(0.0);


    std::cout<<"accel_x2 : "<<accel_world.getX()<<std::endl;
    std::cout<<"accel_y2 : "<<accel_world.getY()<<std::endl;
    std::cout<<"accel_z2 : "<<accel_world.getZ()<<std::endl;
    std::cout<<"velo_x : "<<velocity_.getX()<<std::endl;
    std::cout<<"velo_y : "<<velocity_.getY()<<std::endl;
    std::cout<<"velo_z : "<<velocity_.getZ()<<std::endl;
    position_ += velocity_ * dt;
    std::cout<<"pos_x : "<<position_.getX()<<std::endl;
    std::cout<<"pos_y : "<<position_.getY()<<std::endl;
    std::cout<<"pos_z : "<<position_.getZ()<<std::endl;

    // Get depth from barometer
    double depth = compute_depth_from_pressure(last_baro_.fluid_pressure);

    geometry_msgs::msg::Pose pose;
    pose.position.x = position_.x();
    pose.position.y = position_.y();
    pose.position.z = -depth;  // Override Z with baro-derived depth
    pose.orientation = last_imu_.orientation;

    orca_msgs::msg::Motion motion;
    motion.header.stamp = now_time;
    motion.header.frame_id = odom_frame_;
    motion.pose = pose;

    motion_pub_->publish(motion);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now_time;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = pose.position.x;
    tf_msg.transform.translation.y = pose.position.y;
    tf_msg.transform.translation.z = pose.position.z;
    tf_msg.transform.rotation = pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  double compute_depth_from_pressure(double pressure_pa)
  {
    constexpr double P0 = 101325.0;
    constexpr double rho = 997.0;
    constexpr double g = 9.80665;
    return (pressure_pa - P0) / (rho * g);
  }

  std::string base_frame_;
  std::string odom_frame_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Publisher<orca_msgs::msg::Motion>::SharedPtr motion_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu last_imu_;
  sensor_msgs::msg::FluidPressure last_baro_;
  bool has_imu_ = false;
  bool has_baro_ = false;

  rclcpp::Time last_time_;
  bool first_update_ = true;

  tf2::Vector3 velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 position_{0.0, 0.0, 0.0};
  tf2::Vector3 smoothed_accel_{0.0, 0.0, 0.0};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
