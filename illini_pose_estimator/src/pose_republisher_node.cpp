#include "rclcpp/rclcpp.hpp"
#include "orca_msgs/msg/motion.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PoseRepublisherNode : public rclcpp::Node
{ // This node exists to convert the msg type into one that can be visualized in Rviz
public:
  PoseRepublisherNode()
  : Node("pose_republisher_node")
  {
    sub_ = create_subscription<orca_msgs::msg::Motion>(
      "/pose_estimate", 10,
      [this](const orca_msgs::msg::Motion::SharedPtr msg)
      {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose = msg->pose;
        pub_->publish(pose_msg);
      });

    pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose_estimate_visual", 10);
  }

private:
  rclcpp::Subscription<orca_msgs::msg::Motion>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseRepublisherNode>());
  rclcpp::shutdown();
  return 0;
}
