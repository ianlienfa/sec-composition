#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DDSListener : public rclcpp::Node
{
public:
  DDSListener()
  : Node("dds_listener")
  {
    sub_ = this->create_subscription<std_msgs::msg::String>(
      "/shout", rclcpp::QoS(10),
      [this](std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DDSListener>());
  rclcpp::shutdown();
  return 0;
}