#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RemoteUnathorizedListener : public rclcpp::Node
{
public:
RemoteUnathorizedListener() : Node("remote_unauthorized_listener")
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "RemoteUnathorizedListener heard: '%s'", msg->data.c_str());
    };

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/shout", 10, callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteUnathorizedListener>());
  rclcpp::shutdown();
  return 0;
}