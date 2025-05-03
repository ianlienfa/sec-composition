#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RemoteListener : public rclcpp::Node
{
public:
  RemoteListener() : Node("remote_listener")
  {
    auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "RemoteListener heard: '%s'", msg->data.c_str());
      auto pub_str = std::make_unique<std_msgs::msg::String>();
      pub_str->data = "Remote Publisher: " + std::to_string(++this->count_);
      pub_->publish(std::move(pub_str));
    };

    pub_ = create_publisher<std_msgs::msg::String>("remote", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/shout", 10, callback);
    this->count_ = 0;
  }

private:
  uint32_t count_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RemoteListener>());
  rclcpp::shutdown();
  return 0;
}