#include "composition/attacker_component.hpp"



#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sys/mman.h>

namespace composition
{

// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Attacker::Attacker(const rclcpp::NodeOptions & options)
: Node("attacker", options), num_bytes_allocated_(0), kBytesAllocatedPerAttack_(1 << 14) // 16 MB per attack
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.  

  auto topic_list = this->get_topic_names_and_types();
  for (const auto & topic : topic_list) {    
    topic_name.resize(topic.first.size()-1);    
    copy(topic.first.begin()+1, topic.first.end(), topic_name.begin()); // slicing for topic name    
    RCLCPP_INFO(this->get_logger(), "Discovered topic: %s", topic_name.c_str()); 

    // define callback
    auto callback =
    [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
      RCLCPP_ERROR(this->get_logger(), "Attcker heard from topic[%s]: '%s'",topic_name.c_str(), msg->data.c_str());
    };

    try {
      sub_arr.push_back(create_subscription<std_msgs::msg::String>(topic_name.c_str(), 10, callback));
    }
    catch (...) {
      RCLCPP_INFO(this->get_logger(), "Unable to subscribe to topic[%s]",topic_name.c_str());
    }

    try {
      sub_arr.push_back(create_subscription<std_msgs::msg::String>("remote", 10, callback));
    }
    catch (...) {
      RCLCPP_INFO(this->get_logger(), "Unable to subscribe to topic[%s]","remote");
    }

  }
}


}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Attacker)
