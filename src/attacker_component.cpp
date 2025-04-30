#include "composition/attacker_component.hpp"

#include <iostream>
#include <memory>
#include <chrono>   // For std::chrono::seconds


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"


namespace composition
{

// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Attacker::Attacker(const rclcpp::NodeOptions & options)
: Node("attacker", options)
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  auto callback =
    [this](std_msgs::msg::Int32::ConstSharedPtr msg) -> void
    {
      (void)(msg);
      int* cur = (int*)(malloc(sizeof(int)));          
      free(cur);
      free(cur); // double free
      (void)(*cur++); // accessing that pointer    
    };

  // Create a subscription to the "chatter" topic which can be matched with one or more
  // compatible ROS publishers.
  // Note that not all publishers on the same topic with the same type will be compatible:
  // they must have compatible Quality of Service policies.
  sub_ = create_subscription<std_msgs::msg::Int32>("chatter", 10, callback);
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Attacker)
