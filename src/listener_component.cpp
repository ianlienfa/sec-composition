// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "composition/listener_component.hpp"

#include <iostream>
#include <memory>
#include <chrono>   // For std::chrono::seconds
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

// Create a Listener "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
Listener::Listener(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
  // Create a callback function for when messages are received.
  // Variations of this function also exist using, for example, UniquePtr for zero-copy transport.
  auto callback =
    [this](std_msgs::msg::String::ConstSharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "Lisenter heard: [%s]", msg->data.c_str());            
      std::flush(std::cout);
      retransmit_message(msg->data);
    };

  // Create a subscription to the "chatter" topic which can be matched with one or more
  // compatible ROS publishers.
  // Note that not all publishers on the same topic with the same type will be compatible:
  // they must have compatible Quality of Service policies.
  sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, callback);
  pub_ = this->create_publisher<std_msgs::msg::String>("/shout", 10);
}

void Listener::retransmit_message(std::string s)
{
  std::string session_key = "SECRET";
  // block for processing
  auto msg = std::make_unique<std_msgs::msg::String>();  
  RCLCPP_INFO(this->get_logger(), "size of empty string: '%ld'", strlen(msg->data.c_str()));
  RCLCPP_INFO(this->get_logger(), "address of empty str: '%p'", msg->data.c_str());
  msg->data = session_key + " | " + s;
  RCLCPP_INFO(this->get_logger(), "Sending to remote: '%s'", msg->data.c_str());
  RCLCPP_INFO(this->get_logger(), "size of filled: '%ld'", strlen(msg->data.c_str()));
  RCLCPP_INFO(this->get_logger(), "address of filled str: '%p'", msg->data.c_str());
  std::flush(std::cout);

  // Put the message into a queue to be processed by the middleware.
  // This call is non-blocking.
  pub_->publish(std::move(msg));
}

}  // namespace composition

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(composition::Listener)
