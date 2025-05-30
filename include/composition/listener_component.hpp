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

#ifndef COMPOSITION__LISTENER_COMPONENT_HPP_
#define COMPOSITION__LISTENER_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace composition
{

class Listener : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options);

protected:
  void retransmit_message(std::string);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;

};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_
