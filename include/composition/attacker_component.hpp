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

#ifndef COMPOSITION__ATTACKER_COMPONENT_HPP_
#define COMPOSITION__ATTACKER_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"


namespace composition
{

class Attacker : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit Attacker(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
};

}  // namespace composition

#endif  // COMPOSITION__ATTACKER_COMPONENT_HPP_
