// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__NODE_OPTIONS_HPP_
#define RCLCPP__NODE_OPTIONS_HPP_

#include <map>
#include <string>
#include <vector>

#include "rclcpp/context.hpp"

namespace rclcpp
{

/// Encapsulation of options that can be passed to a Node on creation.
class NodeOptions
{
public:
  NodeOptions() = default;
  virtual ~NodeOptions() = default;

private:
  std::string name_;
  std::string namespace_;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Remappings remappings_;
  bool ignore_global_remappings_;
  std::vector<rclcpp::Parameter> initial_parameter_values_;
  bool use_intra_process_communication_;
  bool disable_parameter_service_;
};

}  // namespace rclcpp

#endif  // RCLCPP__NODE_OPTIONS_HPP_
