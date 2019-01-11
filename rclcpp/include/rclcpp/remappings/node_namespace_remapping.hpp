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

#ifndef RCLCPP__REMAPPINGS__NODE_NAMESPACE_REMAPPING_HPP_
#define RCLCPP__REMAPPINGS__NODE_NAMESPACE_REMAPPING_HPP_

#include "rclcpp/remapping.hpp"

namespace rclcpp
{
namespace remappings
{

/// Encapsulation of a node namespace remapping rule.
class NodeNamespaceRemapping : rclcpp::Remapping
{
public:
  /// Constructor for a "global" node namespace remapping rule.
  explicit
  NodeNamespaceRemapping(const std::string & new_node_namespace);
  /// Constructor for a node namespace remapping rule which applies to only one node.
  NodeNamespaceRemapping(const std::string & new_node_namespace, const std::string & node_name);
  /// Construct based on an existing rcl remap rule.
  explicit
  NodeNamespaceRemapping(const rcl_remap_t & rcl_remap_rule) : Remapping(rcl_remap_rule) {}

  virtual ~NodeNamespaceRemapping() = default;
};

}  // namespace remappings
}  // namespace rclcpp

#endif  // RCLCPP__REMAPPINGS__NODE_NAMESPACE_REMAPPING_HPP_
