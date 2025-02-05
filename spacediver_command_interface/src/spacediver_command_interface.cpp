// Copyright (c) 2025 Tohoku Univ. Space Robotics Lab.
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

#include "spacediver_command_interface/spacediver_command_interface.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#define DEBUG_ENABLED false
namespace spacediver_command_interface
{

CommandInterface::CommandInterface(const rclcpp::NodeOptions& options)
  : rclcpp::Node("spacediver_command_interface", options)
{
  std::cout << "CommandInterface class is established." << std::endl;
}

CommandInterface::~CommandInterface()
{
  std::cout << "CommandInterface class is destructed." << std::endl;
}

}  // namespace spacediver_command_interface

RCLCPP_COMPONENTS_REGISTER_NODE(spacediver_command_interface::CommandInterface)
