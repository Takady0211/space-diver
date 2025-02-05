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

#ifndef SPACEDIVER_COMMAND_INTERFACE__SPACEDIVER_COMMAND_INTERFACE_HPP_
#define SPACEDIVER_COMMAND_INTERFACE__SPACEDIVER_COMMAND_INTERFACE_HPP_

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "spacediver_command_interface/visibility_control.h"

namespace spacediver_command_interface
{

class CommandInterface : public rclcpp::Node
{
public:
  SPACEDIVER_COMMAND_INTERFACE_PUBLIC
  explicit CommandInterface(const rclcpp::NodeOptions& options);

  virtual ~CommandInterface();
};

}  // namespace spacediver_command_interface

#endif  // SPACEDIVER_COMMAND_INTERFACE__SPACEDIVER_COMMAND_INTERFACE_HPP_
