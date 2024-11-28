cmake_minimum_required(VERSION 3.8)
project(spacediver_ros2_control)

# DYNAMIXEL
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(rclcpp REQUIRED) # DYNAMIXEL
find_package(pluginlib REQUIRED) # DYNAMIXEL
find_package(hardware_interface REQUIRED) # DYNAMIXEL
find_package(dynamixel_workbench_toolbox REQUIRED) # DYNAMIXEL
# find_package(rclcpp_lifecycle REQUIRED) # DYNAMIXEL

# DYNAMIXEL
## Add dynamixel hardware library
add_library(
  dynamixel_hardware
  SHARED
  src/dynamixel_hardware.cpp
)
target_include_directories(
  dynamixel_hardware
  PRIVATE
  include
)
ament_target_dependencies(
  dynamixel_hardware
  rclcpp
  hardware_interface
  pluginlib
  dynamixel_workbench_toolbox
  )
pluginlib_export_plugin_description_file(hardware_interface dynamixel_hardware.xml)

## Install rules
install(
  DIRECTORY 
    bringup
    description
    config # DYNAMIXEL
    launch # DYNAMIXEL
  DESTINATION share/${PROJECT_NAME}
)
# DYNAMIXEL
install(
  TARGETS dynamixel_hardware
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

## Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

## Exports
ament_export_include_directories(
  include
)
ament_export_libraries(
  dynamixel_hardware
)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  dynamixel_workbench_toolbox
)

ament_package()
