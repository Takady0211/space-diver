cmake_minimum_required(VERSION 3.8)
project(spacediver_command_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(spacediver_command_interface src/spacediver_command_interface.cpp)
rclcpp_components_register_nodes(spacediver_command_interface "spacediver_command_interface::CommandInterface")

ament_auto_add_executable(spacediver_command_interface_node
  src/spacediver_command_interface_node.cpp
  src/spacediver_command_interface.cpp)

# gtest
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test_spacediver_command_interface test/unit_test_spacediver_command_interface.cpp)
  target_include_directories(unit_test_spacediver_command_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(unit_test_spacediver_command_interface spacediver_command_interface)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
