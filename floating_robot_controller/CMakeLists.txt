cmake_minimum_required(VERSION 3.8)
project(floating_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Dependencies
# set dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  trajectory_msgs
  visualization_msgs
  tf2_eigen
  ament_index_cpp
  control_toolbox
  action_msgs
  ament_cmake
  spacedyn_ros
  builtin_interfaces
  floating_robot_interfaces
  geometry_msgs
  std_msgs
  yaml-cpp)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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

# Add executables
add_executable(end_effector_trajectory_controller
  src/trajectory.cpp
  src/end_effector_trajectory_controller.cpp)
add_executable(client_example
  src/client_example.cpp)
target_link_libraries(end_effector_trajectory_controller spacedyn_ros::spacedyn_ros)
target_link_libraries(client_example yaml-cpp)
ament_target_dependencies(end_effector_trajectory_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(client_example ${THIS_PACKAGE_INCLUDE_DEPENDS})

#include directories
target_include_directories(end_effector_trajectory_controller
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_include_directories(client_example
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  model
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install executables
install(TARGETS
  end_effector_trajectory_controller
  client_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
