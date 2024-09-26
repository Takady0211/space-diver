#include "ament_index_cpp/get_package_share_directory.hpp"
#include "floating_robot_controller/end_effector_trajectory_controller.hpp"

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               floating_robot_controller::EndEffectorTrajectoryController>(
      "end_effector_trajectory_controller", "follow_end_effector_trajectory"));
  rclcpp::shutdown();
  return 0;
} // main()