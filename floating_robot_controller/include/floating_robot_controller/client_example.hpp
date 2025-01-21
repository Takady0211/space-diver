#ifndef REPEATED_ADMITTANCE__HPP
#define REPEATED_ADMITTANCE__HPP
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "floating_robot_interfaces/action/follow_end_effector_trajectory.hpp"
#include "floating_robot_interfaces/msg/end_effector_trajectory_point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace floating_robot_controller
{
// Child of end effector velocity controller
// But separate trajectory control server to other node

class EndEffectorTrajectoryClient : public rclcpp::Node
{
public:
  EndEffectorTrajectoryClient(
    const std::string & yaml_goal,
    const char * node_name);

protected:
  // // Main control function
  void timer_callback();

  // End effector trajectory controller client
  using FollowEndEffecTraject =
    floating_robot_interfaces::action::FollowEndEffectorTrajectory;
  using ClientGoalHandleEndEffecTraject =
    rclcpp_action::ClientGoalHandle<FollowEndEffecTraject>;
  void end_effector_traject_set_goal(const std::string & yaml_goal);
  void end_effector_traject_send_goal();
  floating_robot_interfaces::action::FollowEndEffectorTrajectory::Goal
    end_effec_trj_goal_msg_ = FollowEndEffecTraject::Goal();

  // End effector trajectory controller client
  rclcpp_action::Client<FollowEndEffecTraject>::SharedPtr
    end_effec_trj_ctrl_clnt_ptr_;
  void end_effector_traject_goal_response_callback(
    const ClientGoalHandleEndEffecTraject::SharedPtr & goal_handle);
  void end_effector_traject_feedback_callback(
    ClientGoalHandleEndEffecTraject::SharedPtr,
    const std::shared_ptr<const FollowEndEffecTraject::Feedback> feedback);
  void end_effector_traject_result_callback(
    const ClientGoalHandleEndEffecTraject::WrappedResult & result);

  // State publisher
  rclcpp::Publisher<
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::SharedPtr
    end_effector_goal_point_publisher_;
  rclcpp::Publisher<
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::SharedPtr
    end_effector_desired_point_publisher_;
  rclcpp::Publisher<
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::SharedPtr
    end_effector_actual_point_publisher_;
};
} // namespace floating_robot_controller

#endif
