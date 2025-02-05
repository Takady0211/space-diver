#ifndef FLOATING_ROBOT_CONTROLLER_END_EFFECTOR_TRAJECTORY_CONTROLLER_HPP_
#define FLOATING_ROBOT_CONTROLLER_END_EFFECTOR_TRAJECTORY_CONTROLLER_HPP_
#include "control_toolbox/pid.hpp"
#include "floating_robot_controller/trajectory.hpp"
#include "floating_robot_interfaces/action/follow_end_effector_trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spacedyn_ros/SpaceDyn"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <thread>

//  TODO: Change translator into tf2 from custom

namespace floating_robot_controller
{
class EndEffectorTrajectoryController : public rclcpp::Node
{
public:
  using FollowTraject =
    floating_robot_interfaces::action::FollowEndEffectorTrajectory;
  using ServerGoalHandleTraject =
    rclcpp_action::ServerGoalHandle<FollowTraject>;

  EndEffectorTrajectoryController(
    const char * node_name,
    const char * action_name,
    const char * path_to_robot_model);

private:
  // -------------------------- Basic functions --------------------------
  // To compare with msg stamp, ros_clock has to use ros time, not system time
  int dt_millisec_;
  rclcpp::Clock ros_clock_;
  rclcpp::Time now() {return ros_clock_.now();}
  rclcpp::Time controller_start_time_;
  rclcpp::Time get_current_time();
  void timer_callback();
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  int joint_number_;
  int link_number_;
  int end_effector_number_;

  // // Command
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    joint_trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_command_publisher_;
  void publish_command(std_msgs::msg::Float64MultiArray msg);

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr end_effector_publisher_;

  // // State
  // // // Joint state
  sensor_msgs::msg::JointState joint_state_;
  std::vector<double> serial_to_parallel_joint_angles(
    std::vector<double> joint_angles);
  std::vector<double> serial_to_parallel_joint_velocities(
    std::vector<double> joint_velocities);
  double ROT_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscriber_;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // // // Base state
  nav_msgs::msg::Odometry odm_base_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odm_base_subscriber_;
  void odm_base_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  // // // End effector pose
  geometry_msgs::msg::Pose compute_end_effector_position(const int &id);
  // ---------------------------------------------------------------------

  // ---------------------------- For control ----------------------------
  spacedyn_ros::Robot robot_; // SpaceDyn model

  std::vector<double> compute_next_position(
    std::vector<double> &vel);

  std_msgs::msg::Float64MultiArray
  compute_joint_effort(geometry_msgs::msg::Twist end_effector_velocity);
  std_msgs::msg::Float64MultiArray compute_joint_velocity(
    std::vector<geometry_msgs::msg::Twist> end_effector_velocities);
  void update_robot();
  //   Eigen::VectorXd solve_constrained_least_squares(Eigen::MatrixXd A,
  //                                                   Eigen::VectorXd b,
  //                                                   Eigen::MatrixXd C,
  //                                                   Eigen::VectorXd d);
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  get_current_end_effector_point(int end_effec_num);

  // TRAJECTORY CONTROL BY ACTION INTERFACE
  // Class to calculate trajectory by interpolating
  // Arrange the trajectory in a way that it can be used for multiple arms
  std::vector<std::shared_ptr<Trajectory>> traj_point_active_ptr_;

  // Preallocate variables used in the realtime execute() function
  // Action handling
  rclcpp_action::Server<FollowTraject>::SharedPtr action_server_;
  rclcpp_action::GoalResponse
  handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowTraject::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<ServerGoalHandleTraject> goal_handle);
  void
  handle_accepted(const std::shared_ptr<ServerGoalHandleTraject> goal_handle);
  bool executing_;
  void execute(const std::shared_ptr<ServerGoalHandleTraject> goal_handle);

  // // Used in closed loop end effector control
  /// If true, a velocity feedforward term plus corrective PID term is used
  bool use_closed_loop_pid_adapter_;
  std::vector<std::vector<control_toolbox::Pid>>
  pids_vector_;     // ee_num * joint_num
  double point_follow_gain_r_;
  double point_follow_gain_ff_;
  void init_pids(std::vector<control_toolbox::Pid> & pids);
  geometry_msgs::msg::Twist compute_pids_command(
    std::vector<control_toolbox::Pid> & pids,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint des,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint real);
  // ---------------------------------------------------------------------
}; // EndEffectorTrajectoryController
} // namespace floating_robot_controller

#endif // FLOATING_ROBOT_CONTROLLER_END_EFFECTOR_TRAJECTORY_CONTROLLER_HPP_
