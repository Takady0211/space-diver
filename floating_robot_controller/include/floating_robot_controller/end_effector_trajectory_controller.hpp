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

namespace floating_robot_controller {
class EndEffectorTrajectoryController : public rclcpp::Node {
public:
  using FollowTraject =
      floating_robot_interfaces::action::FollowEndEffectorTrajectory;
  using ServerGoalHandleTraject =
      rclcpp_action::ServerGoalHandle<FollowTraject>;

  EndEffectorTrajectoryController(const char *node_name,
                                  const char *action_name);

private:
  // Basic functions

  // To compare with msg stamp, ros_clock has to use ros time, not system time
  int dt_millisec_;
  rclcpp::Time controller_start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time get_current_time();
  void timer_callback();

  // // Command interface
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
      joint_trajectory_publisher_;
  void publish_command(std_msgs::msg::Float64MultiArray msg);

  // // State interface
  // // // Joint state
  sensor_msgs::msg::JointState joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber_;
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // // // Base state
  nav_msgs::msg::Odometry odm_base_;
  nav_msgs::msg::Odometry odm_target_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odm_base_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odm_target_subscriber_;
  void odm_base_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odm_target_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // For control
  //
  geometry_msgs::msg::Twist desired_twist_;
  void initialize_joint_position(int duration_millisec);
  std_msgs::msg::Float64MultiArray
  compute_joint_effort(geometry_msgs::msg::Twist end_effector_velocity);
  std_msgs::msg::Float64MultiArray
  compute_joint_velocity(geometry_msgs::msg::Twist end_effector_velocity);
  void update_robot();
  Eigen::VectorXd solve_constrained_least_squares(Eigen::MatrixXd A,
                                                  Eigen::VectorXd b,
                                                  Eigen::MatrixXd C,
                                                  Eigen::VectorXd d);
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  get_current_end_effector_point(int end_effec_num);

  // 1.TRAJECTORY CONTROL BY ACTION INTERFACE
  // Class to calculate trajectory by interpolating
  // Arrange the trajectory in a way that it can be used for multiple arms
  std::shared_ptr<Trajectory> traj_point_active_ptr_ = nullptr;

  // Preallocate variables used in the realtime execute() function
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint state_current_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint command_current_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint state_desired_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint state_error_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint goal_state_;

  // Action handling
  rclcpp_action::Server<FollowTraject>::SharedPtr action_server_;
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
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
  std::vector<control_toolbox::Pid> pids_;
  void init_pids(std::vector<control_toolbox::Pid> &pids);
  double point_follow_gain_r_;
  double point_follow_gain_ff_;
  geometry_msgs::msg::Twist compute_pids_command(
      std::vector<control_toolbox::Pid> &pids,
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint des,
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint real);

  // 2.VELOCITY CONTROL BY TOPIC INTERFACE
  // // command interface
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
      end_effect_subbed_point_;
  rclcpp::Subscription<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::SharedPtr
      end_effect_point_subscriber_;
  void end_effect_point_callback(const floating_robot_interfaces::msg::
                                     EndEffectorTrajectoryPoint::SharedPtr msg);

  // Visualization
  // To see the end effector pose in rviz, publish as marker
  visualization_msgs::msg::MarkerArray current_pose_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      current_pose_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      desired_vel_markers_pub_;
  // To see the trajectory in rviz, publish as marker
  visualization_msgs::msg::MarkerArray desired_pose_markers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      desired_pose_markers_pub_;
  void set_traject_markers(
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint point);
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped
  point_to_tf(floating_robot_interfaces::msg::EndEffectorTrajectoryPoint point,
              std::string child_frame_id);
  geometry_msgs::msg::TransformStamped odm_to_tf(nav_msgs::msg::Odometry odm,
                                                 std::string child_frame_id);
  void publish_tfs();
}; // EndEffectorTrajectoryController
} // namespace floating_robot_controller

#endif // FLOATING_ROBOT_CONTROLLER_END_EFFECTOR_TRAJECTORY_CONTROLLER_HPP_