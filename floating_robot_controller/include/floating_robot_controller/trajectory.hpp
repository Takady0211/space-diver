#ifndef floating_robot_controller_TRAJECTORY_HPP_
#define floating_robot_controller_TRAJECTORY_HPP_
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "floating_robot_interfaces/action/follow_end_effector_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace floating_robot_controller
{
using TrajectoryPointIter = std::vector<
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::iterator;
using TrajectoryPointConstIter = std::vector<
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>::const_iterator;

class Trajectory
{
public:
  Trajectory(
    const rclcpp::Time & current_time,
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    & current_point);

  /// Set the point before the trajectory message is replaced/appended
  /// Example: if we receive a new trajectory message and it's first point is
  /// 0.5 seconds from the current one, we call this function to log the current
  /// state, then append/replace the current trajectory
  void set_point_before_trajectory_msg(
    const rclcpp::Time & current_time,
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    & current_point);

  void
  update(
    floating_robot_interfaces::msg::EndEffectorTrajectory joint_trajectory,
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    & current_point);

  /// Find the segment (made up of 2 points) and its expected state from the
  /// containing trajectory.
  bool sample(
    const rclcpp::Time & sample_time, double & sec_to_point,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & output_state,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    & first_goal_state,
    TrajectoryPointConstIter & start_segment_itr,
    TrajectoryPointConstIter & end_segment_itr);

  /// Do interpolation between 2 states given a time in between their respective
  /// timestamps.
  void interpolate_between_points(
    const rclcpp::Time & time_a,
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & state_a,
    const rclcpp::Time & time_b,
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & state_b,
    const rclcpp::Time & sample_time,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & output);

  TrajectoryPointConstIter begin() const;
  TrajectoryPointConstIter end() const;

  floating_robot_interfaces::msg::EndEffectorTrajectory
  get_trajectory_msg() const
  {
    return trajectory_msg_;
  }

private:
  floating_robot_interfaces::msg::EndEffectorTrajectory trajectory_msg_;

  // Base time, which is always at the beginning of traject
  rclcpp::Time trajectory_start_time_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    trajectory_start_state_;

  // Controller initialized time
  rclcpp::Time time_before_traj_msg_;
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
    state_before_traj_mgs_;

  bool sampled_already_ = false;
}; // Trajectory
} // namespace floating_robot_controller

#endif
