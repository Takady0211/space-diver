#include "floating_robot_controller/trajectory.hpp"
#include "eigen3/Eigen/Dense"
#include "tf2_eigen/tf2_eigen.hpp"

namespace floating_robot_controller
{

Trajectory::Trajectory(
  const rclcpp::Time & current_time,
  const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  & current_point)
{
  trajectory_start_time_ = current_time;
  set_point_before_trajectory_msg(current_time, current_point);
}
// Trajectory

void Trajectory::set_point_before_trajectory_msg(
  const rclcpp::Time & current_time,
  const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  & current_point)
{
  time_before_traj_msg_ = current_time;
  state_before_traj_mgs_ = current_point;
} // set_point_before_trajectory_msg

void Trajectory::update(
  floating_robot_interfaces::msg::EndEffectorTrajectory joint_trajectory,
  const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  & current_point)
{
  trajectory_msg_ = joint_trajectory;
  auto current_time = static_cast<rclcpp::Time>(joint_trajectory.header.stamp);
  trajectory_start_time_ = current_time;
  trajectory_start_state_ = current_point;
  sampled_already_ = false;
} // update

bool Trajectory::sample(
  const rclcpp::Time & sample_time, double & sec_to_point,
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & output_state,
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
  & first_goal_state,
  TrajectoryPointConstIter & start_segment_itr,
  TrajectoryPointConstIter & end_segment_itr)
{
  if (trajectory_msg_.points.empty()) {
    start_segment_itr = end();
    end_segment_itr = end();
    output_state = trajectory_start_state_;
    sec_to_point = 0;
    return false;
  }

  // sampling before the current point
  if (sample_time < time_before_traj_msg_) {
    sec_to_point = 0;
    return false;
  }

  output_state = floating_robot_interfaces::msg::EndEffectorTrajectoryPoint();
  auto & first_point_in_msg = trajectory_msg_.points[0];
  const rclcpp::Time first_point_timestamp =
    trajectory_start_time_ + first_point_in_msg.time_from_start;

  // current time hasn't reached traj time of the first point in the msg yet
  if (sample_time < first_point_timestamp) {
    sec_to_point = first_point_timestamp.seconds() - sample_time.seconds();
    // If trajectory is is IGNORE, skip interpolation
    if (first_point_in_msg.path_generation_id ==
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::
      IGNORE_TRAJECTORY)
    {
      output_state = trajectory_start_state_;
    } else {
      interpolate_between_points(
        trajectory_start_time_,
        trajectory_start_state_, first_point_timestamp,
        first_point_in_msg, sample_time, output_state);
    }
    first_goal_state = first_point_in_msg;
    start_segment_itr = begin(); // no segments before the first
    end_segment_itr = begin();
    return true;
  }

  // time_from_start + trajectory time is the expected arrival time of
  // trajectory current time is between trajectory stamps
  const auto last_idx = trajectory_msg_.points.size() - 1;
  for (size_t i = 0; i < last_idx; ++i) {
    auto & point = trajectory_msg_.points[i];
    auto & next_point = trajectory_msg_.points[i + 1];

    const rclcpp::Time t0 = trajectory_start_time_ + point.time_from_start;
    const rclcpp::Time t1 = trajectory_start_time_ + next_point.time_from_start;

    // std::cout << "t1 - t0:" << t1.seconds()-t0.seconds() << std::endl;
    // std::cout << "t  - t0:" << sample_time.seconds()-t0.seconds() <<
    // std::endl;

    // current time is between point[i] and point[i+1]
    if (sample_time >= t0 && sample_time < t1) {
      sec_to_point = t1.seconds() - sample_time.seconds();
      // If interpolation is disabled, just forward the next waypoint
      if (first_point_in_msg.path_generation_id ==
        floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::
        IGNORE_TRAJECTORY)
      {
        output_state = next_point;
      } else {
        // Do interpolation
        // it changes points only if position and velocity do not exist, but
        // their derivatives
        interpolate_between_points(
          t0, point, t1, next_point, sample_time,
          output_state);
      }
      first_goal_state = next_point;
      start_segment_itr = begin() + i;
      end_segment_itr = begin() + (i + 1);
      return true;
    }
  }

  // whole animation has played out
  // std::cout << "Whole animation has played out" << std::endl;
  start_segment_itr = --end();
  end_segment_itr = end();
  output_state = (*start_segment_itr);
  if (&(output_state.twist) == nullptr) {
    output_state.twist.angular.x = 0;
    output_state.twist.angular.y = 0;
    output_state.twist.angular.z = 0;
    output_state.twist.linear.x = 0;
    output_state.twist.linear.y = 0;
    output_state.twist.linear.z = 0;
  }
  if (&(output_state.accel) == nullptr) {
    output_state.accel.angular.x = 0;
    output_state.accel.angular.y = 0;
    output_state.accel.angular.z = 0;
    output_state.accel.linear.x = 0;
    output_state.accel.linear.y = 0;
    output_state.accel.linear.z = 0;
  }
  // the trajectories in msg may have empty velocities/accel, so resize them
  // if (output_state.velocities.empty())
  // {
  //   output_state.velocities.resize(output_state.positions.size(), 0.0);
  // }
  // if (output_state.accelerations.empty())
  // {
  //   output_state.accelerations.resize(output_state.positions.size(), 0.0);
  // }
  return true;

} // sample

void Trajectory::interpolate_between_points(
  const rclcpp::Time & time_a,
  const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & state_a,
  const rclcpp::Time & time_b,
  const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & state_b,
  const rclcpp::Time & sample_time,
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint & output)
{
  // NOW: IGNORING TWIST AND ACCEL
  //
  //
  //

  rclcpp::Duration duration_so_far = sample_time - time_a;
  rclcpp::Duration duration_btwn_points = time_b - time_a;

  // Initialize output trajectory point
  output = floating_robot_interfaces::msg::EndEffectorTrajectoryPoint();

  // Initialize variables
  Eigen::Vector3d start_pos;
  Eigen::Vector3d end_pos;
  Eigen::Vector3d output_pos;
  Eigen::Vector3d output_vel;
  Eigen::Vector3d output_ang_vel;
  Eigen::Vector3d axis; // Rotation axis
  double theta;         // angle around rotation axis
  Eigen::Quaterniond start_ori;
  Eigen::Quaterniond end_ori;
  Eigen::Quaterniond delta_ori; // end_ori = delta_ori * start_ori
  Eigen::Quaterniond output_ori;

  // Convert message to eigen
  tf2::fromMsg(state_a.pose.position, start_pos);
  tf2::fromMsg(state_a.pose.orientation, start_ori);
  tf2::fromMsg(state_b.pose.position, end_pos);
  tf2::fromMsg(state_b.pose.orientation, end_ori);

  delta_ori = end_ori * start_ori.conjugate();
  axis = delta_ori.vec().normalized(); // Rotation axis
  theta = start_ori.angularDistance(end_ori) *
    2;       // Cuz quaternion angle is half as rotation angle

  // double path_len_btwn_points;
  double normed_path_so_far; // 0 at ta, 1 at tb
  double normed_vel;         // integral(normed_vel)=1

  if (duration_so_far.seconds() < 0.0) {
    duration_so_far = rclcpp::Duration::from_seconds(0.0);
  }
  if (duration_so_far.seconds() > duration_btwn_points.seconds()) {
    duration_so_far = duration_btwn_points;
  }

  // Calculate normalized velocity
  switch (state_b.time_scaling_id) {
    case floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::
      CONSTANT_TIME_SCALING:
      // Linier and angular velocities are all constant
      normed_vel = 1 / duration_btwn_points.seconds(); // [1/s]
      normed_path_so_far = duration_so_far.seconds() /
        duration_btwn_points.seconds();                  // 0 at start 1 at end
      break;

    case floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::
      POLY3_TIME_SCALING:
      // Polynomial time scaling 3 order
      normed_vel = 6 * duration_so_far.seconds() /
        std::pow(duration_btwn_points.seconds(), 2) -
        6 * std::pow(duration_so_far.seconds(), 2) /
        std::pow(
        duration_btwn_points.seconds(),
        3);                       // [1/s] 0 at start and end
      normed_path_so_far =
        3 * std::pow(
        duration_so_far.seconds() / duration_btwn_points.seconds(),
        2) -
        2 * std::pow(
        duration_so_far.seconds() / duration_btwn_points.seconds(),
        3);              // 0 at start 1 at end
      break;

    default:
      break;
  }

  // Calculate output state
  switch (state_b.path_generation_id) {
    case floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::
      STRAIGHT_LINE_PATH:
      // Ignore twist and accel this case
      // Interpolate position
      output_pos = start_pos + (end_pos - start_pos) * normed_path_so_far;
      output_vel = (end_pos - start_pos) * normed_vel;
      // Interpolate orientation
      output_ori = start_ori.slerp(
        normed_path_so_far, end_ori); // theta = omega * t, q(t)=a q_A + b q_B
      output_ang_vel = axis * normed_vel * theta;
      break;

    default:
      // Ignore trajectory, if not set path_generation_id or IGNORE_TRAJECTORY
      break;
  }

  output.pose.position = tf2::toMsg(output_pos);
  output.pose.orientation = tf2::toMsg(output_ori);
  output.twist.linear = tf2::toMsg2(output_vel);
  output.twist.angular = tf2::toMsg2(output_ang_vel);

} // interpolate_between_points

TrajectoryPointConstIter Trajectory::begin() const
{
  return trajectory_msg_.points.begin();
} // begin

TrajectoryPointConstIter Trajectory::end() const
{
  return trajectory_msg_.points.end();
} // end

} // namespace floating_robot_controller
