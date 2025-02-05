#include "floating_robot_controller/fixed_base_end_eff_traj_controller.hpp"

#include "floating_robot_controller/trajectory.hpp"
#include "floating_robot_interfaces/action/follow_end_effector_trajectory.hpp"
#include "spacedyn_ros/SpaceDyn"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <control_toolbox/pid.hpp>
#include <functional>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <visualization_msgs/msg/marker_array.hpp>

namespace floating_robot_controller
{
std::string package_share_directory =
  ament_index_cpp::get_package_share_directory("floating_robot_controller");

EndEffectorTrajectoryController::EndEffectorTrajectoryController(
  const char * node_name = "end_effector_trajectory_controller",
  const char * action_name = "end_effector_trajectory_control",
  const char * path_to_robot_model =
  (package_share_directory + "/model/example.urdf").c_str())
: Node(node_name), ros_clock_(RCL_ROS_TIME), robot_(path_to_robot_model)
{
  using namespace std::placeholders;

  // Parameter setting
  declare_parameter("update_rate", 1000);
  declare_parameter("use_closed_loop_pid_adapter", false);
  declare_parameter("point_follow_gain_ff", 1.0);
  declare_parameter("point_follow_gain_p", 0.0);
  declare_parameter("point_follow_gain_i", 0.0);
  declare_parameter("point_follow_gain_d", 0.0);
  declare_parameter("point_follow_gain_i_max", 0.0);
  declare_parameter("point_follow_gain_i_min", 0.0);
  declare_parameter("point_follow_gain_r", 0.0);

  dt_millisec_ = 1000 / get_parameter("update_rate").as_int();
  use_closed_loop_pid_adapter_ =
    get_parameter("use_closed_loop_pid_adapter").as_bool();
  point_follow_gain_ff_ = get_parameter("point_follow_gain_ff").as_double();
  point_follow_gain_r_ = get_parameter("point_follow_gain_r").as_double();

  executing_ = false;

  joint_number_ = robot_.getJointNumber();
  link_number_ = robot_.getLinkNumber();
  end_effector_number_ = robot_.getModel().getLinkage().getEndEffectorNumber();

  // Command
  pids_vector_.resize(end_effector_number_);
  joint_trajectory_publisher_ =
    this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "feedback_effort_controller/joint_trajectory", 10);
  joint_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "forward_velocity_controller/commands", 10);
  end_effector_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "end_effector_pose", 10);
  // State
  joint_state_.effort.resize(joint_number_);
  joint_state_.position.resize(joint_number_);
  joint_state_.velocity.resize(joint_number_);
  joint_state_subscriber_ =
    this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states/serial", 10,
    std::bind(
      &EndEffectorTrajectoryController::joint_state_callback,
      this, _1));
  odm_base_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odm/base", 10,
    std::bind(&EndEffectorTrajectoryController::odm_base_callback, this, _1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(dt_millisec_),
    std::bind(&EndEffectorTrajectoryController::timer_callback, this));

  // Action server
  this->action_server_ = rclcpp_action::create_server<FollowTraject>(
    this, action_name,
    std::bind(&EndEffectorTrajectoryController::handle_goal, this, _1, _2),
    std::bind(&EndEffectorTrajectoryController::handle_cancel, this, _1),
    std::bind(&EndEffectorTrajectoryController::handle_accepted, this, _1));

  // update robot before calculate end effector current point
  update_robot();
  traj_point_active_ptr_.resize(end_effector_number_);
  for (int e = 0; e < end_effector_number_; e++) {
    traj_point_active_ptr_.at(e) = std::make_shared<Trajectory>(
      this->now(), get_current_end_effector_point(e));
  }

  // Visualized state
  RCLCPP_INFO(
    this->get_logger(),
    "End effector trajectory controller started!");
  RCLCPP_INFO(this->get_logger(), "Action server name : %s \n", action_name);
  RCLCPP_INFO(this->get_logger(), " -----SpaceDyn model------ ");
  RCLCPP_INFO(
    this->get_logger(), "| Joint number        : %d |",
    joint_number_);
  RCLCPP_INFO(this->get_logger(), "| Link number         : %d |", link_number_);
  RCLCPP_INFO(
    this->get_logger(), "| End effector number : %d |",
    end_effector_number_);
  RCLCPP_INFO(this->get_logger(), " ------------------------- ");

  controller_start_time_ = this->now();
}

rclcpp_action::GoalResponse EndEffectorTrajectoryController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FollowTraject::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received new goal request");
  RCLCPP_INFO(this->get_logger(), "Cancel preexisting goal");
  update_robot();

  if (static_cast<int>(goal->trajectories.size()) != end_effector_number_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Goal trajectory should have the same number as end-effectors. ");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check if the goal is valid

  double workspace_radius = 0.2+0.2;

  for (size_t i = 0; i < goal->trajectories.size(); ++i) {
    const auto & trajectory = goal->trajectories[i];
    for (size_t j = 0; j < trajectory.points.size(); ++j) {
      const auto & point = trajectory.points[j];
      double magnitude = std::sqrt(
        std::pow(point.pose.position.x, 2) +
        std::pow(point.pose.position.y, 2) +
        std::pow(point.pose.position.z, 2));
      if (magnitude > workspace_radius) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Goal trajectory should be within the workspace radius.\n"
          "Workspace radius: %f\n"
          "Magnitude: %f\n"
          , workspace_radius, magnitude);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  }

  // For each end-effector
  for (int e = 0; e < end_effector_number_; e++) {
    // Initialize pid controller
    init_pids(pids_vector_.at(e));
    traj_point_active_ptr_.at(e)->update(
      goal->trajectories[e],
      get_current_end_effector_point(e));
  }

  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EndEffectorTrajectoryController::handle_cancel(
  const std::shared_ptr<ServerGoalHandleTraject> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EndEffectorTrajectoryController::handle_accepted(
  const std::shared_ptr<ServerGoalHandleTraject> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{std::bind(&EndEffectorTrajectoryController::execute, this, _1),
    goal_handle}
  .detach();
}

// This function is called every time a new goal is received
// This is the main function where the trajectory is executed
void EndEffectorTrajectoryController::execute(
  const std::shared_ptr<ServerGoalHandleTraject> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  // Initialize variables
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowTraject::Feedback>();
  auto result = std::make_shared<FollowTraject::Result>();

  feedback->desired_points.clear();
  feedback->actual_points.clear();

  feedback->desired_points.resize(end_effector_number_);
  feedback->actual_points.resize(end_effector_number_);

  bool trj_completed = false;
  while (!trj_completed) {
    executing_ = true;

    // If cancel requested, cancel trajectory
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // update robot state information
    update_robot();
    RCLCPP_INFO(this->get_logger(), "Robot state updated");

    // For each arm
    trj_completed = true;
    std::vector<geometry_msgs::msg::Twist> desired_twist(end_effector_number_);
    for (int e = 0; e < end_effector_number_; e++) {
      double sec_to_point;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint current_state;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint desired_state;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint goal_state;
      TrajectoryPointConstIter start_segment_itr;
      TrajectoryPointConstIter end_segment_itr;

      // Calculate desired point by interpolate
      traj_point_active_ptr_.at(e)->sample(
        this->now(), sec_to_point,
        desired_state, goal_state,
        start_segment_itr, end_segment_itr);

      RCLCPP_INFO(this->get_logger(), "Time to point: %f", sec_to_point);
      RCLCPP_INFO(this->get_logger(), "Desired state: [%f, %f, %f]", desired_state.pose.position.x, desired_state.pose.position.y, desired_state.pose.position.z);
      current_state = get_current_end_effector_point(e);
      RCLCPP_INFO(this->get_logger(), "Current pos: [%f, %f, %f]", current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z);
      // RCLCPP_INFO(this->get_logger(), "S Current joint angles: [%f, %f, %f]", joint_state_.position[0], joint_state_.position[1], joint_state_.position[2]);
      // RCLCPP_INFO(this->get_logger(), "S Current joint velocities: [%f, %f, %f]", joint_state_.velocity[0], joint_state_.velocity[1], joint_state_.velocity[2]);
      // std::vector<double> current_joint_angles = serial_to_parallel_joint_angles(joint_state_.position);
      // RCLCPP_INFO(this->get_logger(), "P Current joint angles: [%f, %f, %f]", current_joint_angles[0], current_joint_angles[1], current_joint_angles[2]);
      // std::vector<double> current_joint_velocities = serial_to_parallel_joint_velocities(joint_state_.velocity);
      // RCLCPP_INFO(this->get_logger(), "P Current joint velocities: [%f, %f, %f]", current_joint_velocities[0], current_joint_velocities[1], current_joint_velocities[2]);
      // RCLCPP_INFO(this->get_logger(), " ");

      // Calculate commanding end effector twist
      if (use_closed_loop_pid_adapter_) {
        // With feedback
        desired_twist.at(e) = compute_pids_command(
          pids_vector_.at(e), desired_state, current_state);
      } else {
        // Without feedback
        desired_twist.at(e) = desired_state.twist;
      }

      feedback->desired_points.at(e) = (desired_state);
      feedback->actual_points.at(e) = (current_state);

      // Break if all trajectory is completed
      if (end_segment_itr != traj_point_active_ptr_.at(e)->end()) {
        trj_completed = false;
      }
      // trj_completed = true;
    }

    // Calculate joint velocity
    auto joint_velocity = compute_joint_velocity(desired_twist);
    goal_handle->publish_feedback(feedback);
    publish_command(joint_velocity);
    // publish_command(joint_velocity);
    // joint_command_publisher_->publish(joint_velocity);
    // Log an empty line
    RCLCPP_INFO(this->get_logger(), "\n");
  }
  // std_msgs::msg::Float64MultiArray zero_velocity;
  // zero_velocity.data.resize(joint_number_, 0.0);
  // publish_command(zero_velocity);
  // joint_command_publisher_->publish(zero_velocity);

  executing_ = false;
  result->set__error_code(FollowTraject::Result::SUCCESSFUL);
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
}

void EndEffectorTrajectoryController::timer_callback() {}

geometry_msgs::msg::Pose
EndEffectorTrajectoryController::compute_end_effector_position(const int &id) {
  double th1;
  double th2;
  double th3;
  double link1 = 0.2;
  double link2 = 0.2;

  geometry_msgs::msg::Pose pose;
  
  if (id == 0) // left arm
  {
    th1 = joint_state_.position[0];
    th2 = joint_state_.position[1];
    th3 = joint_state_.position[2];
    pose.position.x = link1*cos(th2) + link2*cos(th2+th3);
    pose.position.y = ( link1*sin(th2) + link2*sin(th2+th3) )*cos(th1);
    pose.position.z = -( link1*sin(th2) + link2*sin(th2+th3) )*sin(th1);
  }
  else if (id == 1) // right arm
  {
    th1 = joint_state_.position[3];
    th2 = joint_state_.position[4];
    th3 = joint_state_.position[5];
    pose.position.x = link1*cos(th2) + link2*cos(th2+th3);
    pose.position.y = ( link1*sin(th2) + link2*sin(th2+th3) )*cos(th1);
    pose.position.z = ( link1*sin(th2) + link2*sin(th2+th3) )*sin(th1);
  }
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return pose;
}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_velocity(
  std::vector<geometry_msgs::msg::Twist> end_effector_velocities)
{
  std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>
  gen_jacob_for(end_effector_number_);

  // Solve constrained least squares TODO: Currently, just use inverse jacobian
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(joint_number_, joint_number_);
  Eigen::VectorXd ve = Eigen::VectorXd::Zero(joint_number_);

  double th1;
  double th2;
  double th3;
  double link1 = 0.2;
  double link2 = 0.2;
  double flip = 1.0;

  std_msgs::msg::Float64MultiArray joint_velocity_msg; // output

  for (int e = 0; e < end_effector_number_; e++) {
    gen_jacob_for.at(e) = robot_.computeGeneralizedJacobianForEndEffector(e);
    th1 = joint_state_.position[e*3+0];
    th2 = joint_state_.position[e*3+1];
    th3 = joint_state_.position[e*3+2];
    if (e == 1) {
      flip = -1.0;
    }

    gen_jacob_for.at(e)(0, 0) = 0;
    gen_jacob_for.at(e)(0, 1) = -link1 * sin(th2) - link2 * sin(th2 + th3);
    gen_jacob_for.at(e)(0, 2) = -link2 * sin(th2 + th3);
    gen_jacob_for.at(e)(1, 0) = -(link1 * sin(th2) + link2 * sin(th2 + th3)) * sin(th1);
    gen_jacob_for.at(e)(1, 1) = (link1 * cos(th2) + link2 * cos(th2 + th3)) * cos(th1);
    gen_jacob_for.at(e)(1, 2) = link2 * cos(th2 + th3) * cos(th1);
    gen_jacob_for.at(e)(2, 0) = -(link1 * sin(th2) + link2 * sin(th2 + th3)) * cos(th1) * flip;
    gen_jacob_for.at(e)(2, 1) = -(link1 * cos(th2) + link2 * cos(th2 + th3)) * sin(th1) * flip;
    gen_jacob_for.at(e)(2, 2) = -(link2 * cos(th2 + th3) * sin(th1)) * flip;

    // Select dimension to solve
    // Set manually for now
    J.row(0) = gen_jacob_for.at(e).row(0);
    J.row(1) = gen_jacob_for.at(e).row(1);
    J.row(2) = gen_jacob_for.at(e).row(2);
    // RCLCPP_INFO(this->get_logger(), "Jacobian matrix: \n[%f, %f, %f]\n[%f, %f, %f]\n[%f, %f, %f]",
    //       J(0, 0), J(0, 1), J(0, 2),
    //       J(1, 0), J(1, 1), J(1, 2),
    //       J(2, 0), J(2, 1), J(2, 2));

    ve(0) = end_effector_velocities.at(e).linear.x;
    ve(1) = end_effector_velocities.at(e).linear.y;
    ve(2) = end_effector_velocities.at(e).linear.z;

    Eigen::FullPivLU<Eigen::MatrixXd> LU(J);
    Eigen::VectorXd joint_velocity = LU.solve(ve);
    joint_velocity_msg.data.push_back(joint_velocity(0));
    joint_velocity_msg.data.push_back(joint_velocity(1));
    joint_velocity_msg.data.push_back(joint_velocity(2));
  }

  return joint_velocity_msg;
}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_effort(
  geometry_msgs::msg::Twist end_effector_velocity)
{
  // Mark parameter as unused to avoid colcon build warning
  (void)end_effector_velocity;

  std_msgs::msg::Float64MultiArray joint_effort;
  return joint_effort;
}

std::vector<double>
EndEffectorTrajectoryController::compute_next_position(
  std::vector<double> vel)
{
  std::vector<double> next_position;
  for (int i = 0; i < joint_number_; i++) {
    next_position.push_back(joint_state_.position[i] + vel[i] * (dt_millisec_/1000.0));
  }
  return next_position;
}

void EndEffectorTrajectoryController::publish_command(
  std_msgs::msg::Float64MultiArray command)
{

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = joint_state_.header.stamp;
  joint_trajectory.joint_names = joint_state_.name;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  int32_t dt_sec = dt_millisec_ / 1000;
  int32_t dt_nsec = (dt_millisec_ % 1000) * 1000000;
  point.time_from_start = rclcpp::Duration(dt_sec, dt_nsec);
  point.positions = serial_to_parallel_joint_angles(compute_next_position(command.data));
  point.velocities = serial_to_parallel_joint_velocities(command.data);
  RCLCPP_INFO(this->get_logger(), "Joint positions: [%f, %f, %f]", point.positions[0], point.positions[1], point.positions[2]);
  RCLCPP_INFO(this->get_logger(), "Joint velocities: [%f, %f, %f]", point.velocities[0], point.velocities[1], point.velocities[2]);
  joint_trajectory.points.push_back(point);
  joint_trajectory_publisher_->publish(joint_trajectory);
}

std::vector<double>
EndEffectorTrajectoryController::serial_to_parallel_joint_angles(
  std::vector<double> joint_angles)
{
  // convert to parallel chain joint angles
  joint_angles[0] = joint_angles[0];
  joint_angles[1] = joint_angles[1];
  joint_angles[2] = joint_angles[2] + joint_angles[1];
  
  // shift the joint angles by the starting position
  joint_angles[0] = joint_angles[0];
  joint_angles[1] = joint_angles[1] - (M_PI/2 + M_PI/6);
  joint_angles[2] = joint_angles[2];
  
  return joint_angles;
}

std::vector<double>
EndEffectorTrajectoryController::serial_to_parallel_joint_velocities(
  std::vector<double> joint_velocities)
{
  // convert to parallel chain joint velocities
  joint_velocities[0] = joint_velocities[0];
  joint_velocities[1] = joint_velocities[1];
  joint_velocities[2] = joint_velocities[2] + joint_velocities[1];

  return joint_velocities;
}

void EndEffectorTrajectoryController::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Update robot state information
  joint_state_ = *msg;
  std_msgs::msg::Float64MultiArray positions;
  for (int e = 0; e < end_effector_number_; e++) {
    geometry_msgs::msg::Pose pose = compute_end_effector_position(e);
    positions.data.push_back(pose.position.x);
    positions.data.push_back(pose.position.y);
    positions.data.push_back(pose.position.z);
  }
  end_effector_publisher_->publish(positions);
}

void EndEffectorTrajectoryController::odm_base_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Update robot state information
  odm_base_ = *msg;
}

void EndEffectorTrajectoryController::update_robot()
{
  // Update robot state information
  Eigen::Isometry3d pose =
    Eigen::Translation3d(
    odm_base_.pose.pose.position.x,
    odm_base_.pose.pose.position.y,
    odm_base_.pose.pose.position.z) *
    Eigen::Quaterniond(
    odm_base_.pose.pose.orientation.w, odm_base_.pose.pose.orientation.x,
    odm_base_.pose.pose.orientation.y, odm_base_.pose.pose.orientation.z);
  Eigen::VectorXd twist(6);
  twist << odm_base_.twist.twist.linear.x, odm_base_.twist.twist.linear.y,
    odm_base_.twist.twist.linear.z, odm_base_.twist.twist.angular.x,
    odm_base_.twist.twist.angular.y, odm_base_.twist.twist.angular.z;
  robot_.overwriteBasePoseInWorldFrame(pose);
  robot_.overwriteBaseTwistInWorldFrame(twist);
  robot_.overwriteJointState(joint_state_);
  robot_.setStateVariable(
    spacedyn_ros::Kinematics::computeForward(robot_, true, true, false));
}

floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
EndEffectorTrajectoryController::get_current_end_effector_point(int id)
{
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint point;
  auto link_id = robot_.getEndEffector(id).getId();
  // point.pose = tf2::toMsg(robot_.getLinkPoseInWorldFrame(link_id).getPoseInWorldFrame());
  point.pose = compute_end_effector_position(id);
  auto twist = robot_.getLinkState(link_id).getTwistInWorldFrame();
  point.twist.linear = tf2::toMsg2(twist.getLinearVelocity());
  point.twist.angular = tf2::toMsg2(twist.getAngularVelocity());
  return point;
}

void EndEffectorTrajectoryController::init_pids(
  std::vector<control_toolbox::Pid> & pids)
{
  const int dof_xyz = 3;

  pids.clear();

  control_toolbox::Pid pid;
  pid.initPid(
    get_parameter("point_follow_gain_p").as_double(),
    get_parameter("point_follow_gain_i").as_double(),
    get_parameter("point_follow_gain_d").as_double(),
    get_parameter("point_follow_gain_i_max").as_double(),
    get_parameter("point_follow_gain_i_min").as_double(), false);

  for (int i = 0; i < dof_xyz; i++) {
    pids.push_back(pid);
  }
}

geometry_msgs::msg::Twist
EndEffectorTrajectoryController::compute_pids_command(
  std::vector<control_toolbox::Pid> & pids,
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint des,
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint real)
{
  // TODO: Currently, using simple quaternion error to calculate orientation
  geometry_msgs::msg::Twist twist;

  const uint64_t dt = dt_millisec_ * 1000000; // Convert to nanosec

  Eigen::Quaterniond des_quat;
  Eigen::Quaterniond real_quat;
  tf2::fromMsg(des.pose.orientation, des_quat);
  tf2::fromMsg(real.pose.orientation, real_quat);
  Eigen::Quaterniond delta_quat = des_quat * real_quat.conjugate();

  Eigen::Vector3d axis = delta_quat.vec().normalized(); // Rotation axis
  double theta = real_quat.angularDistance(des_quat) *
    2;              // Cuz quaternion angle is half as rotation angle
  Eigen::Vector3d ang_vel = axis * theta / dt * 1000;

  twist.linear.x =
    des.twist.linear.x * point_follow_gain_ff_ +
    pids[0].computeCommand(des.pose.position.x - real.pose.position.x, dt);
  twist.linear.y =
    des.twist.linear.y * point_follow_gain_ff_ +
    pids[1].computeCommand(des.pose.position.y - real.pose.position.y, dt);
  twist.linear.z =
    des.twist.linear.z * point_follow_gain_ff_ +
    pids[2].computeCommand(des.pose.position.z - real.pose.position.z, dt);
  twist.angular.x = des.twist.angular.x * point_follow_gain_ff_ +
    ang_vel(0) * point_follow_gain_r_;
  twist.angular.y = des.twist.angular.y * point_follow_gain_ff_ +
    ang_vel(1) * point_follow_gain_r_;
  twist.angular.z = des.twist.angular.z * point_follow_gain_ff_ +
    ang_vel(2) * point_follow_gain_r_;
  return twist;
}

} // namespace floating_robot_controller

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  if (argc > 1) {
    rclcpp::spin(
      std::make_shared<
        floating_robot_controller::EndEffectorTrajectoryController>(
        "end_effector_trajectory_controller", "follow_end_effector_trajectory",
        argv[1]));
  } else {
    rclcpp::spin(
      std::make_shared<
        floating_robot_controller::EndEffectorTrajectoryController>(
        "end_effector_trajectory_controller",
        "follow_end_effector_trajectory"));
  }

  rclcpp::shutdown();
  return 0;
} // main()
