#include "floating_robot_controller/end_effector_trajectory_controller.hpp"

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

namespace floating_robot_controller {
std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("floating_robot_controller");

EndEffectorTrajectoryController::EndEffectorTrajectoryController(
    const char *node_name = "end_effector_trajectory_controller",
    const char *action_name = "end_effector_trajectory_control",
    const char *path_to_robot_model =
        (package_share_directory + "/model/example.urdf").c_str())
    : Node(node_name), robot_(path_to_robot_model) {
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
  // State
  joint_state_.effort.resize(joint_number_);
  joint_state_.position.resize(joint_number_);
  joint_state_.velocity.resize(joint_number_);
  joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 10,
          std::bind(&EndEffectorTrajectoryController::joint_state_callback,
                    this, _1));
  odm_base_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odm/base", 10,
      std::bind(&EndEffectorTrajectoryController::odm_base_callback, this, _1));

  // Action server
  this->action_server_ = rclcpp_action::create_server<FollowTraject>(
      this, action_name,
      std::bind(&EndEffectorTrajectoryController::handle_goal, this, _1, _2),
      std::bind(&EndEffectorTrajectoryController::handle_cancel, this, _1),
      std::bind(&EndEffectorTrajectoryController::handle_accepted, this, _1));

  // Topic server
  end_effect_point_subscriber_ = this->create_subscription<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
      "end_effector_trajectory_controller/goal", 10,
      std::bind(&EndEffectorTrajectoryController::end_effect_point_callback,
                this, _1));

  // update robot before calculate end effector current point
  update_robot();
  traj_point_active_ptr_.resize(end_effector_number_);
  for (int e = 0; e < end_effector_number_; e++) {
    traj_point_active_ptr_.at(e) = std::make_shared<Trajectory>(
        this->now(), get_current_end_effector_point(e));
  }

  // Visualized state
  RCLCPP_INFO(this->get_logger(),
              "End effector trajectory controller started!");
  RCLCPP_INFO(this->get_logger(), "Action server name : %s \n", action_name);
  RCLCPP_INFO(this->get_logger(), " -----SpaceDyn model------ ");
  RCLCPP_INFO(this->get_logger(), "| Joint number        : %d |",
              joint_number_);
  RCLCPP_INFO(this->get_logger(), "| Link number         : %d |", link_number_);
  RCLCPP_INFO(this->get_logger(), "| End effector number : %d |",
              end_effector_number_);
  RCLCPP_INFO(this->get_logger(), " ------------------------- ");

  controller_start_time_ = this->now();
}

rclcpp_action::GoalResponse EndEffectorTrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowTraject::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received new goal request");
  RCLCPP_INFO(this->get_logger(), "Cancel preexisting goal");
  update_robot();

  if (goal->trajectories.size() != end_effector_number_) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Goal trajectory should have the same number as end-effectors. ");
  }

  // For each end-effector
  for (int e = 0; e < end_effector_number_; e++) {
    // Initialize pid controller
    init_pids(pids_vector_.at(e));
    traj_point_active_ptr_.at(e)->update(goal->trajectories[e],
                                         get_current_end_effector_point(e));
  }

  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse EndEffectorTrajectoryController::handle_cancel(
    const std::shared_ptr<ServerGoalHandleTraject> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void EndEffectorTrajectoryController::handle_accepted(
    const std::shared_ptr<ServerGoalHandleTraject> goal_handle) {
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
    const std::shared_ptr<ServerGoalHandleTraject> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  // Initialize variables
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowTraject::Feedback>();
  auto result = std::make_shared<FollowTraject::Result>();

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
    std::vector<geometry_msgs::msg::Twist> commanded_twist(
        end_effector_number_);
    for (int e = 0; e < end_effector_number_; e++) {
      double sec_to_point;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint current_state;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint desired_state;
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint goal_state;
      TrajectoryPointConstIter start_segment_itr;
      TrajectoryPointConstIter end_segment_itr;

      // Calculate desired point by interpolate
      traj_point_active_ptr_.at(e)->sample(this->now(), sec_to_point,
                                           desired_state, goal_state,
                                           start_segment_itr, end_segment_itr);

      RCLCPP_INFO(this->get_logger(), "Time to point: %f", sec_to_point);

      // Calculate commanding end effector twist
      if (use_closed_loop_pid_adapter_) {
        // With feedback
        current_state = get_current_end_effector_point(e);
        commanded_twist.at(e) = compute_pids_command(
            pids_vector_.at(e), desired_state, current_state);
      } else {
        // Without feedback
        commanded_twist.at(e) = desired_state.twist;
      }

      feedback->desired_points.push_back(desired_state);
      feedback->actual_points.push_back(current_state);

      goal_handle->publish_feedback(feedback);
      // Break if all trajectory is completed
      if (end_segment_itr != traj_point_active_ptr_.at(e)->end()) {
        trj_completed = false;
      }
    }
  }

  executing_ = false;
  result->set__error_code(FollowTraject::Result::SUCCESSFUL);
  goal_handle->succeed(result);
}

void EndEffectorTrajectoryController::end_effect_point_callback(
    const floating_robot_interfaces::msg::EndEffectorTrajectoryPoint::SharedPtr
        msg) {
  // Update robot state information
  update_robot();
  // Calculate desired point by interpolate
  auto joint_velocity = compute_joint_velocity(msg->twist);
  publish_command(joint_velocity);
  return;
}

void EndEffectorTrajectoryController ::timer_callback() {}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_velocity(
    geometry_msgs::msg::Twist end_effector_velocity) {
  // Compute joint velocity
  // auto GJ = robot_.computeGeneralizedJacobianForEndEffector(-1);
  auto GJ = robot_.computeGeneralizedJacobianForEndEffector(-1);
  // Eigen::VectorXd end_effector_velocity_vec(6);
  // end_effector_velocity_vec << end_effector_velocity.linear.x,
  //     end_effector_velocity.linear.y, end_effector_velocity.linear.z,
  //     end_effector_velocity.angular.x, end_effector_velocity.angular.y,
  //     end_effector_velocity.angular.z;
  // // calc joint velocity
  // Eigen::FullPivLU<Eigen::MatrixXd> LU(GJ);
  // Eigen::VectorXd des_joint_vel = LU.solve(end_effector_velocity_vec);
  std_msgs::msg::Float64MultiArray joint_velocity;
  // for (int i = 0; i < robot_.getJointNumber(); i++) {
  //   joint_velocity.data.push_back(des_joint_vel(i));
  // }
  return joint_velocity;
}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_effort(
    geometry_msgs::msg::Twist end_effector_velocity) {
  // Compute joint effort
  // auto GJ = robot_.computeGeneralizedJacobianForEndEffector(-1);
  // Eigen::VectorXd end_effector_velocity_vec(6);
  // end_effector_velocity_vec << end_effector_velocity.linear.x,
  //     end_effector_velocity.linear.y, end_effector_velocity.linear.z,
  //     end_effector_velocity.angular.x, end_effector_velocity.angular.y,
  //     end_effector_velocity.angular.z;
  // // calc joint velocity
  // Eigen::FullPivLU<Eigen::MatrixXd> LU(GJ);
  // Eigen::VectorXd des_joint_vel = LU.solve(end_effector_velocity_vec);
  // Eigen::VectorXd current_joint_vel = robot_.getJointVelocity();
  // Eigen::VectorXd des_joint_acc =
  //     (des_joint_vel - current_joint_vel) / dt_millisec_ * 1000;
  // des_joint_acc = Eigen::VectorXd::Zero(robot_.getJointNumber());
  // des_joint_acc(0) = 0.001;
  std_msgs::msg::Float64MultiArray joint_effort;
  return joint_effort;
}

void EndEffectorTrajectoryController::publish_command(
    std_msgs::msg::Float64MultiArray command) {

  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.header.stamp = joint_state_.header.stamp;
  joint_trajectory.joint_names = joint_state_.name;
  trajectory_msgs::msg::JointTrajectoryPoint point;
  int32_t dt_sec = dt_millisec_ / 1000;
  int32_t dt_nsec = (dt_millisec_ % 1000) * 1000000;
  point.time_from_start = rclcpp::Duration(dt_sec, dt_nsec);
  point.velocities = command.data;
  joint_trajectory.points.push_back(point);
  joint_trajectory_publisher_->publish(joint_trajectory);
}

void EndEffectorTrajectoryController::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Update robot state information
  joint_state_ = *msg;
}

void EndEffectorTrajectoryController::odm_base_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Update robot state information
  odm_base_ = *msg;
}

void EndEffectorTrajectoryController::update_robot() {
  // Update robot state information
  // robot_.update(joint_state_, odm_base_.pose.pose, odm_base_.twist.twist);
  Eigen::Isometry3d pose =
      Eigen::Translation3d(odm_base_.pose.pose.position.x,
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
  robot_.overWriteJointState(joint_state_);
  robot_.setStateVariable(
      spacedyn_ros::Kinematics::computeForward(robot_, true, true, false));
}

floating_robot_interfaces::msg::EndEffectorTrajectoryPoint
EndEffectorTrajectoryController::get_current_end_effector_point(int id) {
  // Update robot state information
  floating_robot_interfaces::msg::EndEffectorTrajectoryPoint point;
  // point.pose = tf2::toMsg(robot_.getEndEffectorPose(id));
  // point.twist = tf2::toMsg(robot_.getEndEffectorTwist(id));
  return point;
}

void EndEffectorTrajectoryController ::init_pids(
    std::vector<control_toolbox::Pid> &pids) {
  const int dof_xyz = 3;

  pids.clear();

  control_toolbox::Pid pid;
  pid.initPid(get_parameter("point_follow_gain_p").as_double(),
              get_parameter("point_follow_gain_i").as_double(),
              get_parameter("point_follow_gain_d").as_double(),
              get_parameter("point_follow_gain_i_max").as_double(),
              get_parameter("point_follow_gain_i_min").as_double(), false);

  for (int i = 0; i < dof_xyz; i++) {
    pids.push_back(pid);
  }
}

geometry_msgs::msg::Twist
EndEffectorTrajectoryController ::compute_pids_command(
    std::vector<control_toolbox::Pid> &pids,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint des,
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint real) {
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
                 2; // Cuz quaternion angle is half as rotation angle
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

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  if (argc == 2) {
    rclcpp::spin(std::make_shared<
                 floating_robot_controller::EndEffectorTrajectoryController>(
        "end_effector_trajectory_controller", "follow_end_effector_trajectory",
        argv[1]));
  } else {
    rclcpp::spin(std::make_shared<
                 floating_robot_controller::EndEffectorTrajectoryController>(
        "end_effector_trajectory_controller",
        "follow_end_effector_trajectory"));
  }

  rclcpp::shutdown();
  return 0;
} // main()