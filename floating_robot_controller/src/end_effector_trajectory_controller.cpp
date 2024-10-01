#include "floating_robot_controller/end_effector_trajectory_controller.hpp"

#include "floating_robot_controller/trajectory.hpp"
#include "floating_robot_interfaces/action/follow_end_effector_trajectory.hpp"
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

#define END_EFFECTOR_ID 1
#define JOINT_NUM 6

namespace floating_robot_controller {
std::string package_share_directory =
    ament_index_cpp::get_package_share_directory("floating_robot_controller");
std::string model_path =
    package_share_directory.append("/description/space_dyn/sar_testbed.def");

EndEffectorTrajectoryController::EndEffectorTrajectoryController(
    const char *node_name = "end_effector_trajectory_controller",
    const char *action_name = "end_effector_trajectory_control")
    : Node(node_name) {
  using namespace std::placeholders;

  // Parameter setting
  declare_parameter("update_rate", 1000);
  declare_parameter("joint_initialize_duration", 5000);
  declare_parameter("initial_joint_positions",
                    std::vector<double>(JOINT_NUM, 0.0));
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

  // Basic functions
  // // Create timer
  executing_ = false;
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(dt_millisec_),
      std::bind(&EndEffectorTrajectoryController::timer_callback, this));
  // // Command interface
  command_effort_publisher_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "forward_effort_controller/commands", 10);
  joint_trajectory_publisher_ =
      this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
          "feedback_effort_controller/joint_trajectory", 10);
  // // State interface
  joint_state_.effort.resize(JOINT_NUM);
  joint_state_.position.resize(JOINT_NUM);
  joint_state_.velocity.resize(JOINT_NUM);
  joint_state_subscriber_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
          "joint_states", 10,
          std::bind(&EndEffectorTrajectoryController::joint_state_callback,
                    this, _1));
  odm_base_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odm/base", 10,
      std::bind(&EndEffectorTrajectoryController::odm_base_callback, this, _1));
  odm_target_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odm/target", 10,
      std::bind(&EndEffectorTrajectoryController::odm_target_callback, this,
                _1));

  // Action server
  this->action_server_ = rclcpp_action::create_server<FollowTraject>(
      this, action_name,
      std::bind(&EndEffectorTrajectoryController::handle_goal, this, _1, _2),
      std::bind(&EndEffectorTrajectoryController::handle_cancel, this, _1),
      std::bind(&EndEffectorTrajectoryController::handle_accepted, this, _1));
  RCLCPP_INFO(this->get_logger(),
              "End effector trajectory controller started!");
  // RCLCPP_INFO(this->get_logger(), "is floating %d", is_floating_);

  // Topic server
  end_effect_point_subscriber_ = this->create_subscription<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
      "end_effector_trajectory_controller/goal", 10,
      std::bind(&EndEffectorTrajectoryController::end_effect_point_callback,
                this, _1));

  // Publisher for data analysis
  current_pose_markers_.markers.resize(1);
  current_pose_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "end_effector_current_pose", 10);
  desired_pose_markers_.markers.resize(1);
  // end_effect_crnt_point_pub_ = this->create_publisher<
  //     floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
  //     "end_effector_current_point", 10);
  desired_pose_markers_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "end_effector_goal_pose", 10);
  // Transform publisher
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  // update robot before calculate end effector current point
  update_robot();
  traj_point_active_ptr_ = std::make_shared<Trajectory>(
      this->now(), get_current_end_effector_point(END_EFFECTOR_ID));

  controller_start_time_ = this->now();
}

rclcpp_action::GoalResponse EndEffectorTrajectoryController::handle_goal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FollowTraject::Goal> goal) {
  // Initialize trajectory when received goal
  // auto current_external_msg =
  // traj_point_active_ptr_->get_trajectory_msg();
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  update_robot();

  if (goal->trajectories.size() != 1) {
    RCLCPP_ERROR(this->get_logger(), "Goal must have only 1 trajectories!!");
  }

  // Initialize pid controller
  init_pids(pids_);

  auto current_point = get_current_end_effector_point(END_EFFECTOR_ID);

  traj_point_active_ptr_->update(goal->trajectories[0], current_point);

  RCLCPP_INFO(this->get_logger(), "Cancel preexisting goal");
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

void EndEffectorTrajectoryController::execute(
    const std::shared_ptr<ServerGoalHandleTraject> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FollowTraject::Feedback>();
  auto result = std::make_shared<FollowTraject::Result>();

  double sec_to_point;
  TrajectoryPointConstIter start_segment_left_itr;
  TrajectoryPointConstIter end_segment_left_itr;

  int loop_count = 0;
  int show_log_freq = 5000;
  bool lef_trj_completed = false;
  while (true) {
    loop_count++;
    executing_ = true;
    // If cancel requested, cancel trajectory
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // update robot state information
    update_robot();

    // Calculate desired point by interpolate
    traj_point_active_ptr_->sample(this->now(), sec_to_point, state_desired_,
                                   goal_state_, start_segment_left_itr,
                                   end_segment_left_itr);

    // If both goal completed, break loop
    if (end_segment_left_itr == traj_point_active_ptr_->end()) {

      if (!lef_trj_completed) {
        RCLCPP_INFO(this->get_logger(), "end-effector trajectory completed!!");
      }
      lef_trj_completed = true;
      break;
    }

    std::stringstream ss;
    ss << "sec to next point : ";
    ss << sec_to_point << "[s]";
    if (loop_count % show_log_freq == 0) {
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    // Calculate commanding end effector twist
    if (use_closed_loop_pid_adapter_) {
      // With feedback
      state_current_ = get_current_end_effector_point(END_EFFECTOR_ID);
      desired_twist_ =
          compute_pids_command(pids_, state_desired_, state_current_);
    } else {
      // Without feedback
      desired_twist_ = state_desired_.twist;
    }

    feedback->desired_points.push_back(state_desired_);
    feedback->actual_points.push_back(state_current_);

    goal_handle->publish_feedback(feedback);
  }
  executing_ = false;
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

rclcpp::Time EndEffectorTrajectoryController::get_current_time() {
  rclcpp::Duration timer_from_start_ = this->now() - controller_start_time_;
  rclcpp::Time zero_time(0, 0, RCL_ROS_TIME);
  return zero_time + timer_from_start_;
}

void EndEffectorTrajectoryController ::timer_callback() {
  update_robot();
  publish_tfs();
  if (executing_) {
    publish_command(compute_joint_velocity(desired_twist_)); // velocity
  } else {
    std_msgs::msg::Float64MultiArray zero;
    zero.data = std::vector<double>(JOINT_NUM, 0.0);
    publish_command(zero); // effort
  }
}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_velocity(
    geometry_msgs::msg::Twist end_effector_velocity) {
  // Compute joint velocity
  // auto GJ = robot_.computeGeneralizedJacobianForEndEffector(END_EFFECTOR_ID);
  // Eigen::VectorXd end_effector_velocity_vec(6);
  // end_effector_velocity_vec << end_effector_velocity.linear.x,
  //     end_effector_velocity.linear.y, end_effector_velocity.linear.z,
  //     end_effector_velocity.angular.x, end_effector_velocity.angular.y,
  //     end_effector_velocity.angular.z;
  // // calc joint velocity
  // Eigen::FullPivLU<Eigen::MatrixXd> LU(GJ);
  // Eigen::VectorXd des_joint_vel = LU.solve(end_effector_velocity_vec);
  std_msgs::msg::Float64MultiArray joint_velocity;
  // for (int i = 0; i < JOINT_NUM; i++) {
  //   joint_velocity.data.push_back(des_joint_vel(i));
  // }
  return joint_velocity;
}

std_msgs::msg::Float64MultiArray
EndEffectorTrajectoryController::compute_joint_effort(
    geometry_msgs::msg::Twist end_effector_velocity) {
  // Compute joint effort
  // auto GJ = robot_.computeGeneralizedJacobianForEndEffector(END_EFFECTOR_ID);
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
  // des_joint_acc = Eigen::VectorXd::Zero(JOINT_NUM);
  // des_joint_acc(0) = 0.001;
  std_msgs::msg::Float64MultiArray joint_effort;
  return joint_effort;
}

void EndEffectorTrajectoryController::publish_command(
    std_msgs::msg::Float64MultiArray command) {
  // command_effort_publisher_->publish(command);

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
  // joint_state_.velocity.resize(JOINT_NUM);
  // joint_state_.velocity[0] = 100.0;
}

void EndEffectorTrajectoryController::odm_base_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Update robot state information
  odm_base_ = *msg;
}

void EndEffectorTrajectoryController::odm_target_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Update robot state information
  odm_target_ = *msg;
}

void EndEffectorTrajectoryController::update_robot() {
  // Update robot state information
  // robot_.update(joint_state_, odm_base_.pose.pose, odm_base_.twist.twist);
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
  const int dim_lin = 3;
  // const int dim_rot = 4;

  pids.clear();

  control_toolbox::Pid pid;
  pid.initPid(get_parameter("point_follow_gain_p").as_double(),
              get_parameter("point_follow_gain_i").as_double(),
              get_parameter("point_follow_gain_d").as_double(),
              get_parameter("point_follow_gain_i_max").as_double(),
              get_parameter("point_follow_gain_i_min").as_double(), false);

  for (int i = 0; i < dim_lin; i++) {
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

void EndEffectorTrajectoryController ::set_traject_markers(
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint left) {
  // To rotate the arrow 90 deg.
  // End effector initial orientation is heading +y
  Eigen::Affine3d aff_in;
  Eigen::Affine3d aff_out;
  geometry_msgs::msg::TransformStamped trs;
  Eigen::Affine3d aff_trs;
  trs.transform.rotation.z = sqrt(2) * .5;
  trs.transform.rotation.w = sqrt(2) * .5;
  aff_trs = tf2::transformToEigen(trs);

  visualization_msgs::msg::Marker marker;
  // marker.lifetime.sec = 1e9;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.header.frame_id = "world";

  marker.header.stamp = joint_state_.header.stamp;
  marker.scale.x = 0.05;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  // left
  marker.ns = "desired_trajectory_left";
  marker.id = 0;
  tf2::fromMsg(left.pose, aff_in);
  aff_out = aff_in * aff_trs;
  marker.pose = tf2::toMsg(aff_out);
  marker.pose.position.z = 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  desired_pose_markers_.markers[0] = marker;
}

geometry_msgs::msg::TransformStamped
EndEffectorTrajectoryController::point_to_tf(
    floating_robot_interfaces::msg::EndEffectorTrajectoryPoint point,
    std::string child_frame_id) {
  geometry_msgs::msg::TransformStamped trs;
  trs.header.stamp = joint_state_.header.stamp;
  trs.header.frame_id = "world";
  trs.child_frame_id = child_frame_id;
  trs.transform.translation.x = point.pose.position.x;
  trs.transform.translation.y = point.pose.position.y;
  trs.transform.translation.z = point.pose.position.z;
  trs.transform.rotation = point.pose.orientation;
  return trs;
}

geometry_msgs::msg::TransformStamped
EndEffectorTrajectoryController::odm_to_tf(nav_msgs::msg::Odometry odm,
                                           std::string child_frame_id) {
  geometry_msgs::msg::TransformStamped trs;
  trs.header.stamp = joint_state_.header.stamp;
  trs.header.frame_id = "world";
  trs.child_frame_id = child_frame_id;
  trs.transform.translation.x = odm.pose.pose.position.x;
  trs.transform.translation.y = odm.pose.pose.position.y;
  trs.transform.translation.z = odm.pose.pose.position.z;
  trs.transform.rotation = odm.pose.pose.orientation;
  return trs;
}

void EndEffectorTrajectoryController::publish_tfs() {
  // Publish current end effector pose
  geometry_msgs::msg::TransformStamped trs;

  // Publish base odometry
  trs = odm_to_tf(odm_base_, "base_link");
  tf_broadcaster_->sendTransform(trs);

  // Publish target odometry
  trs = odm_to_tf(odm_target_, "target");
  tf_broadcaster_->sendTransform(trs);

  // Publish end effector: space_dyn
  trs = point_to_tf(get_current_end_effector_point(END_EFFECTOR_ID),
                    "end_effector_current");
  tf_broadcaster_->sendTransform(trs);

  // Publish end effector: goal
  trs = point_to_tf(goal_state_, "end_effector_goal");
  tf_broadcaster_->sendTransform(trs);

  // Publish end effector: goal
  trs = point_to_tf(state_desired_, "end_effector_desired");
  tf_broadcaster_->sendTransform(trs);
}
} // namespace floating_robot_controller

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               floating_robot_controller::EndEffectorTrajectoryController>(
      "end_effector_trajectory_controller", "follow_end_effector_trajectory"));
  rclcpp::shutdown();
  return 0;
} // main()