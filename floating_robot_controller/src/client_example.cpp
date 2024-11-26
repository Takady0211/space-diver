#include "floating_robot_controller/client_example.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace floating_robot_controller {
EndEffectorTrajectoryClient ::EndEffectorTrajectoryClient(
    const char *node_name = "end_effect_trajectory_client")
    : Node(node_name) {
  // // End Effector Trajectory CLient Setting // //
  this->end_effec_trj_ctrl_clnt_ptr_ =
      rclcpp_action::create_client<FollowEndEffecTraject>(
          this, "follow_end_effector_trajectory");

  // State Publisher
  end_effector_goal_point_publisher_ = this->create_publisher<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
      "end_effector_trajectory_controller/goal", 10);
  end_effector_desired_point_publisher_ = this->create_publisher<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
      "end_effector_trajectory_controller/desired", 10);
  end_effector_actual_point_publisher_ = this->create_publisher<
      floating_robot_interfaces::msg::EndEffectorTrajectoryPoint>(
      "end_effector_trajectory_controller/actual", 10);

  // Request Trajectory Once
  end_effector_traject_set_goal();
  end_effector_traject_send_goal();
  return;
}

// Callback function to be called every time
// MAIN FUNCTION HERE
// ---------------------------------------------------------------->
void EndEffectorTrajectoryClient::timer_callback() {
  // send request to capture
  RCLCPP_INFO(this->get_logger(), "Successfully sent capture request");
}
// MAIN FUNCTION HERE

// End Effector Trajectory CLient
void EndEffectorTrajectoryClient::end_effector_traject_set_goal() {
  using Traject = floating_robot_interfaces::msg::EndEffectorTrajectory;
  using Point = floating_robot_interfaces::msg::EndEffectorTrajectoryPoint;
  Traject trajectory;
  Point point;
  trajectory.header.stamp = now();
  trajectory.header.stamp = now();

  // point 1
  point.pose.position.x = 1.3;
  point.pose.position.y = 0.2;
  point.pose.position.z = 0.2;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = sin(0.5 * -0.2);
  point.pose.orientation.w = cos(0.5 * -0.2);
  point.time_from_start.sec = 10;
  point.path_generation_id = Point::STRAIGHT_LINE_PATH;
  point.time_scaling_id = Point::POLY3_TIME_SCALING;
  trajectory.points.push_back(point);

  // point 2
  point.pose.position.x = 1.3;
  point.pose.position.y = -0.2;
  point.pose.position.z = 0.2;
  point.pose.orientation.x = 0.0;
  point.pose.orientation.y = 0.0;
  point.pose.orientation.z = sin(0.5 * 0.2);
  point.pose.orientation.w = cos(0.5 * 0.2);
  point.time_from_start.sec = 20;
  point.path_generation_id = Point::STRAIGHT_LINE_PATH;
  point.time_scaling_id = Point::POLY3_TIME_SCALING;
  trajectory.points.push_back(point);

  end_effec_trj_goal_msg_.trajectories.push_back(trajectory);
}

void EndEffectorTrajectoryClient::end_effector_traject_send_goal() {
  using namespace std::placeholders;
  if (!this->end_effec_trj_ctrl_clnt_ptr_->wait_for_action_server(
          std::chrono::seconds(6))) {
    RCLCPP_ERROR(
        this->get_logger(),
        "End Effector Trajectory Action server not available after waiting");
    rclcpp::shutdown();
    return;
  }

  // Send goal that is set before
  RCLCPP_INFO(this->get_logger(), "Sending End Effector Traject Goal");

  auto send_goal_options =
      rclcpp_action::Client<FollowEndEffecTraject>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &EndEffectorTrajectoryClient::end_effector_traject_goal_response_callback,
      this, _1);
  send_goal_options.feedback_callback = std::bind(
      &EndEffectorTrajectoryClient::end_effector_traject_feedback_callback,
      this, _1, _2);
  send_goal_options.result_callback = std::bind(
      &EndEffectorTrajectoryClient::end_effector_traject_result_callback, this,
      _1);
  this->end_effec_trj_ctrl_clnt_ptr_->async_send_goal(end_effec_trj_goal_msg_,
                                                      send_goal_options);
}

void EndEffectorTrajectoryClient::end_effector_traject_goal_response_callback(
    const ClientGoalHandleEndEffecTraject::SharedPtr &goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(),
                 "End effector traject goal was rejected by server");
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        "End effector traject goal accepted by server, waiting for result");
  }
}

void EndEffectorTrajectoryClient::end_effector_traject_feedback_callback(
    ClientGoalHandleEndEffecTraject::SharedPtr,
    const std::shared_ptr<const FollowEndEffecTraject::Feedback> feedback) {

  // Publish feedback
  end_effector_goal_point_publisher_->publish(
      end_effec_trj_goal_msg_.trajectories[0].points[0]);
  end_effector_desired_point_publisher_->publish(feedback->desired_points[0]);
  end_effector_actual_point_publisher_->publish(feedback->actual_points[0]);
  RCLCPP_INFO(this->get_logger(), "Received feedback ....");
}

void EndEffectorTrajectoryClient::end_effector_traject_result_callback(
    const ClientGoalHandleEndEffecTraject::WrappedResult &result) {
  switch (result.code) {
  case rclcpp_action::ResultCode::SUCCEEDED:
    break;
  case rclcpp_action::ResultCode::ABORTED:
    RCLCPP_ERROR(this->get_logger(),
                 "End effector traject goal was aborted, continue...");
    return;
  case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_ERROR(this->get_logger(), "End effector traject goal was canceled");
    return;
  default:
    RCLCPP_ERROR(this->get_logger(),
                 "End effector traject unknown result code");
    rclcpp::shutdown();
    return;
  }
  std::stringstream ss;
  ss << "End effector traject goal achieved";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
} // End Effector Trajectory CLient
} // namespace floating_robot_controller

// Is it better to separate main function?
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               floating_robot_controller::EndEffectorTrajectoryClient>());
  rclcpp::shutdown();
  return 0;
}