#include "floating_robot_controller/client_example.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "yaml-cpp/yaml.h"

namespace floating_robot_controller {
EndEffectorTrajectoryClient::EndEffectorTrajectoryClient(
    const std::string &yaml_goal,
    const char *node_name = "end_effect_trajectory_client")
    : Node(node_name) {
  (void)yaml_goal;
  RCLCPP_INFO(this->get_logger(), "YAML Goal: %s", yaml_goal.c_str());

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
  end_effector_traject_set_goal(yaml_goal);
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
void EndEffectorTrajectoryClient::end_effector_traject_set_goal(const std::string &yaml_goal) {
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

int validate_goal(const std::string &yaml_goal) {

  YAML::Node config;

  try {
    config = YAML::Load(yaml_goal);
  } catch (const YAML::ParserException &e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "YAML Parsing Error: %s",
                 e.what());
    return 0;
  }

  if (config["goal"]) {
    try {
      for (const auto &point : config["goal"]) {
        if (!point["pose"]["position"]["x"] ||
            !point["pose"]["position"]["y"] ||
            !point["pose"]["position"]["z"]) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Missing pose position { x, y, z } in YAML input");
          return 0;
        }
        if (!point["pose"]["orientation"]["x"] ||
            !point["pose"]["orientation"]["y"] ||
            !point["pose"]["orientation"]["z"] ||
            !point["pose"]["orientation"]["w"]) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Missing pose orientation { x, y, z, w } in YAML input");
          return 0;
        }
        if (!point["time_from_start"]) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Missing time_from_start in YAML input");
          return 0;
        }
        if (!point["path_generation_id"] || !point["time_scaling_id"]) {
          RCLCPP_ERROR(
              rclcpp::get_logger("rclcpp"),
              "Missing path_generation_id or time_scaling_id in YAML input");
          return 0;
        }
        int id = point["id"].as<int>();
        auto position = point["pose"]["position"];
        auto orientation = point["pose"]["orientation"];
        int time_from_start = point["time_from_start"].as<int>();
        std::string path_generation_id =
            point["path_generation_id"].as<std::string>();
        std::string time_scaling_id =
            point["time_scaling_id"].as<std::string>();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "\n\n\nPoint ID: %d, Position: [%.2f, %.2f, %.2f], "
                    "Orientation: [%.2f, %.2f, %.2f, %.2f]\n\n\n",
                    id, position["x"].as<double>(), position["y"].as<double>(),
                    position["z"].as<double>(), orientation["x"].as<double>(),
                    orientation["y"].as<double>(),
                    orientation["z"].as<double>(),
                    orientation["w"].as<double>());
      }
    } catch (const YAML::TypedBadConversion<YAML::Node> &e) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "YAML Parsing Error: %s",
                   e.what());
      return 0;
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "No 'goal' key found in YAML input");
    return 0;
  }
  return 1;
}

// Is it better to separate main function?
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "usage: <executable> <YAML_goal>");
    return 1;
  }

  std::string yaml_goal = argv[1];

  const int valid_goal = validate_goal(yaml_goal);

  if (!valid_goal) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid YAML goal");
    return 1;
  }

  rclcpp::spin(
      std::make_shared<floating_robot_controller::EndEffectorTrajectoryClient>(
          yaml_goal));
  rclcpp::shutdown();
  return 0;
}
