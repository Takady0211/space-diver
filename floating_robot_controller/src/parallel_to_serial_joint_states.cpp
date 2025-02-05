#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ParallelToSerialJointStates : public rclcpp::Node
{
public:
    ParallelToSerialJointStates() : Node("my_node")
    {
        // Publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states/serial", 10);

        // Subscriber
        subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&ParallelToSerialJointStates::subscription_callback, this, std::placeholders::_1));

    }

private:

    const double ROT_ = M_PI/4;

    void subscription_callback(const sensor_msgs::msg::JointState& msg)
    {
        sensor_msgs::msg::JointState serial_joint_state;
        serial_joint_state.name = msg.name;

        std::vector<double> serial_joint_position;
        std::vector<double> serial_joint_velocity;
        std::vector<double> serial_joint_effort;

        left_arm_joint_position(msg.position, serial_joint_position);
        left_arm_joint_velocity(msg.velocity, serial_joint_velocity);
        for (int i = 0; i < 3; i++)
        {
            serial_joint_effort.push_back(msg.effort[i]);
        }

        if (msg.position.size() == 6)
        {
            right_arm_joint_position(msg.position, serial_joint_position);
            right_arm_joint_velocity(msg.velocity, serial_joint_velocity);
            for (int i = 3; i < 6; i++)
            {
                serial_joint_effort.push_back(msg.effort[i]);
            }
        }

        serial_joint_state.position = serial_joint_position;
        serial_joint_state.velocity = serial_joint_velocity;
        serial_joint_state.effort = serial_joint_effort;

        publisher_->publish(serial_joint_state);
    }

    void left_arm_joint_position(
        const std::vector<double>& joint_position, std::vector<double>& serial_joint_position)
    {
        serial_joint_position.push_back(joint_position[0]);
        serial_joint_position.push_back(
            (joint_position[1] + M_PI/2 + ROT_)
        );
        serial_joint_position.push_back(
            (joint_position[2]) - (joint_position[1]+M_PI/2+ROT_)
        );
    }
    void left_arm_joint_velocity(
        const std::vector<double>& joint_velocity, std::vector<double>& serial_joint_velocity)
    {
        serial_joint_velocity.push_back(joint_velocity[0]);
        serial_joint_velocity.push_back(joint_velocity[1]);
        serial_joint_velocity.push_back(joint_velocity[2] - joint_velocity[1]);
    }

    void right_arm_joint_position(
        const std::vector<double>& joint_position, std::vector<double>& serial_joint_position)
    {
        serial_joint_position.push_back(joint_position[3]);
        serial_joint_position.push_back(
            (joint_position[4] + M_PI/2 - ROT_)
        );
        serial_joint_position.push_back(
            (joint_position[5]) + (M_PI) - (joint_position[4]+M_PI/2-ROT_)
        );
    }
    void right_arm_joint_velocity(
        const std::vector<double>& joint_velocity, std::vector<double>& serial_joint_velocity)
    {
        serial_joint_velocity.push_back(joint_velocity[3]);
        serial_joint_velocity.push_back(joint_velocity[4]);
        serial_joint_velocity.push_back(joint_velocity[5] - joint_velocity[4]);
    }
    

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParallelToSerialJointStates>());
    rclcpp::shutdown();
    return 0;
}
