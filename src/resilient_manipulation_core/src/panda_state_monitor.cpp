#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class PandaStateMonitor : public rclcpp::Node
{
public:
  PandaStateMonitor()
  : Node("panda_state_monitor")
  {
    joint_state_subscription_ =
      this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::SensorDataQoS(),
        std::bind(
          &PandaStateMonitor::jointStateCallback,
          this,
          std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Panda state monitor started. Waiting for /joint_states...");
  }

private:
  void jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (received_first_message_) {
      return;
    }

    if (msg->name.empty() || msg->position.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Received an empty JointState message.");
      return;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received JointState message with %zu joints.",
      msg->name.size());

    for (std::size_t i = 0;
         i < msg->name.size() && i < msg->position.size();
         ++i)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "%s = %.3f rad",
        msg->name[i].c_str(),
        msg->position[i]);
    }

    received_first_message_ = true;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_subscription_;

  bool received_first_message_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PandaStateMonitor>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
