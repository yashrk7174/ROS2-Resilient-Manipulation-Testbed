#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "joint_space_planner",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  // ----------------------------------------------------------
  // ROS2 executor
  //
  // The executor processes incoming callbacks, including
  // /joint_states required by MoveIt's CurrentStateMonitor.
  // ----------------------------------------------------------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  using moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface move_group(node, "panda_arm");

  RCLCPP_INFO(
    node->get_logger(),
    "Planning group: %s",
    move_group.getName().c_str());

  // Start listening to current robot state.
  move_group.startStateMonitor();

  auto current_state = move_group.getCurrentState(10.0);

  if (!current_state)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to obtain current robot state.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  std::vector<double> joint_target =
    move_group.getCurrentJointValues();

  if (joint_target.size() != 7)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Expected 7 Panda arm joints, received %zu.",
      joint_target.size());

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Current panda_joint1: %.3f rad",
    joint_target[0]);

  // ----------------------------------------------------------
  // Joint-space target
  // ----------------------------------------------------------
  joint_target[0] = 0.30;

  move_group.setJointValueTarget(joint_target);

  RCLCPP_INFO(
    node->get_logger(),
    "Target panda_joint1: %.3f rad",
    joint_target[0]);

  MoveGroupInterface::Plan plan;

  const bool planning_success =
    static_cast<bool>(move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Joint-space planning FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Joint-space planning SUCCESS.");

  const auto execution_result =
    move_group.execute(plan);

  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Trajectory execution FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Trajectory execution SUCCESS.");

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();

  return 0;
}
