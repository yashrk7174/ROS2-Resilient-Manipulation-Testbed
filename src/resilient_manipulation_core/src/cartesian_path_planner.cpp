#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "cartesian_path_planner",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  // ----------------------------------------------------------
  // Executor
  // Required so MoveIt can process joint-state, TF,
  // action and service callbacks while our main thread works.
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

  RCLCPP_INFO(
    node->get_logger(),
    "Planning frame: %s",
    move_group.getPlanningFrame().c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "End-effector link: %s",
    move_group.getEndEffectorLink().c_str());

  // Start monitoring the current Panda state.
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

  move_group.setStartStateToCurrentState();

  // ----------------------------------------------------------
  // Read current end-effector pose
  // ----------------------------------------------------------
  const auto current_pose_stamped =
    move_group.getCurrentPose();

  geometry_msgs::msg::Pose current_pose =
    current_pose_stamped.pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose: x=%.3f y=%.3f z=%.3f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);

  // ----------------------------------------------------------
  // Define Cartesian target:
  // move 5 cm upward, preserve orientation
  // ----------------------------------------------------------
  geometry_msgs::msg::Pose target_pose = current_pose;

  target_pose.position.z += 0.08;

  RCLCPP_INFO(
    node->get_logger(),
    "Target pose:  x=%.3f y=%.3f z=%.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);

  // Waypoints for the end-effector.
  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  // Cartesian interpolation resolution = 1 cm.
  const double eef_step = 0.01;

  // 0.0 disables jump-threshold checking.
  // Acceptable here because this project is simulation-only.
  const double jump_threshold = 0.0;

  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      eef_step,
      jump_threshold,
      trajectory,
      true);

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian path achieved: %.1f%%",
    fraction * 100.0);

  // Do not execute an incomplete Cartesian trajectory.
  if (fraction < 0.95)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cartesian path incomplete. Execution cancelled.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian path planning SUCCESS.");

  const auto execution_result =
    move_group.execute(trajectory);

  if (execution_result != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cartesian trajectory execution FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Cartesian trajectory execution SUCCESS.");

  // Read pose again after execution for verification.
  const auto final_pose_stamped =
    move_group.getCurrentPose();

  const auto & final_pose =
    final_pose_stamped.pose;

  const double achieved_delta_z =
    final_pose.position.z - current_pose.position.z;

  RCLCPP_INFO(
    node->get_logger(),
    "Final pose:   x=%.3f y=%.3f z=%.3f",
    final_pose.position.x,
    final_pose.position.y,
    final_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Measured Z displacement: %.3f m",
    achieved_delta_z);

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();

  return 0;
}
