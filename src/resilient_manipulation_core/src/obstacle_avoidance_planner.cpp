#include <chrono>
#include <memory>
#include <thread>
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "obstacle_avoidance_planner",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  // ----------------------------------------------------------
  // Executor for joint states, TF and MoveIt communication
  // ----------------------------------------------------------
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;

  MoveGroupInterface move_group(node, "panda_arm");
  PlanningSceneInterface planning_scene_interface;
  // Remove any obstacle left from an earlier test run.
planning_scene_interface.removeCollisionObjects(
  {"phase3_blocking_box"});

rclcpp::sleep_for(1s);

  move_group.startStateMonitor();

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

  // Slow execution slightly for clear visual verification.
  move_group.setMaxVelocityScalingFactor(0.20);
  move_group.setMaxAccelerationScalingFactor(0.20);

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);

  // ----------------------------------------------------------
  // Current end-effector pose
  // ----------------------------------------------------------
  const auto current_pose_stamped =
    move_group.getCurrentPose();

  const auto current_pose =
    current_pose_stamped.pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Current pose: x=%.3f y=%.3f z=%.3f",
    current_pose.position.x,
    current_pose.position.y,
    current_pose.position.z);

  // ----------------------------------------------------------
  // Target pose:
  // 30 cm sideways, same height and orientation
  // ----------------------------------------------------------
  geometry_msgs::msg::Pose target_pose = current_pose;

// Two experimentally verified test positions.
// The planner alternates between them so the experiment
// can be repeated from either end.
constexpr double y_position_a = 0.091;
constexpr double y_position_b = 0.391;

const double distance_to_a =
  std::abs(current_pose.position.y - y_position_a);

const double distance_to_b =
  std::abs(current_pose.position.y - y_position_b);

if (distance_to_a < distance_to_b)
{
  target_pose.position.y = y_position_b;
}
else
{
  target_pose.position.y = y_position_a;
}

  RCLCPP_INFO(
    node->get_logger(),
    "Target pose:  x=%.3f y=%.3f z=%.3f",
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z);
// ----------------------------------------------------------
// Baseline reachability check WITHOUT obstacle
// ----------------------------------------------------------
move_group.setPoseTarget(target_pose);

MoveGroupInterface::Plan baseline_plan;

RCLCPP_INFO(
  node->get_logger(),
  "Baseline check: planning to target WITHOUT obstacle...");

const bool baseline_success =
  static_cast<bool>(
    move_group.plan(baseline_plan));

if (!baseline_success)
{
  RCLCPP_ERROR(
    node->get_logger(),
    "Baseline planning FAILED. Target pose is not reachable "
    "reliably even without the obstacle.");

  executor.cancel();
  executor_thread.join();
  rclcpp::shutdown();
  return 1;
}

RCLCPP_INFO(
  node->get_logger(),
  "Baseline pose-goal planning SUCCESS.");

move_group.clearPoseTargets();
move_group.setStartStateToCurrentState();


  // ----------------------------------------------------------
  // Create blocking collision object
  // ----------------------------------------------------------
  moveit_msgs::msg::CollisionObject obstacle;

  obstacle.header.frame_id =
    move_group.getPlanningFrame();

  obstacle.id = "phase3_blocking_box";

  shape_msgs::msg::SolidPrimitive box;

  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions.resize(3);

  // Box dimensions [m]
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.10;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.08;
  box.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.16;

  geometry_msgs::msg::Pose box_pose;

  box_pose.orientation.w = 1.0;

  // Place obstacle midway between current and target pose.
  box_pose.position.x = current_pose.position.x;
  box_pose.position.y =
  0.5 * (current_pose.position.y + target_pose.position.y);
  box_pose.position.z = current_pose.position.z;

  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(box_pose);
  obstacle.operation =
    moveit_msgs::msg::CollisionObject::ADD;

  RCLCPP_INFO(
    node->get_logger(),
    "Adding obstacle '%s' at x=%.3f y=%.3f z=%.3f",
    obstacle.id.c_str(),
    box_pose.position.x,
    box_pose.position.y,
    box_pose.position.z);

  const bool obstacle_added =
    planning_scene_interface.applyCollisionObject(obstacle);

  if (!obstacle_added)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to add collision object.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  // Give RViz a moment to display the updated scene.
  rclcpp::sleep_for(1s);

  // ----------------------------------------------------------
  // Verify obstacle exists
  // ----------------------------------------------------------
  const auto objects =
    planning_scene_interface.getObjects(
      {obstacle.id});

  if (objects.find(obstacle.id) == objects.end())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Collision object verification FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Collision object verified in planning scene.");

  // ----------------------------------------------------------
  // Pose-goal planning
  // ----------------------------------------------------------
  move_group.setPoseTarget(target_pose);

  MoveGroupInterface::Plan plan;

  RCLCPP_INFO(
    node->get_logger(),
    "Planning collision-aware trajectory...");

  const bool planning_success =
    static_cast<bool>(
      move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Collision-aware planning FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Collision-aware planning SUCCESS.");

  // ----------------------------------------------------------
  // Execute planned trajectory
  // ----------------------------------------------------------
  const auto execution_result =
    move_group.execute(plan);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Collision-aware trajectory execution FAILED.");

    executor.cancel();
    executor_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "Collision-aware trajectory execution SUCCESS.");

  // ----------------------------------------------------------
  // Final pose verification
  // ----------------------------------------------------------
  const auto final_pose =
    move_group.getCurrentPose().pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Final pose: x=%.3f y=%.3f z=%.3f",
    final_pose.position.x,
    final_pose.position.y,
    final_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Measured Y displacement: %.3f m",
    final_pose.position.y -
    current_pose.position.y);

  move_group.clearPoseTargets();

  // NOTE:
  // We intentionally leave the obstacle in the planning scene
  // so it remains visible for evidence screenshots.

  executor.cancel();
  executor_thread.join();

  rclcpp::shutdown();

  return 0;
}
