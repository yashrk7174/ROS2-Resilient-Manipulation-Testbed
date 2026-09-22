#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"


using MoveGroupInterface =
  moveit::planning_interface::MoveGroupInterface;


// ============================================================
// Helper: Plan and execute the target already set in MoveIt
// ============================================================

bool planAndExecute(
  MoveGroupInterface & move_group,
  const rclcpp::Logger & logger,
  const std::string & stage_name)
{
  MoveGroupInterface::Plan plan;

  const bool planning_success =
    static_cast<bool>(
      move_group.plan(plan));

  if (!planning_success)
  {
    RCLCPP_ERROR(
      logger,
      "%s planning FAILED.",
      stage_name.c_str());

    return false;
  }

  RCLCPP_INFO(
    logger,
    "%s planning SUCCESS.",
    stage_name.c_str());

  const auto execution_result =
    move_group.execute(plan);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger,
      "%s execution FAILED.",
      stage_name.c_str());

    return false;
  }

  RCLCPP_INFO(
    logger,
    "%s execution SUCCESS.",
    stage_name.c_str());

  return true;
}


// ============================================================
// Helper: Move robot to an SRDF named state
// ============================================================

bool executeNamedTarget(
  MoveGroupInterface & move_group,
  const std::string & target_name,
  const rclcpp::Logger & logger)
{
  move_group.setStartStateToCurrentState();

  const bool target_valid =
    move_group.setNamedTarget(target_name);

  if (!target_valid)
  {
    RCLCPP_ERROR(
      logger,
      "Named target '%s' was not found.",
      target_name.c_str());

    return false;
  }

  return planAndExecute(
    move_group,
    logger,
    "Named-target");
}


// ============================================================
// Helper: Plan and execute an end-effector pose goal
// ============================================================

bool executePoseGoal(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pose,
  const rclcpp::Logger & logger)
{
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  const bool success =
    planAndExecute(
      move_group,
      logger,
      "Pose-goal");

  move_group.clearPoseTargets();

  return success;
}


// ============================================================
// Helper: Execute straight Cartesian end-effector motion
// ============================================================

bool executeCartesianMove(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pose,
  const rclcpp::Logger & logger)
{
  move_group.setStartStateToCurrentState();

  const auto current_pose =
    move_group.getCurrentPose().pose;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  waypoints.push_back(current_pose);
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  constexpr double EEF_STEP = 0.01;
  constexpr double JUMP_THRESHOLD = 0.0;

  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      EEF_STEP,
      JUMP_THRESHOLD,
      trajectory,
      true);

  RCLCPP_INFO(
    logger,
    "Cartesian path achieved: %.1f%%",
    fraction * 100.0);

  if (fraction < 0.95)
  {
    RCLCPP_ERROR(
      logger,
      "Cartesian path incomplete. Execution cancelled.");

    return false;
  }

  const auto execution_result =
    move_group.execute(trajectory);

  if (execution_result !=
      moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger,
      "Cartesian trajectory execution FAILED.");

    return false;
  }

  RCLCPP_INFO(
    logger,
    "Cartesian trajectory execution SUCCESS.");

  return true;
}


// ============================================================
// Helper: Open / close Panda gripper
// ============================================================

bool commandGripper(
  MoveGroupInterface & hand_group,
  const double finger_position,
  const rclcpp::Logger & logger)
{
  std::vector<double> target =
    hand_group.getCurrentJointValues();

  if (target.size() != 2)
  {
    RCLCPP_ERROR(
      logger,
      "Expected 2 Panda finger joints, received %zu.",
      target.size());

    return false;
  }

  target[0] = finger_position;
  target[1] = finger_position;

  hand_group.setStartStateToCurrentState();

  const bool target_valid =
    hand_group.setJointValueTarget(target);

  if (!target_valid)
  {
    RCLCPP_ERROR(
      logger,
      "Invalid gripper joint target.");

    return false;
  }

  return planAndExecute(
    hand_group,
    logger,
    "Gripper");
}


// ============================================================
// MAIN
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp::Node>(
      "manipulation_mission",
      rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true));


  // ----------------------------------------------------------
  // ROS2 executor
  // ----------------------------------------------------------

  rclcpp::executors::SingleThreadedExecutor executor;

  executor.add_node(node);

  std::thread executor_thread(
    [&executor]()
    {
      executor.spin();
    });


  // Clean shutdown function
  auto shutdown_ros =
    [&]()
    {
      executor.cancel();

      if (executor_thread.joinable())
      {
        executor_thread.join();
      }

      rclcpp::shutdown();
    };


  // ----------------------------------------------------------
  // MoveIt interfaces
  // ----------------------------------------------------------

  MoveGroupInterface arm(
    node,
    "panda_arm");

  MoveGroupInterface hand(
    node,
    "hand");

  arm.startStateMonitor();
  hand.startStateMonitor();


  // ----------------------------------------------------------
  // Planning configuration
  // ----------------------------------------------------------

  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);

  arm.setMaxVelocityScalingFactor(0.20);
  arm.setMaxAccelerationScalingFactor(0.20);

  hand.setMaxVelocityScalingFactor(0.20);
  hand.setMaxAccelerationScalingFactor(0.20);


  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");

  RCLCPP_INFO(
    node->get_logger(),
    "PHASE 4.2 — FIXED BASELINE MANIPULATION MISSION");

  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");


  // ----------------------------------------------------------
  // Verify Panda state is available
  // ----------------------------------------------------------

  auto current_state =
    arm.getCurrentState(10.0);

  if (!current_state)
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "Could not obtain current Panda state.");

    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 1 — RESET ARM TO KNOWN READY STATE
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[1/7] Moving arm to SRDF 'ready' state...");

  if (!executeNamedTarget(
        arm,
        "ready",
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ----------------------------------------------------------
  // Measure known ready pose
  // ----------------------------------------------------------

  const auto ready_pose =
    arm.getCurrentPose().pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Ready pose measured: x=%.3f y=%.3f z=%.3f",
    ready_pose.position.x,
    ready_pose.position.y,
    ready_pose.position.z);


  // ==========================================================
  // FIXED TASK GEOMETRY
  //
  // These coordinates are intentionally fixed.
  // Every mission run uses the same task poses.
  // Orientation comes from the known Panda ready pose.
  // ==========================================================

  constexpr double TASK_X = 0.387;
  constexpr double TASK_Y = 0.000;

  constexpr double PREGRASP_Z = 0.550;
  constexpr double GRASP_Z = 0.470;
  constexpr double RETREAT_Z = 0.590;


  geometry_msgs::msg::Pose pregrasp_pose =
    ready_pose;

  pregrasp_pose.position.x = TASK_X;
  pregrasp_pose.position.y = TASK_Y;
  pregrasp_pose.position.z = PREGRASP_Z;


  geometry_msgs::msg::Pose grasp_pose =
    ready_pose;

  grasp_pose.position.x = TASK_X;
  grasp_pose.position.y = TASK_Y;
  grasp_pose.position.z = GRASP_Z;


  geometry_msgs::msg::Pose retreat_pose =
    ready_pose;

  retreat_pose.position.x = TASK_X;
  retreat_pose.position.y = TASK_Y;
  retreat_pose.position.z = RETREAT_Z;


  RCLCPP_INFO(
    node->get_logger(),
    "Fixed pre-grasp: x=%.3f y=%.3f z=%.3f",
    pregrasp_pose.position.x,
    pregrasp_pose.position.y,
    pregrasp_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Fixed grasp:     x=%.3f y=%.3f z=%.3f",
    grasp_pose.position.x,
    grasp_pose.position.y,
    grasp_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Fixed retreat:   x=%.3f y=%.3f z=%.3f",
    retreat_pose.position.x,
    retreat_pose.position.y,
    retreat_pose.position.z);


  // ==========================================================
  // STAGE 2 — OPEN GRIPPER
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[2/7] Opening gripper...");

  constexpr double GRIPPER_OPEN = 0.030;

  if (!commandGripper(
        hand,
        GRIPPER_OPEN,
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 3 — MOVE TO FIXED PRE-GRASP
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[3/7] Moving to fixed pre-grasp pose...");

  if (!executePoseGoal(
        arm,
        pregrasp_pose,
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 4 — CARTESIAN APPROACH
  //
  // z = 0.550 -> 0.470
  // displacement = 0.080 m downward
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[4/7] Cartesian approach 0.080 m downward...");

  if (!executeCartesianMove(
        arm,
        grasp_pose,
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 5 — CLOSE GRIPPER
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[5/7] Closing gripper...");

  constexpr double GRIPPER_CLOSED = 0.000;

  if (!commandGripper(
        hand,
        GRIPPER_CLOSED,
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 6 — CARTESIAN RETREAT
  //
  // z = 0.470 -> 0.590
  // displacement = 0.120 m upward
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[6/7] Cartesian retreat 0.120 m upward...");

  if (!executeCartesianMove(
        arm,
        retreat_pose,
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 7 — RETURN TO READY
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[7/7] Returning arm to SRDF 'ready' state...");

  if (!executeNamedTarget(
        arm,
        "ready",
        node->get_logger()))
  {
    shutdown_ros();

    return 1;
  }


  // ----------------------------------------------------------
  // Final verification
  // ----------------------------------------------------------

  const auto final_pose =
    arm.getCurrentPose().pose;

  RCLCPP_INFO(
    node->get_logger(),
    "Final pose after return: x=%.3f y=%.3f z=%.3f",
    final_pose.position.x,
    final_pose.position.y,
    final_pose.position.z);


  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");

  RCLCPP_INFO(
    node->get_logger(),
    "PHASE 4.2 BASELINE MISSION SUCCESS.");

  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");


  shutdown_ros();

  return 0;
}
