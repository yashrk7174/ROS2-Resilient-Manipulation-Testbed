#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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
  const std::string & stage_name,
  const bool resilience_enabled)
{
  // ----------------------------------------------------------
  // Phase 8.3 — Bounded MoveIt planning recovery
  // ----------------------------------------------------------

  const int max_planning_retries =
    resilience_enabled ? 1 : 0;

  MoveGroupInterface::Plan plan;

  bool planning_success = false;

  bool recovery_triggered = false;

  int planning_attempts = 0;

  std::chrono::steady_clock::time_point
    recovery_start;


  // Initial planning attempt + maximum one recovery retry.
  for (
    int retry_index = 0;
    retry_index <= max_planning_retries;
    ++retry_index)
  {
    ++planning_attempts;


    // Always refresh the robot start state immediately
    // before a new planning attempt.
    move_group.setStartStateToCurrentState();


    RCLCPP_INFO(
      logger,
      "%s planning attempt %d/%d.",
      stage_name.c_str(),
      planning_attempts,
      max_planning_retries + 1);


    // --------------------------------------------------------
    // Phase 8.3 test fault injection
    //
    // Disabled by default. Used only for controlled V&V.
    // Example:
    // RESILIENT_TEST_PLANNING_FAILURE_STAGE=Pose-goal
    // --------------------------------------------------------

    const char * injected_failure_stage =
      std::getenv(
        "RESILIENT_TEST_PLANNING_FAILURE_STAGE");


    const char * persistent_failure_stage =
      std::getenv(
        "RESILIENT_TEST_PERSISTENT_PLANNING_FAILURE_STAGE");


    const bool inject_first_attempt_failure =
      retry_index == 0
      &&
      injected_failure_stage != nullptr
      &&
      stage_name ==
        std::string(injected_failure_stage);


    const bool inject_persistent_failure =
      persistent_failure_stage != nullptr
      &&
      stage_name ==
        std::string(persistent_failure_stage);


    if (
      inject_first_attempt_failure ||
      inject_persistent_failure)
    {
      RCLCPP_WARN(
        logger,
        "PHASE 8.3 TEST FAULT INJECTION — "
        "forcing planning failure for stage=%s "
        "on attempt %d%s.",
        stage_name.c_str(),
        planning_attempts,
        inject_persistent_failure ?
          " [PERSISTENT]" :
          " [SINGLE]");


      planning_success = false;
    }
    else
    {
      planning_success =
        static_cast<bool>(
          move_group.plan(plan));
    }

    if (planning_success)
    {
      RCLCPP_INFO(
        logger,
        "%s planning SUCCESS on attempt %d.",
        stage_name.c_str(),
        planning_attempts);

      break;
    }


    RCLCPP_WARN(
      logger,
      "%s planning FAILED on attempt %d.",
      stage_name.c_str(),
      planning_attempts);


    // --------------------------------------------------------
    // One bounded recovery attempt is permitted.
    // --------------------------------------------------------

    if (retry_index < max_planning_retries)
    {
      recovery_triggered = true;

      recovery_start =
        std::chrono::steady_clock::now();


      RCLCPP_WARN(
        logger,
        "PHASE 8.3 PLANNING RECOVERY TRIGGERED — "
        "%s will refresh current state and re-plan once.",
        stage_name.c_str());
    }
  }


  // ----------------------------------------------------------
  // Planning recovery result
  // ----------------------------------------------------------

  double recovery_duration_ms = 0.0;


  if (recovery_triggered)
  {
    const auto recovery_end =
      std::chrono::steady_clock::now();


    recovery_duration_ms =
      static_cast<double>(
        std::chrono::duration_cast<
          std::chrono::milliseconds>(
            recovery_end -
            recovery_start)
          .count());
  }


  if (!planning_success)
  {
    if (!resilience_enabled)
    {
      RCLCPP_ERROR(
        logger,
        "PHASE 10 BASELINE PLANNING RESULT: "
        "NO_RECOVERY | stage=%s | attempts=%d",
        stage_name.c_str(),
        planning_attempts);

      RCLCPP_ERROR(
        logger,
        "%s planning failed; baseline planning recovery is disabled.",
        stage_name.c_str());
    }
    else
    {
      RCLCPP_ERROR(
        logger,
        "PHASE 8.3 PLANNING RECOVERY RESULT: "
        "SAFE_ABORT | stage=%s | attempts=%d | "
        "duration=%.1f ms",
        stage_name.c_str(),
        planning_attempts,
        recovery_duration_ms);

      RCLCPP_ERROR(
        logger,
        "%s planning failed after bounded retry.",
        stage_name.c_str());
    }

    return false;
  }


  if (recovery_triggered)
  {
    RCLCPP_INFO(
      logger,
      "PHASE 8.3 PLANNING RECOVERY RESULT: "
      "RECOVERED | stage=%s | attempts=%d | "
      "duration=%.1f ms",
      stage_name.c_str(),
      planning_attempts,
      recovery_duration_ms);
  }
  else
  {
    RCLCPP_INFO(
      logger,
      "PHASE 8.3 PLANNING RECOVERY RESULT: "
      "NOT_REQUIRED | stage=%s | attempts=%d | "
      "duration=0.0 ms",
      stage_name.c_str(),
      planning_attempts);
  }


  // ----------------------------------------------------------
  // Execution
  //
  // Planning recovery ends here.
  // Execution recovery belongs to Phase 8.4.
  // ----------------------------------------------------------

  // ----------------------------------------------------------
  // Phase 8.4 — Execution failure detection and safe handling
  // ----------------------------------------------------------

  const char * injected_execution_failure_stage =
    std::getenv(
      "RESILIENT_TEST_EXECUTION_FAILURE_STAGE");


  const bool inject_execution_failure =
    injected_execution_failure_stage != nullptr
    &&
    stage_name ==
      std::string(
        injected_execution_failure_stage);


  moveit::core::MoveItErrorCode execution_result =
    moveit::core::MoveItErrorCode::FAILURE;


  if (inject_execution_failure)
  {
    RCLCPP_WARN(
      logger,
      "PHASE 8.4 TEST FAULT INJECTION — "
      "forcing execution failure for stage=%s.",
      stage_name.c_str());
  }
  else
  {
    execution_result =
      move_group.execute(plan);
  }


  if (
    execution_result !=
    moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_ERROR(
      logger,
      "%s execution FAILED.",
      stage_name.c_str());


    RCLCPP_ERROR(
      logger,
      "PHASE 8.4 EXECUTION FAILURE HANDLING: "
      "SAFE_ABORT | stage=%s | "
      "automatic_retry=DISABLED",
      stage_name.c_str());


    RCLCPP_ERROR(
      logger,
      "Mission will not continue after "
      "execution failure.");


    return false;
  }


  RCLCPP_INFO(
    logger,
    "PHASE 8.4 EXECUTION FAILURE HANDLING: "
    "NOT_REQUIRED | stage=%s",
    stage_name.c_str());

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
  const rclcpp::Logger & logger,
  const bool resilience_enabled)
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
    "Named-target",
    resilience_enabled);
}


// ============================================================
// Helper: Plan and execute an end-effector pose goal
// ============================================================

bool executePoseGoal(
  MoveGroupInterface & move_group,
  const geometry_msgs::msg::Pose & target_pose,
  const rclcpp::Logger & logger,
  const bool resilience_enabled)
{
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target_pose);

  const bool success =
    planAndExecute(
      move_group,
      logger,
      "Pose-goal",
      resilience_enabled);

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
  const rclcpp::Logger & logger,
  const bool resilience_enabled)
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
    "Gripper",
    resilience_enabled);
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
  if (!node->has_parameter("object_pose_topic"))
  {
    node->declare_parameter<std::string>(
      "object_pose_topic",
      "/object_pose_estimate");
  }

  if (!node->has_parameter("resilience_enabled"))
  {
    node->declare_parameter<bool>(
      "resilience_enabled",
      true);
  }

  const std::string object_pose_topic =
    node->get_parameter(
      "object_pose_topic").as_string();

  const bool resilience_enabled =
    node->get_parameter(
      "resilience_enabled").as_bool();

  RCLCPP_INFO(
    node->get_logger(),
    "Validation configuration: mode=%s | object_pose_topic=%s",
    resilience_enabled ? "RESILIENT" : "BASELINE",
    object_pose_topic.c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "Validation configuration: estimate_recovery=%s | planning_retry=%s",
    resilience_enabled ? "ENABLED" : "DISABLED",
    resilience_enabled ? "ENABLED" : "DISABLED");

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
  // Phase 7.1 — Estimated object-pose interface
  // ----------------------------------------------------------

  std::mutex estimate_mutex;
  std::condition_variable estimate_cv;

  geometry_msgs::msg::PoseStamped latest_object_estimate;

  bool estimate_received = false;


  auto estimate_subscription =
    node->create_subscription<geometry_msgs::msg::PoseStamped>(
    object_pose_topic,
      10,
      [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        {
          std::lock_guard<std::mutex> lock(
            estimate_mutex);

          latest_object_estimate = *msg;
          estimate_received = true;
        }

        estimate_cv.notify_one();
      });
  // ----------------------------------------------------------
  // C4 — Exact manipulation-task target publisher
  // ----------------------------------------------------------

  auto task_target_publisher =
    node->create_publisher<geometry_msgs::msg::PoseArray>(
      "/manipulation_task_targets",
      10);


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
    RCLCPP_INFO(
    node->get_logger(),
    "MoveIt planning frame: %s",
    arm.getPlanningFrame().c_str());

  RCLCPP_INFO(
    node->get_logger(),
    "MoveIt end-effector link: %s",
    arm.getEndEffectorLink().c_str());

  // ----------------------------------------------------------
  // Planning configuration
  // ----------------------------------------------------------

  arm.setPlanningTime(10.0);
  arm.setNumPlanningAttempts(10);

  arm.setMaxVelocityScalingFactor(0.20);
  arm.setMaxAccelerationScalingFactor(0.20);

  hand.setMaxVelocityScalingFactor(0.20);
  hand.setMaxAccelerationScalingFactor(0.20);

  // ----------------------------------------------------------
  // Phase 7.1 — Wait for first configured object pose
  // ----------------------------------------------------------

  if (
    resilience_enabled &&
    object_pose_topic == "/object_pose_estimate")
  {
    // Preserve the exact Phase-9 synchronization marker.
    RCLCPP_INFO(
      node->get_logger(),
      "PHASE 7 — Waiting for valid /object_pose_estimate...");
  }
  else
  {
    RCLCPP_INFO(
      node->get_logger(),
      "PHASE 10 %s — Waiting for pose input on %s...",
      resilience_enabled ? "RESILIENT" : "BASELINE",
      object_pose_topic.c_str());
  }

  geometry_msgs::msg::PoseStamped object_estimate;


  // ----------------------------------------------------------
  // Phase 7.1 — Wait for first estimated object pose
  // ----------------------------------------------------------

  {
    std::unique_lock<std::mutex> lock(
      estimate_mutex);


    const bool received =
      estimate_cv.wait_for(
        lock,
        std::chrono::seconds(10),
        [&]()
        {
          return estimate_received;
        });


    if (!received)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "No pose input received on %s within 10 seconds.",
        object_pose_topic.c_str());

      shutdown_ros();

      return 1;
    }


    object_estimate =
      latest_object_estimate;
  }


  // ----------------------------------------------------------
  // Phase 8.2 — Estimate validation + bounded reacquisition
  // ----------------------------------------------------------

  constexpr double MAX_ESTIMATE_AGE_SECONDS =
    0.50;


  constexpr auto ESTIMATE_RECOVERY_TIMEOUT =
    std::chrono::seconds(3);


  // ----------------------------------------------------------
  // Local estimate validation function
  // ----------------------------------------------------------

  const auto validate_estimate =
    [&](const geometry_msgs::msg::PoseStamped & estimate,
        const char * label)
    {
      const double now_seconds =
        node->get_clock()->now().seconds();


      const double estimate_stamp_seconds =
        static_cast<double>(
          estimate.header.stamp.sec)
        +
        static_cast<double>(
          estimate.header.stamp.nanosec)
          * 1.0e-9;


      double estimate_age_seconds =
        now_seconds -
        estimate_stamp_seconds;


      // Protect against very small negative values caused by
      // clock adjustment / timing jitter.
      if (estimate_age_seconds < 0.0)
      {
        estimate_age_seconds = 0.0;
      }


      const bool estimate_fresh =
        estimate_age_seconds <=
        MAX_ESTIMATE_AGE_SECONDS;


      const bool frame_valid =
        estimate.header.frame_id ==
        arm.getPlanningFrame();


      const bool position_finite =
        std::isfinite(
          estimate.pose.position.x)
        &&
        std::isfinite(
          estimate.pose.position.y)
        &&
        std::isfinite(
          estimate.pose.position.z);


      RCLCPP_INFO(
        node->get_logger(),
        "%s age: %.1f ms "
        "(maximum allowed %.1f ms)",
        label,
        estimate_age_seconds * 1000.0,
        MAX_ESTIMATE_AGE_SECONDS * 1000.0);


      RCLCPP_INFO(
        node->get_logger(),
        "%s frame check: %s",
        label,
        frame_valid ? "PASS" : "FAIL");


      RCLCPP_INFO(
        node->get_logger(),
        "%s finite-position check: %s",
        label,
        position_finite ? "PASS" : "FAIL");


      RCLCPP_INFO(
        node->get_logger(),
        "%s freshness check: %s",
        label,
        estimate_fresh ? "PASS" : "FAIL");


      return
        estimate_fresh &&
        frame_valid &&
        position_finite;
    };


  // ----------------------------------------------------------
  // First validation attempt
  // ----------------------------------------------------------

  const bool initial_estimate_valid =
    validate_estimate(
      object_estimate,
      "Initial estimate");


  if (initial_estimate_valid)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "PHASE 7.4 ESTIMATE VALIDITY CHECK: PASS");


    RCLCPP_INFO(
      node->get_logger(),
      "PHASE 8.2 RECOVERY RESULT: "
      "NOT_REQUIRED | attempts=0 | duration=0.0 ms");
  }
  else
  {
    RCLCPP_WARN(
      node->get_logger(),
      "PHASE 7.4 ESTIMATE VALIDITY CHECK: FAIL");

    if (!resilience_enabled)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "PHASE 10 BASELINE: invalid/stale pose detected; "
        "estimate recovery is disabled.");

      RCLCPP_ERROR(
        node->get_logger(),
        "PHASE 10 BASELINE RESULT: "
        "SAFE_ABORT | recovery=DISABLED");

      shutdown_ros();

      return 1;
    }

    RCLCPP_WARN(
      node->get_logger(),
      "PHASE 8.2 RECOVERY TRIGGERED — "
      "waiting up to 3.0 s for a NEW valid estimate.");


    const auto recovery_start =
      std::chrono::steady_clock::now();


    const auto recovery_deadline =
      recovery_start +
      ESTIMATE_RECOVERY_TIMEOUT;


    int recovery_attempts = 0;

    bool recovered = false;


    auto last_stamp_sec =
      object_estimate.header.stamp.sec;

    auto last_stamp_nanosec =
      object_estimate.header.stamp.nanosec;


    // --------------------------------------------------------
    // Bounded estimate-reacquisition loop
    // --------------------------------------------------------

    while (
      std::chrono::steady_clock::now() <
      recovery_deadline)
    {
      geometry_msgs::msg::PoseStamped
        candidate_estimate;


      bool new_estimate_received = false;


      {
        std::unique_lock<std::mutex> lock(
          estimate_mutex);


        new_estimate_received =
          estimate_cv.wait_until(
            lock,
            recovery_deadline,
            [&]()
            {
              return
                estimate_received
                &&
                (
                  latest_object_estimate
                    .header.stamp.sec
                    != last_stamp_sec
                  ||
                  latest_object_estimate
                    .header.stamp.nanosec
                    != last_stamp_nanosec
                );
            });


        if (new_estimate_received)
        {
          candidate_estimate =
            latest_object_estimate;
        }
      }


      if (!new_estimate_received)
      {
        break;
      }


      ++recovery_attempts;


      last_stamp_sec =
        candidate_estimate.header.stamp.sec;

      last_stamp_nanosec =
        candidate_estimate.header.stamp.nanosec;


      RCLCPP_WARN(
        node->get_logger(),
        "Recovery candidate received "
        "(attempt %d).",
        recovery_attempts);


      if (
        validate_estimate(
          candidate_estimate,
          "Recovery candidate"))
      {
        object_estimate =
          candidate_estimate;

        recovered = true;

        break;
      }


      RCLCPP_WARN(
        node->get_logger(),
        "Recovery candidate rejected. "
        "Waiting for another estimate.");
    }


    const auto recovery_end =
      std::chrono::steady_clock::now();


    const double recovery_duration_ms =
      static_cast<double>(
        std::chrono::duration_cast<
          std::chrono::milliseconds>(
            recovery_end -
            recovery_start)
          .count());


    if (!recovered)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "PHASE 8.2 RECOVERY RESULT: "
        "SAFE_ABORT | attempts=%d | "
        "duration=%.1f ms",
        recovery_attempts,
        recovery_duration_ms);


      RCLCPP_ERROR(
        node->get_logger(),
        "No valid estimate reacquired "
        "within recovery timeout. "
        "Manipulation cancelled before planning.");


      shutdown_ros();

      return 1;
    }


    RCLCPP_INFO(
      node->get_logger(),
      "PHASE 8.2 RECOVERY RESULT: "
      "RECOVERED | attempts=%d | "
      "duration=%.1f ms",
      recovery_attempts,
      recovery_duration_ms);


    RCLCPP_INFO(
      node->get_logger(),
      "Fresh valid estimate reacquired. "
      "Mission will continue.");
  }


  RCLCPP_INFO(
    node->get_logger(),
    "PHASE 8.2 ESTIMATE ACQUISITION: PASS");

  RCLCPP_INFO(
    node->get_logger(),
    "ESTIMATED OBJECT POSE RECEIVED");


  RCLCPP_INFO(
    node->get_logger(),
    "Estimate frame: %s",
    object_estimate.header.frame_id.c_str());


  RCLCPP_INFO(
    node->get_logger(),
    "Estimated object position: x=%.3f y=%.3f z=%.3f",
    object_estimate.pose.position.x,
    object_estimate.pose.position.y,
    object_estimate.pose.position.z);


  RCLCPP_INFO(
  node->get_logger(),
  "Estimate accepted for MoveIt task generation.");
  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");

  RCLCPP_INFO(
    node->get_logger(),
    "PHASE 7 — ROBUST ESTIMATE-DRIVEN MANIPULATION MISSION");

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
        node->get_logger(),
        resilience_enabled))
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
  // PHASE 7.2 — ESTIMATE-DRIVEN TASK GEOMETRY
  //
  // Object position comes from the configured validation pose topic.
  // Resilient mode uses /object_pose_estimate.
  // Baseline mode uses /object_pose_measurement.

  // Manipulation orientation remains the verified Panda
  // ready-pose orientation.
  //
  // Pre-grasp : object Z + 0.080 m
  // Grasp     : estimated object position
  // Retreat   : object Z + 0.120 m
  // ==========================================================

  constexpr double PREGRASP_OFFSET_Z = 0.080;
  constexpr double RETREAT_OFFSET_Z = 0.120;


  const double object_x =
    object_estimate.pose.position.x;

  const double object_y =
    object_estimate.pose.position.y;

  const double object_z =
    object_estimate.pose.position.z;


  geometry_msgs::msg::Pose pregrasp_pose =
    ready_pose;

  pregrasp_pose.position.x = object_x;
  pregrasp_pose.position.y = object_y;
  pregrasp_pose.position.z =
    object_z + PREGRASP_OFFSET_Z;


  geometry_msgs::msg::Pose grasp_pose =
    ready_pose;

  grasp_pose.position.x = object_x;
  grasp_pose.position.y = object_y;
  grasp_pose.position.z = object_z;


  geometry_msgs::msg::Pose retreat_pose =
    ready_pose;

  retreat_pose.position.x = object_x;
  retreat_pose.position.y = object_y;
  retreat_pose.position.z =
    object_z + RETREAT_OFFSET_Z;

    RCLCPP_INFO(
    node->get_logger(),
    "Estimated pre-grasp: x=%.3f y=%.3f z=%.3f",
    pregrasp_pose.position.x,
    pregrasp_pose.position.y,
    pregrasp_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Estimated grasp:     x=%.3f y=%.3f z=%.3f",
    grasp_pose.position.x,
    grasp_pose.position.y,
    grasp_pose.position.z);

  RCLCPP_INFO(
    node->get_logger(),
    "Estimated retreat:   x=%.3f y=%.3f z=%.3f",
    retreat_pose.position.x,
    retreat_pose.position.y,
    retreat_pose.position.z);
  // ----------------------------------------------------------
  // C4 — Publish the exact task poses used by this mission
  // ----------------------------------------------------------

  geometry_msgs::msg::PoseArray task_targets;

  task_targets.header.frame_id =
    arm.getPlanningFrame();

  task_targets.header.stamp =
    node->get_clock()->now();


  task_targets.poses.push_back(
    pregrasp_pose);

  task_targets.poses.push_back(
    grasp_pose);

  task_targets.poses.push_back(
    retreat_pose);


  task_target_publisher->publish(
    task_targets);


  RCLCPP_INFO(
    node->get_logger(),
    "Published exact manipulation targets: "
    "PRE_GRASP, GRASP, RETREAT.");

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
        node->get_logger(),
        resilience_enabled))
  {
    shutdown_ros();

    return 1;
  }


  // ==========================================================
  // STAGE 3 — MOVE TO ESTIMATE-DRIVEN PRE-GRASP
  // ==========================================================

  RCLCPP_INFO(
    node->get_logger(),
    "[3/7] Moving to estimate-driven pre-grasp pose...");

  if (!executePoseGoal(
        arm,
        pregrasp_pose,
        node->get_logger(),
        resilience_enabled))
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
        node->get_logger(),
        resilience_enabled))
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
        node->get_logger(),
        resilience_enabled))
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
    "PHASE 7 INTEGRATED ESTIMATE-DRIVEN MANIPULATION SUCCESS.");

  RCLCPP_INFO(
    node->get_logger(),
    "==================================================");


  shutdown_ros();

  return 0;
}
