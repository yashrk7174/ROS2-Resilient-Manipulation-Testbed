#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;


class PoseVisualizer : public rclcpp::Node
{
public:
  PoseVisualizer()
  : Node("pose_visualizer")
  {
    marker_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
        "/resilient_manipulation/pose_markers",
        10);


    ground_truth_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_ground_truth",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);

          ground_truth_pose_ = *msg;
          ground_truth_received_ = true;
        });


    measurement_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_measurement",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);

          measurement_pose_ = *msg;
          measurement_received_ = true;

          measurement_history_.push_back(
            msg->pose.position);

          trimHistory(
            measurement_history_);
        });


    estimate_subscription_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_estimate",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);

          estimate_pose_ = *msg;
          estimate_received_ = true;

          estimate_history_.push_back(
            msg->pose.position);

          trimHistory(
            estimate_history_);
        });

    task_target_subscription_ =
      create_subscription<geometry_msgs::msg::PoseArray>(
        "/manipulation_task_targets",
        10,
        [this](
          const geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
          if (msg->poses.size() != 3)
          {
            RCLCPP_WARN(
              get_logger(),
              "Expected 3 manipulation task poses, received %zu.",
              msg->poses.size());

            return;
          }


          std::lock_guard<std::mutex> lock(
            data_mutex_);


          task_targets_ =
            *msg;

          task_targets_received_ =
            true;


          RCLCPP_INFO(
            get_logger(),
            "Exact mission targets received.");
        });


    timer_ =
      create_wall_timer(
        100ms,
        std::bind(
          &PoseVisualizer::publishMarkers,
          this));


    RCLCPP_INFO(
      get_logger(),
      "Pose visualizer started.");

    RCLCPP_INFO(
      get_logger(),
      "Perception + estimation + manipulation targets enabled.");

    RCLCPP_INFO(
      get_logger(),
      "Marker topic: /resilient_manipulation/pose_markers");
  }


private:
  // ============================================================
  // Configuration
  // ============================================================

  static constexpr std::size_t HISTORY_LENGTH =
    40;

  static constexpr double ERROR_MAGNIFICATION =
    10.0;


  // Diagnostic error-view panel.
  static constexpr double PANEL_X =
    0.15;

  static constexpr double PANEL_Y =
    -0.45;

  static constexpr double PANEL_Z =
    0.75;


  // ============================================================
  // History
  // ============================================================

  void trimHistory(
    std::deque<geometry_msgs::msg::Point> & history)
  {
    while (
      history.size() >
      HISTORY_LENGTH)
    {
      history.pop_front();
    }
  }


  // ============================================================
  // Generic position marker
  // ============================================================

  visualization_msgs::msg::Marker makePositionMarker(
    const geometry_msgs::msg::PoseStamped & source,
    const int id,
    const std::string & marker_namespace,
    const int marker_type,
    const double scale,
    const double red,
    const double green,
    const double blue,
    const double alpha)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
      source.header.frame_id;

    marker.header.stamp =
      now();

    marker.ns =
      marker_namespace;

    marker.id =
      id;

    marker.type =
      marker_type;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose =
      source.pose;


    const double quaternion_magnitude =
      std::abs(marker.pose.orientation.x)
      +
      std::abs(marker.pose.orientation.y)
      +
      std::abs(marker.pose.orientation.z)
      +
      std::abs(marker.pose.orientation.w);


    if (quaternion_magnitude < 1.0e-9)
    {
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
    }


    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;


    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;


    return marker;
  }


  // ============================================================
  // Text marker
  // ============================================================

  visualization_msgs::msg::Marker makeTextLabel(
    const std::string & frame_id,
    const int id,
    const std::string & marker_namespace,
    const std::string & text,
    const double x,
    const double y,
    const double z,
    const double red,
    const double green,
    const double blue,
    const double text_size = 0.018)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
      frame_id;

    marker.header.stamp =
      now();

    marker.ns =
      marker_namespace;

    marker.id =
      id;

    marker.type =
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

    marker.action =
      visualization_msgs::msg::Marker::ADD;


    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.w = 1.0;


    marker.scale.z =
      text_size;


    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;


    marker.text =
      text;


    return marker;
  }


  // ============================================================
  // True-scale sample history
  // ============================================================

  visualization_msgs::msg::Marker makeTrueScaleTrail(
    const std::deque<geometry_msgs::msg::Point> & history,
    const std::string & frame_id,
    const int id,
    const std::string & marker_namespace,
    const double point_size,
    const double red,
    const double green,
    const double blue,
    const double alpha)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
      frame_id;

    marker.header.stamp =
      now();

    marker.ns =
      marker_namespace;

    marker.id =
      id;

    marker.type =
      visualization_msgs::msg::Marker::SPHERE_LIST;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;


    marker.scale.x = point_size;
    marker.scale.y = point_size;
    marker.scale.z = point_size;


    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;


    for (const auto & point : history)
    {
      marker.points.push_back(
        point);
    }


    return marker;
  }


  // ============================================================
  // 10x diagnostic error history
  // ============================================================

  visualization_msgs::msg::Marker makeDiagnosticTrail(
    const std::deque<geometry_msgs::msg::Point> & history,
    const geometry_msgs::msg::Point & ground_truth,
    const std::string & frame_id,
    const int id,
    const std::string & marker_namespace,
    const double point_size,
    const double red,
    const double green,
    const double blue,
    const double alpha)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
      frame_id;

    marker.header.stamp =
      now();

    marker.ns =
      marker_namespace;

    marker.id =
      id;

    marker.type =
      visualization_msgs::msg::Marker::SPHERE_LIST;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;


    marker.scale.x = point_size;
    marker.scale.y = point_size;
    marker.scale.z = point_size;


    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;


    for (const auto & point : history)
    {
      geometry_msgs::msg::Point
        diagnostic_point;


      diagnostic_point.x =
        PANEL_X
        +
        ERROR_MAGNIFICATION
        * (
          point.x
          -
          ground_truth.x);


      diagnostic_point.y =
        PANEL_Y
        +
        ERROR_MAGNIFICATION
        * (
          point.y
          -
          ground_truth.y);


      diagnostic_point.z =
        PANEL_Z
        +
        ERROR_MAGNIFICATION
        * (
          point.z
          -
          ground_truth.z);


      marker.points.push_back(
        diagnostic_point);
    }


    return marker;
  }


  // ============================================================
  // Diagnostic zero-error reference
  // ============================================================

  visualization_msgs::msg::Marker makeDiagnosticReference(
    const std::string & frame_id)
  {
    geometry_msgs::msg::PoseStamped reference;

    reference.header.frame_id =
      frame_id;

    reference.pose.position.x =
      PANEL_X;

    reference.pose.position.y =
      PANEL_Y;

    reference.pose.position.z =
      PANEL_Z;

    reference.pose.orientation.w =
      1.0;


    return makePositionMarker(
      reference,
      150,
      "diagnostic_zero_error",
      visualization_msgs::msg::Marker::SPHERE,
      0.012,
      0.20,
      0.50,
      1.00,
      1.00);
  }



  // ============================================================
  // Manipulation target path
  // ============================================================

  visualization_msgs::msg::Marker makeTaskPath(
    const geometry_msgs::msg::PoseStamped & pregrasp,
    const geometry_msgs::msg::PoseStamped & grasp,
    const geometry_msgs::msg::PoseStamped & retreat)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id =
      grasp.header.frame_id;

    marker.header.stamp =
      now();

    marker.ns =
      "manipulation_target_path";

    marker.id =
      300;

    marker.type =
      visualization_msgs::msg::Marker::LINE_STRIP;

    marker.action =
      visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w =
      1.0;


    marker.scale.x =
      0.005;


    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.80;


    marker.points.push_back(
      pregrasp.pose.position);

    marker.points.push_back(
      grasp.pose.position);

    marker.points.push_back(
      retreat.pose.position);


    return marker;
  }


  // ============================================================
  // Publish complete visualization
  // ============================================================

  void publishMarkers()
  {
    geometry_msgs::msg::PoseStamped
      ground_truth;

    geometry_msgs::msg::PoseStamped
      measurement;

    geometry_msgs::msg::PoseStamped
      estimate;
    geometry_msgs::msg::PoseArray
      task_targets;


    std::deque<geometry_msgs::msg::Point>
      measurement_history;

    std::deque<geometry_msgs::msg::Point>
      estimate_history;

     bool have_task_targets =  false;
    bool have_ground_truth = false;
    bool have_measurement = false;
    bool have_estimate = false;


    {
      std::lock_guard<std::mutex> lock(
        data_mutex_);


      if (ground_truth_received_)
      {
        ground_truth =
          ground_truth_pose_;

        have_ground_truth =
          true;
      }


      if (measurement_received_)
      {
        measurement =
          measurement_pose_;

        measurement_history =
          measurement_history_;

        have_measurement =
          true;
      }


      if (estimate_received_)
      {
        estimate =
          estimate_pose_;

        estimate_history =
          estimate_history_;

        have_estimate =
          true;
      }
      if (task_targets_received_)
      {
        task_targets =
          task_targets_;

        have_task_targets =
          true;
      }
    }


    visualization_msgs::msg::MarkerArray
      marker_array;


    // ==========================================================
    // TRUE-SCALE PERCEPTION VIEW
    // ==========================================================

    if (have_ground_truth)
    {
      marker_array.markers.push_back(
        makePositionMarker(
          ground_truth,
          0,
          "ground_truth_object",
          visualization_msgs::msg::Marker::CUBE,
          0.050,
          0.15,
          0.45,
          1.00,
          0.12));
    }


    if (have_measurement)
    {
      marker_array.markers.push_back(
        makePositionMarker(
          measurement,
          10,
          "raw_measurement_current",
          visualization_msgs::msg::Marker::SPHERE,
          0.010,
          1.00,
          0.35,
          0.10,
          1.00));


      marker_array.markers.push_back(
        makeTrueScaleTrail(
          measurement_history,
          measurement.header.frame_id,
          11,
          "raw_measurement_true_history",
          0.004,
          1.00,
          0.35,
          0.10,
          0.55));
    }


    if (have_estimate)
    {
      marker_array.markers.push_back(
        makePositionMarker(
          estimate,
          20,
          "kalman_estimate_current",
          visualization_msgs::msg::Marker::SPHERE,
          0.012,
          0.10,
          1.00,
          0.25,
          1.00));


      marker_array.markers.push_back(
        makeTrueScaleTrail(
          estimate_history,
          estimate.header.frame_id,
          21,
          "kalman_estimate_true_history",
          0.003,
          0.10,
          1.00,
          0.25,
          0.70));
    }


    // ==========================================================
    // ESTIMATE-DRIVEN MANIPULATION TARGETS
    // ==========================================================

    // ==========================================================
    // EXACT MISSION MANIPULATION TARGETS
    //
    // These poses come directly from manipulation_mission.cpp.
    // They are not reconstructed independently by this node.
    // ==========================================================

    if (
      have_task_targets
      &&
      task_targets.poses.size() == 3)
    {
      geometry_msgs::msg::PoseStamped pregrasp;
      geometry_msgs::msg::PoseStamped grasp;
      geometry_msgs::msg::PoseStamped retreat;


      pregrasp.header =
        task_targets.header;

      grasp.header =
        task_targets.header;

      retreat.header =
        task_targets.header;


      pregrasp.pose =
        task_targets.poses[0];

      grasp.pose =
        task_targets.poses[1];

      retreat.pose =
        task_targets.poses[2];


      // PRE-GRASP — cyan
      marker_array.markers.push_back(
        makePositionMarker(
          pregrasp,
          310,
          "pregrasp_target",
          visualization_msgs::msg::Marker::SPHERE,
          0.018,
          0.10,
          0.80,
          1.00,
          0.95));


      // GRASP — yellow
      marker_array.markers.push_back(
        makePositionMarker(
          grasp,
          320,
          "grasp_target",
          visualization_msgs::msg::Marker::CUBE,
          0.018,
          1.00,
          0.85,
          0.10,
          0.75));


      // RETREAT — magenta
      marker_array.markers.push_back(
        makePositionMarker(
          retreat,
          330,
          "retreat_target",
          visualization_msgs::msg::Marker::SPHERE,
          0.018,
          1.00,
          0.25,
          0.80,
          0.95));


      marker_array.markers.push_back(
        makeTaskPath(
          pregrasp,
          grasp,
          retreat));


      marker_array.markers.push_back(
        makeTextLabel(
          task_targets.header.frame_id,
          311,
          "task_target_labels",
          "PRE_GRASP",
          pregrasp.pose.position.x,
          pregrasp.pose.position.y + 0.065,
          pregrasp.pose.position.z,
          0.10,
          0.80,
          1.00));


      marker_array.markers.push_back(
        makeTextLabel(
          task_targets.header.frame_id,
          321,
          "task_target_labels",
          "GRASP",
          grasp.pose.position.x,
          grasp.pose.position.y + 0.065,
          grasp.pose.position.z,
          1.00,
          0.85,
          0.10));


      marker_array.markers.push_back(
        makeTextLabel(
          task_targets.header.frame_id,
          331,
          "task_target_labels",
          "RETREAT",
          retreat.pose.position.x,
          retreat.pose.position.y + 0.065,
          retreat.pose.position.z,
          1.00,
          0.25,
          0.80));
    }



    // ==========================================================
    // 10x ERROR DIAGNOSTIC VIEW
    // ==========================================================

    if (have_ground_truth)
    {
      marker_array.markers.push_back(
        makeDiagnosticReference(
          ground_truth.header.frame_id));


      marker_array.markers.push_back(
        makeTextLabel(
          ground_truth.header.frame_id,
          200,
          "diagnostic_labels",
          "POSE_ERROR_VIEW_10x",
          PANEL_X,
          PANEL_Y,
          PANEL_Z + 0.18,
          1.00,
          1.00,
          1.00));


      marker_array.markers.push_back(
        makeTextLabel(
          ground_truth.header.frame_id,
          201,
          "diagnostic_labels",
          "RAW",
          PANEL_X - 0.11,
          PANEL_Y,
          PANEL_Z + 0.12,
          1.00,
          0.35,
          0.10));


      marker_array.markers.push_back(
        makeTextLabel(
          ground_truth.header.frame_id,
          202,
          "diagnostic_labels",
          "KALMAN",
          PANEL_X,
          PANEL_Y,
          PANEL_Z + 0.12,
          0.10,
          1.00,
          0.25));


      marker_array.markers.push_back(
        makeTextLabel(
          ground_truth.header.frame_id,
          203,
          "diagnostic_labels",
          "ZERO_ERROR",
          PANEL_X + 0.12,
          PANEL_Y,
          PANEL_Z + 0.12,
          0.20,
          0.50,
          1.00));
    }


    if (
      have_ground_truth
      &&
      have_measurement)
    {
      marker_array.markers.push_back(
        makeDiagnosticTrail(
          measurement_history,
          ground_truth.pose.position,
          ground_truth.header.frame_id,
          110,
          "diagnostic_raw_history",
          0.007,
          1.00,
          0.35,
          0.10,
          0.80));
    }


    if (
      have_ground_truth
      &&
      have_estimate)
    {
      marker_array.markers.push_back(
        makeDiagnosticTrail(
          estimate_history,
          ground_truth.pose.position,
          ground_truth.header.frame_id,
          120,
          "diagnostic_estimate_history",
          0.006,
          0.10,
          1.00,
          0.25,
          0.90));
    }


    if (!marker_array.markers.empty())
    {
      marker_publisher_->publish(
        marker_array);
    }
  }


  // ============================================================
  // Shared data
  // ============================================================

  std::mutex data_mutex_;


  geometry_msgs::msg::PoseStamped
    ground_truth_pose_;

  geometry_msgs::msg::PoseStamped
    measurement_pose_;

  geometry_msgs::msg::PoseStamped
    estimate_pose_;
  geometry_msgs::msg::PoseArray
    task_targets_;

  std::deque<geometry_msgs::msg::Point>
    measurement_history_;

  std::deque<geometry_msgs::msg::Point>
    estimate_history_;


  bool ground_truth_received_ =
    false;

  bool measurement_received_ =
    false;

  bool estimate_received_ =
    false;
  bool task_targets_received_ =
    false;

  // ============================================================
  // ROS interfaces
  // ============================================================

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    ground_truth_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    measurement_subscription_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    estimate_subscription_;


  rclcpp::Publisher<
    visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;


  rclcpp::TimerBase::SharedPtr
    timer_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseArray>::SharedPtr
    task_target_subscription_;
};


// ============================================================
// MAIN
// ============================================================

int main(
  int argc,
  char * argv[])
{
  rclcpp::init(
    argc,
    argv);


  auto node =
    std::make_shared<PoseVisualizer>();


  rclcpp::spin(
    node);


  rclcpp::shutdown();

  return 0;
}
