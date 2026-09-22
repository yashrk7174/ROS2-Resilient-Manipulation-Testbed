#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;


// ============================================================
//  Stale / Delayed Pose Measurement Injector
//
// Ground truth:
//   Published continuously.
//
// Measurement:
//   Published continuously, but after buffer warm-up it
//   represents the ground-truth pose from 10 samples earlier.
//
// A deterministic X-position change is used only as a test
// stimulus so that stale data can be observed quantitatively.
//
// No Gaussian noise.
// No systematic bias.
// No outlier.
// No dropout.
// ============================================================

class PoseUncertaintyInjector : public rclcpp::Node
{
public:
  PoseUncertaintyInjector()
  : Node("pose_uncertainty_injector")
  {
    ground_truth_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/object_pose_ground_truth",
        10);

    measurement_publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/object_pose_measurement",
        10);

    timer_ =
      this->create_wall_timer(
        100ms,
        std::bind(
          &PoseUncertaintyInjector::publishPose,
          this));

    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 5.6 — STALE / DELAYED POSE SOURCE");

    RCLCPP_INFO(
      this->get_logger(),
      "Mode: FIXED SAMPLE DELAY");

    RCLCPP_INFO(
      this->get_logger(),
      "Base task pose: x=%.3f y=%.3f z=%.3f m",
      BASE_X,
      GROUND_TRUTH_Y,
      GROUND_TRUTH_Z);

    RCLCPP_INFO(
      this->get_logger(),
      "Test motion: X changes by %.3f m (%.1f mm)",
      TEST_MOTION_X,
      TEST_MOTION_X * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Measurement delay: %zu samples (~%.1f s nominal)",
      DELAY_SAMPLES,
      static_cast<double>(DELAY_SAMPLES) * 0.1);

    RCLCPP_INFO(
      this->get_logger(),
      "Measurement remains continuously available");

    RCLCPP_INFO(
      this->get_logger(),
      "Frame: world | Publish rate: 10 Hz");

    RCLCPP_INFO(
      this->get_logger(),
      "==============================================");
  }


private:

  // ==========================================================
  //  baseline task pose
  // ==========================================================

  static constexpr double BASE_X = 0.387;
  static constexpr double GROUND_TRUTH_Y = 0.000;
  static constexpr double GROUND_TRUTH_Z = 0.470;


  // ==========================================================
  // Controlled test stimulus
  //
  // Ground truth alternates between:
  //
  // 0.387 m
  // 0.407 m
  //
  // every 30 samples.
  //
  // This is NOT a fault.
  // It only makes measurement delay observable.
  // ==========================================================

  static constexpr double TEST_MOTION_X = 0.020;
  static constexpr std::size_t MOTION_INTERVAL = 30;


  // ==========================================================
  // Delay configuration
  //
  // 10 samples at nominal 10 Hz ≈ 1 second.
  // ==========================================================

  static constexpr std::size_t DELAY_SAMPLES = 10;


  // ==========================================================
  // Determine current ground-truth X position
  // ==========================================================

  double getGroundTruthX() const
  {
    const std::size_t motion_phase =
      (sample_count_ / MOTION_INTERVAL) % 2;

    if (motion_phase == 0)
    {
      return BASE_X;
    }

    return BASE_X + TEST_MOTION_X;
  }


  // ==========================================================
  // Timer callback
  // ==========================================================

  void publishPose()
  {
    ++sample_count_;


    // --------------------------------------------------------
    // Current ground truth
    // --------------------------------------------------------

    geometry_msgs::msg::PoseStamped ground_truth;

    ground_truth.header.stamp =
      this->get_clock()->now();

    ground_truth.header.frame_id =
      "world";

    ground_truth.pose.position.x =
      getGroundTruthX();

    ground_truth.pose.position.y =
      GROUND_TRUTH_Y;

    ground_truth.pose.position.z =
      GROUND_TRUTH_Z;

    ground_truth.pose.orientation.x = 0.0;
    ground_truth.pose.orientation.y = 0.0;
    ground_truth.pose.orientation.z = 0.0;
    ground_truth.pose.orientation.w = 1.0;


    // Ground truth always published.
    ground_truth_publisher_->publish(
      ground_truth);


    // --------------------------------------------------------
    // Add current truth sample to delay buffer
    // --------------------------------------------------------

    pose_buffer_.push_back(
      ground_truth);


    geometry_msgs::msg::PoseStamped measurement;

    bool delay_active = false;


    // --------------------------------------------------------
    // After enough samples exist, publish the oldest
    // buffered sample.
    //
    // This produces exactly DELAY_SAMPLES of sample delay.
    // --------------------------------------------------------

    if (pose_buffer_.size() > DELAY_SAMPLES)
    {
      measurement =
        pose_buffer_.front();

      pose_buffer_.pop_front();

      delay_active = true;
    }
    else
    {
      // Buffer warm-up.
      //
      // Publish current truth so measurement remains available.
      // No artificial dropout is introduced.
      measurement =
        ground_truth;
    }


    // --------------------------------------------------------
    // Publish measurement
    // --------------------------------------------------------

    measurement_publisher_->publish(
      measurement);


    // --------------------------------------------------------
    // Quantify spatial error
    // --------------------------------------------------------

    const double error_x =
      measurement.pose.position.x -
      ground_truth.pose.position.x;

    const double error_y =
      measurement.pose.position.y -
      ground_truth.pose.position.y;

    const double error_z =
      measurement.pose.position.z -
      ground_truth.pose.position.z;

    const double position_error =
      std::sqrt(
        error_x * error_x +
        error_y * error_y +
        error_z * error_z);


    // --------------------------------------------------------
    // Quantify measurement age
    //
    // Important:
    // measurement.header.stamp belongs to the OLD sample.
    //
    // --------------------------------------------------------

    const rclcpp::Time now =
      this->get_clock()->now();

    const rclcpp::Time measurement_stamp(
      measurement.header.stamp);

    const double measurement_age =
      (now - measurement_stamp).seconds();


    // --------------------------------------------------------
    // Logging
    // --------------------------------------------------------

    if (!delay_active)
    {
      if (sample_count_ == 1)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "DELAY BUFFER WARM-UP START");
      }

      if (sample_count_ == DELAY_SAMPLES)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "DELAY BUFFER WARM-UP COMPLETE");
      }
    }


    if (sample_count_ % 10 == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "%s | sample=%zu | "
        "GT_X=%.3f | "
        "MEAS_X=%.3f | "
        "error=%.3f m | "
        "age=%.3f s",
        delay_active ? "DELAYED" : "WARMUP",
        sample_count_,
        ground_truth.pose.position.x,
        measurement.pose.position.x,
        position_error,
        measurement_age);
    }


    // Highlight moments when stale data becomes spatially wrong.
    if (
      delay_active &&
      position_error > 0.001)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "STALE MEASUREMENT | sample=%zu | "
        "current GT_X=%.3f | "
        "old MEAS_X=%.3f | "
        "position error=%.3f m | "
        "measurement age=%.3f s",
        sample_count_,
        ground_truth.pose.position.x,
        measurement.pose.position.x,
        position_error,
        measurement_age);
    }
  }


  // ==========================================================
  // ROS2 interfaces
  // ==========================================================

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    ground_truth_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    measurement_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;


  // ==========================================================
  // Delay buffer
  // ==========================================================

  std::deque<
    geometry_msgs::msg::PoseStamped>
    pose_buffer_;


  std::size_t sample_count_{0};
};


// ============================================================
// Main
// ============================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseUncertaintyInjector>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
