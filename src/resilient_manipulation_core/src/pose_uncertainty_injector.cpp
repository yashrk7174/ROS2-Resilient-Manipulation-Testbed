#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;


// ============================================================
// Phase 7.6B Test Source
// Changed Target + Gaussian Measurement Noise
//
// True object position:
//   [0.387, 0.050, 0.470] m
//
// Measurement noise:
//   zero-mean Gaussian
//   sigma = 5 mm per XYZ axis
//
// Purpose:
//   Verify that the manipulation mission consumes the
//   filtered /object_pose_estimate rather than raw jitter.
//
// No systematic bias.
// No gross outlier.
// No dropout.
// No stale timestamp.
// ============================================================

class PoseUncertaintyInjector : public rclcpp::Node
{
public:
  PoseUncertaintyInjector()
  : Node("pose_uncertainty_injector"),
    generator_(42),
    noise_distribution_(0.0, NOISE_STDDEV)
  {
    ground_truth_publisher_ =
      this->create_publisher<
        geometry_msgs::msg::PoseStamped>(
          "/object_pose_ground_truth",
          10);

    measurement_publisher_ =
      this->create_publisher<
        geometry_msgs::msg::PoseStamped>(
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
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 7.6B — NOISY ESTIMATED TARGET TEST");

    RCLCPP_INFO(
      this->get_logger(),
      "True target       : [%.3f %.3f %.3f] m",
      TARGET_X,
      TARGET_Y,
      TARGET_Z);

    RCLCPP_INFO(
      this->get_logger(),
      "Gaussian sigma    : %.1f mm / axis",
      NOISE_STDDEV * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Random seed       : 42");

    RCLCPP_INFO(
      this->get_logger(),
      "Publish rate      : 10 Hz");

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


private:

  static constexpr double TARGET_X = 0.387;
  static constexpr double TARGET_Y = 0.050;
  static constexpr double TARGET_Z = 0.470;

  static constexpr double NOISE_STDDEV = 0.005;


  void publishPose()
  {
    ++sample_count_;


    geometry_msgs::msg::PoseStamped ground_truth;

    ground_truth.header.stamp =
      this->get_clock()->now();

    ground_truth.header.frame_id =
      "world";

    ground_truth.pose.position.x =
      TARGET_X;

    ground_truth.pose.position.y =
      TARGET_Y;

    ground_truth.pose.position.z =
      TARGET_Z;

    ground_truth.pose.orientation.x = 0.0;
    ground_truth.pose.orientation.y = 0.0;
    ground_truth.pose.orientation.z = 0.0;
    ground_truth.pose.orientation.w = 1.0;


    geometry_msgs::msg::PoseStamped measurement =
      ground_truth;


    measurement.pose.position.x +=
      noise_distribution_(generator_);

    measurement.pose.position.y +=
      noise_distribution_(generator_);

    measurement.pose.position.z +=
      noise_distribution_(generator_);


    const double dx =
      measurement.pose.position.x -
      ground_truth.pose.position.x;

    const double dy =
      measurement.pose.position.y -
      ground_truth.pose.position.y;

    const double dz =
      measurement.pose.position.z -
      ground_truth.pose.position.z;


    const double raw_error =
      std::sqrt(
        dx * dx +
        dy * dy +
        dz * dz);


    ground_truth_publisher_->publish(
      ground_truth);

    measurement_publisher_->publish(
      measurement);


    if (sample_count_ % 10 == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "sample=%zu | "
        "GT [%.3f %.3f %.3f] | "
        "RAW [%.3f %.3f %.3f] | "
        "raw_error=%.2f mm",
        sample_count_,
        ground_truth.pose.position.x,
        ground_truth.pose.position.y,
        ground_truth.pose.position.z,
        measurement.pose.position.x,
        measurement.pose.position.y,
        measurement.pose.position.z,
        raw_error * 1000.0);
    }
  }


  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    ground_truth_publisher_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    measurement_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;


  std::mt19937 generator_;

  std::normal_distribution<double>
    noise_distribution_;


  std::size_t sample_count_{0};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseUncertaintyInjector>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
