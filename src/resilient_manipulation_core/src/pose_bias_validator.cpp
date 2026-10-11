#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


// ============================================================
// Phase 6.4
// Systematic Bias Limitation Validator
//
// Purpose:
// Prove that the current position-only Kalman filter does not
// automatically eliminate an unknown constant sensor bias.
//
// Inputs:
//   /object_pose_ground_truth
//   /object_pose_measurement
//   /object_pose_estimate
//
// Ground truth is used ONLY for validation.
// ============================================================

class PoseBiasValidator : public rclcpp::Node
{
public:
  PoseBiasValidator()
  : Node("pose_bias_validator")
  {
    gt_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_ground_truth",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeGroundTruth(msg);
        });

    raw_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_measurement",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeRaw(msg);
        });

    estimate_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_estimate",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeEstimate(msg);
        });


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.4 — SYSTEMATIC BIAS LIMITATION VALIDATION");

    RCLCPP_INFO(
      this->get_logger(),
      "Warm-up samples    : %zu",
      WARMUP_SAMPLES);

    RCLCPP_INFO(
      this->get_logger(),
      "Evaluation samples : %zu",
      EVALUATION_SAMPLES);

    RCLCPP_INFO(
      this->get_logger(),
      "Expected bias      : %.1f mm",
      EXPECTED_X_BIAS * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Goal: demonstrate bias remains after filtering");

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


private:

  static constexpr std::size_t WARMUP_SAMPLES = 50;
  static constexpr std::size_t EVALUATION_SAMPLES = 300;

  static constexpr double EXPECTED_X_BIAS =
    0.010;

  static constexpr double BIAS_TOLERANCE =
    0.0005;   // 0.5 mm


  struct PendingSample
  {
    bool has_gt{false};
    bool has_raw{false};
    bool has_estimate{false};

    geometry_msgs::msg::PoseStamped gt;
    geometry_msgs::msg::PoseStamped raw;
    geometry_msgs::msg::PoseStamped estimate;
  };


  std::int64_t timestampKey(
    const geometry_msgs::msg::PoseStamped & msg) const
  {
    return
      static_cast<std::int64_t>(
        msg.header.stamp.sec) *
        1000000000LL
      +
      static_cast<std::int64_t>(
        msg.header.stamp.nanosec);
  }


  void storeGroundTruth(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key =
      timestampKey(*msg);

    auto & sample =
      pending_samples_[key];

    sample.gt = *msg;
    sample.has_gt = true;

    tryProcessSample(key);
  }


  void storeRaw(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key =
      timestampKey(*msg);

    auto & sample =
      pending_samples_[key];

    sample.raw = *msg;
    sample.has_raw = true;

    tryProcessSample(key);
  }


  void storeEstimate(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key =
      timestampKey(*msg);

    auto & sample =
      pending_samples_[key];

    sample.estimate = *msg;
    sample.has_estimate = true;

    tryProcessSample(key);
  }


  void tryProcessSample(
    std::int64_t key)
  {
    const auto iterator =
      pending_samples_.find(key);

    if (iterator ==
        pending_samples_.end())
    {
      return;
    }

    const auto & sample =
      iterator->second;

    if (
      !sample.has_gt ||
      !sample.has_raw ||
      !sample.has_estimate)
    {
      return;
    }


    processMatchedSample(
      sample);

    pending_samples_.erase(
      iterator);
  }


  void processMatchedSample(
    const PendingSample & sample)
  {
    if (validation_complete_)
    {
      return;
    }


    ++matched_sample_count_;


    if (
      matched_sample_count_ <=
      WARMUP_SAMPLES)
    {
      if (
        matched_sample_count_ ==
        WARMUP_SAMPLES)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "Warm-up complete — starting bias evaluation.");
      }

      return;
    }


    ++evaluation_sample_count_;


    const double gt_x =
      sample.gt.pose.position.x;

    const double raw_x =
      sample.raw.pose.position.x;

    const double est_x =
      sample.estimate.pose.position.x;


    const double raw_error_x =
      raw_x - gt_x;

    const double est_error_x =
      est_x - gt_x;


    raw_error_sum_ +=
      raw_error_x;

    estimate_error_sum_ +=
      est_error_x;


    raw_squared_error_sum_ +=
      raw_error_x * raw_error_x;

    estimate_squared_error_sum_ +=
      est_error_x * est_error_x;


    if (
      evaluation_sample_count_ %
      100 == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Evaluation progress: %zu / %zu",
        evaluation_sample_count_,
        EVALUATION_SAMPLES);
    }


    if (
      evaluation_sample_count_ >=
      EVALUATION_SAMPLES)
    {
      reportResults();

      validation_complete_ =
        true;

      rclcpp::shutdown();
    }
  }


  void reportResults()
  {
    const double n =
      static_cast<double>(
        evaluation_sample_count_);


    const double raw_mean_error =
      raw_error_sum_ / n;

    const double est_mean_error =
      estimate_error_sum_ / n;


    const double raw_rmse =
      std::sqrt(
        raw_squared_error_sum_ / n);

    const double est_rmse =
      std::sqrt(
        estimate_squared_error_sum_ / n);


    const double bias_difference =
      std::abs(
        est_mean_error -
        raw_mean_error);


    const bool raw_bias_present =
      std::abs(
        raw_mean_error -
        EXPECTED_X_BIAS)
      <= BIAS_TOLERANCE;


    const bool estimate_bias_remains =
      std::abs(
        est_mean_error -
        EXPECTED_X_BIAS)
      <= BIAS_TOLERANCE;


    const bool bias_not_removed =
      bias_difference
      <= BIAS_TOLERANCE;


    const bool overall_pass =
      raw_bias_present &&
      estimate_bias_remains &&
      bias_not_removed;


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.4 — BIAS LIMITATION RESULT");

    RCLCPP_INFO(
      this->get_logger(),
      "Evaluation samples : %zu",
      evaluation_sample_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    RCLCPP_INFO(
      this->get_logger(),
      "RAW mean X error    : %.3f mm",
      raw_mean_error * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "EST mean X error    : %.3f mm",
      est_mean_error * 1000.0);


    RCLCPP_INFO(
      this->get_logger(),
      "RAW X RMSE          : %.3f mm",
      raw_rmse * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "EST X RMSE          : %.3f mm",
      est_rmse * 1000.0);


    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    RCLCPP_INFO(
      this->get_logger(),
      "Injected bias check : %s",
      raw_bias_present ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Bias remains in EST : %s",
      estimate_bias_remains ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Bias not removed    : %s",
      bias_not_removed ? "PASS" : "FAIL");


    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    if (overall_pass)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "PHASE 6.4 LIMITATION DEMONSTRATION: PASS");
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "PHASE 6.4 LIMITATION DEMONSTRATION: FAIL");
    }


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    gt_subscriber_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    raw_subscriber_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    estimate_subscriber_;


  std::unordered_map<
    std::int64_t,
    PendingSample>
    pending_samples_;


  std::size_t matched_sample_count_{0};
  std::size_t evaluation_sample_count_{0};


  double raw_error_sum_{0.0};
  double estimate_error_sum_{0.0};

  double raw_squared_error_sum_{0.0};
  double estimate_squared_error_sum_{0.0};


  bool validation_complete_{false};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseBiasValidator>();

  rclcpp::spin(node);

  return 0;
}
