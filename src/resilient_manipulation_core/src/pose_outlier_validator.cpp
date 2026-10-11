#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


// ============================================================
// Phase 6.5B
// Quantitative Outlier-Rejection Validator
//
// Inputs:
//   /object_pose_ground_truth
//   /object_pose_measurement
//   /object_pose_estimate
//
// Ground truth is used ONLY for validation.
//
// Purpose:
//   Quantify the effect of deterministic gross pose outliers
//   before and after innovation-gated Kalman filtering.
// ============================================================

class PoseOutlierValidator : public rclcpp::Node
{
public:
  PoseOutlierValidator()
  : Node("pose_outlier_validator")
  {
    gt_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_ground_truth",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeGroundTruth(msg);
        });

    raw_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_measurement",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeRaw(msg);
        });

    estimate_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_estimate",
        10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeEstimate(msg);
        });


    csv_file_.open(
      CSV_OUTPUT_PATH,
      std::ios::out | std::ios::trunc);

    if (!csv_file_.is_open())
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Could not open CSV: %s",
        CSV_OUTPUT_PATH.c_str());

      rclcpp::shutdown();
      return;
    }


    csv_file_
      << "sample_index,"
      << "raw_position_error_m,"
      << "estimate_position_error_m,"
      << "raw_outlier_detected,"
      << "estimate_corrupted\n";


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.5B — QUANTITATIVE OUTLIER VALIDATION");

    RCLCPP_INFO(
      this->get_logger(),
      "Evaluation samples     : %zu",
      EVALUATION_SAMPLES);

    RCLCPP_INFO(
      this->get_logger(),
      "Raw outlier threshold  : %.1f mm",
      RAW_OUTLIER_THRESHOLD * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Estimate error limit   : %.1f mm",
      ESTIMATE_ERROR_LIMIT * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "CSV output             : %s",
      CSV_OUTPUT_PATH.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


  ~PoseOutlierValidator()
  {
    if (csv_file_.is_open())
    {
      csv_file_.flush();
      csv_file_.close();
    }
  }


private:

  static constexpr std::size_t EVALUATION_SAMPLES = 400;

  // Normal samples are exactly at GT in this controlled test.
  // Gross outlier magnitude is about 70.7 mm.
  static constexpr double RAW_OUTLIER_THRESHOLD =
    0.020;  // 20 mm

  // Estimate should remain close to GT after rejection.
  static constexpr double ESTIMATE_ERROR_LIMIT =
    0.005;  // 5 mm


  const std::string CSV_OUTPUT_PATH =
    "results/phase6_outlier/"
    "phase6_outlier_validation_samples.csv";


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
      static_cast<std::int64_t>(msg.header.stamp.sec)
      * 1000000000LL
      +
      static_cast<std::int64_t>(msg.header.stamp.nanosec);
  }


  void storeGroundTruth(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key = timestampKey(*msg);

    auto & sample = pending_samples_[key];

    sample.gt = *msg;
    sample.has_gt = true;

    tryProcessSample(key);
  }


  void storeRaw(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key = timestampKey(*msg);

    auto & sample = pending_samples_[key];

    sample.raw = *msg;
    sample.has_raw = true;

    tryProcessSample(key);
  }


  void storeEstimate(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key = timestampKey(*msg);

    auto & sample = pending_samples_[key];

    sample.estimate = *msg;
    sample.has_estimate = true;

    tryProcessSample(key);
  }


  void tryProcessSample(std::int64_t key)
  {
    const auto iterator =
      pending_samples_.find(key);

    if (iterator == pending_samples_.end())
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

    processMatchedSample(sample);

    pending_samples_.erase(iterator);
  }


  void processMatchedSample(
    const PendingSample & sample)
  {
    if (validation_complete_)
    {
      return;
    }

    ++evaluation_sample_count_;


    const double gt_x =
      sample.gt.pose.position.x;

    const double gt_y =
      sample.gt.pose.position.y;

    const double gt_z =
      sample.gt.pose.position.z;


    const double raw_dx =
      sample.raw.pose.position.x - gt_x;

    const double raw_dy =
      sample.raw.pose.position.y - gt_y;

    const double raw_dz =
      sample.raw.pose.position.z - gt_z;


    const double est_dx =
      sample.estimate.pose.position.x - gt_x;

    const double est_dy =
      sample.estimate.pose.position.y - gt_y;

    const double est_dz =
      sample.estimate.pose.position.z - gt_z;


    const double raw_position_error =
      std::sqrt(
        raw_dx * raw_dx +
        raw_dy * raw_dy +
        raw_dz * raw_dz);


    const double estimate_position_error =
      std::sqrt(
        est_dx * est_dx +
        est_dy * est_dy +
        est_dz * est_dz);


    const bool raw_outlier =
      raw_position_error >
      RAW_OUTLIER_THRESHOLD;


    const bool estimate_corrupted =
      estimate_position_error >
      ESTIMATE_ERROR_LIMIT;


    if (raw_outlier)
    {
      ++raw_outlier_count_;

      if (!estimate_corrupted)
      {
        ++protected_outlier_count_;
      }
    }


    if (estimate_corrupted)
    {
      ++estimate_corrupted_count_;
    }


    raw_squared_error_sum_ +=
      raw_position_error *
      raw_position_error;

    estimate_squared_error_sum_ +=
      estimate_position_error *
      estimate_position_error;


    csv_file_
      << evaluation_sample_count_ << ","
      << raw_position_error << ","
      << estimate_position_error << ","
      << (raw_outlier ? 1 : 0) << ","
      << (estimate_corrupted ? 1 : 0)
      << "\n";


    if (evaluation_sample_count_ % 100 == 0)
    {
      csv_file_.flush();

      RCLCPP_INFO(
        this->get_logger(),
        "Progress: %zu / %zu | "
        "raw_outliers=%zu | "
        "protected=%zu",
        evaluation_sample_count_,
        EVALUATION_SAMPLES,
        raw_outlier_count_,
        protected_outlier_count_);
    }


    if (
      evaluation_sample_count_ >=
      EVALUATION_SAMPLES)
    {
      reportResults();

      csv_file_.flush();

      validation_complete_ = true;

      rclcpp::shutdown();
    }
  }


  void reportResults()
  {
    const double n =
      static_cast<double>(
        evaluation_sample_count_);


    const double raw_rmse =
      std::sqrt(
        raw_squared_error_sum_ / n);

    const double estimate_rmse =
      std::sqrt(
        estimate_squared_error_sum_ / n);


    const double rmse_reduction =
      100.0 *
      (
        raw_rmse -
        estimate_rmse
      ) /
      raw_rmse;


    const double protection_rate =
      raw_outlier_count_ > 0
      ?
      100.0 *
      static_cast<double>(
        protected_outlier_count_)
      /
      static_cast<double>(
        raw_outlier_count_)
      :
      0.0;


    const bool expected_outlier_count_pass =
      raw_outlier_count_ == EXPECTED_OUTLIERS;

    const bool protection_pass =
      protected_outlier_count_ ==
      raw_outlier_count_;

    const bool no_estimate_corruption_pass =
      estimate_corrupted_count_ == 0;

    const bool rmse_pass =
      estimate_rmse < raw_rmse;


    const bool overall_pass =
      expected_outlier_count_pass &&
      protection_pass &&
      no_estimate_corruption_pass &&
      rmse_pass;


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.5B — OUTLIER VALIDATION RESULT");

    RCLCPP_INFO(
      this->get_logger(),
      "Evaluation samples       : %zu",
      evaluation_sample_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "Expected raw outliers    : %zu",
      EXPECTED_OUTLIERS);

    RCLCPP_INFO(
      this->get_logger(),
      "Detected raw outliers    : %zu",
      raw_outlier_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "Protected outliers       : %zu",
      protected_outlier_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "Estimate corruptions     : %zu",
      estimate_corrupted_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "Protection rate          : %.2f %%",
      protection_rate);

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");

    RCLCPP_INFO(
      this->get_logger(),
      "RAW 3D RMSE              : %.3f mm",
      raw_rmse * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "GATED EST 3D RMSE        : %.3f mm",
      estimate_rmse * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "3D RMSE reduction        : %.2f %%",
      rmse_reduction);

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");

    RCLCPP_INFO(
      this->get_logger(),
      "Outlier-count check      : %s",
      expected_outlier_count_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Protection check         : %s",
      protection_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Estimate corruption check: %s",
      no_estimate_corruption_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "RMSE improvement check   : %s",
      rmse_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "CSV saved                : %s",
      CSV_OUTPUT_PATH.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    if (overall_pass)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "PHASE 6.5B OUTLIER VALIDATION: PASS");
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "PHASE 6.5B OUTLIER VALIDATION: FAIL");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


  static constexpr std::size_t EXPECTED_OUTLIERS =
    EVALUATION_SAMPLES / 20;


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


  std::ofstream csv_file_;


  std::size_t evaluation_sample_count_{0};

  std::size_t raw_outlier_count_{0};
  std::size_t protected_outlier_count_{0};
  std::size_t estimate_corrupted_count_{0};


  double raw_squared_error_sum_{0.0};
  double estimate_squared_error_sum_{0.0};


  bool validation_complete_{false};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseOutlierValidator>();

  rclcpp::spin(node);

  return 0;
}
