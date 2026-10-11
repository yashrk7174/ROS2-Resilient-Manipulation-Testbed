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
// Phase 6.3
// Quantitative Pose Estimation Validator
//
// Inputs:
//   /object_pose_ground_truth
//   /object_pose_measurement
//   /object_pose_estimate
//
// Outputs:
//   Terminal RMSE report
//   CSV dataset containing 500 matched evaluation samples
//
// IMPORTANT:
// Ground truth is used ONLY for validation.
// It is never provided to the estimator.
// ============================================================

class PoseEstimationValidator : public rclcpp::Node
{
public:
  PoseEstimationValidator()
  : Node("pose_estimation_validator")
  {
    ground_truth_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_ground_truth",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeGroundTruth(msg);
        });

    measurement_subscriber_ =
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object_pose_measurement",
        10,
        [this](
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          storeMeasurement(msg);
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


    // --------------------------------------------------------
    // CSV output
    // --------------------------------------------------------

    csv_file_.open(
      CSV_OUTPUT_PATH,
      std::ios::out |
      std::ios::trunc);

    if (!csv_file_.is_open())
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Could not open CSV output file: %s",
        CSV_OUTPUT_PATH.c_str());

      rclcpp::shutdown();

      return;
    }


    csv_file_
      << "sample_index,"
      << "timestamp_ns,"
      << "gt_x_m,gt_y_m,gt_z_m,"
      << "raw_x_m,raw_y_m,raw_z_m,"
      << "est_x_m,est_y_m,est_z_m,"
      << "raw_error_x_m,raw_error_y_m,raw_error_z_m,"
      << "est_error_x_m,est_error_y_m,est_error_z_m,"
      << "raw_position_error_m,"
      << "est_position_error_m\n";


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.3 — POSE ESTIMATION RMSE VALIDATION");

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
      "CSV output         : %s",
      CSV_OUTPUT_PATH.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "Samples matched by PoseStamped timestamp");

    RCLCPP_INFO(
      this->get_logger(),
      "Ground truth is used ONLY by validator");

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


  ~PoseEstimationValidator()
  {
    if (csv_file_.is_open())
    {
      csv_file_.flush();
      csv_file_.close();
    }
  }


private:

  static constexpr std::size_t WARMUP_SAMPLES = 50;
  static constexpr std::size_t EVALUATION_SAMPLES = 500;

  const std::string CSV_OUTPUT_PATH =
    "results/phase6_estimation/"
    "phase6_pose_estimation_samples.csv";


  struct PendingSample
  {
    bool has_ground_truth{false};
    bool has_measurement{false};
    bool has_estimate{false};

    geometry_msgs::msg::PoseStamped ground_truth;
    geometry_msgs::msg::PoseStamped measurement;
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

    sample.ground_truth = *msg;
    sample.has_ground_truth = true;

    tryProcessSample(key);
  }


  void storeMeasurement(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto key =
      timestampKey(*msg);

    auto & sample =
      pending_samples_[key];

    sample.measurement = *msg;
    sample.has_measurement = true;

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
      !sample.has_ground_truth ||
      !sample.has_measurement ||
      !sample.has_estimate)
    {
      return;
    }

    processMatchedSample(
      key,
      sample);

    pending_samples_.erase(
      iterator);
  }


  void processMatchedSample(
    std::int64_t timestamp_ns,
    const PendingSample & sample)
  {
    if (validation_complete_)
    {
      return;
    }

    ++matched_sample_count_;


    // --------------------------------------------------------
    // Kalman warm-up
    // --------------------------------------------------------

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
          "Warm-up complete — starting quantitative evaluation.");
      }

      return;
    }


    ++evaluation_sample_count_;


    // --------------------------------------------------------
    // Ground truth
    // --------------------------------------------------------

    const double gt_x =
      sample.ground_truth.pose.position.x;

    const double gt_y =
      sample.ground_truth.pose.position.y;

    const double gt_z =
      sample.ground_truth.pose.position.z;


    // --------------------------------------------------------
    // RAW position
    // --------------------------------------------------------

    const double raw_x =
      sample.measurement.pose.position.x;

    const double raw_y =
      sample.measurement.pose.position.y;

    const double raw_z =
      sample.measurement.pose.position.z;


    // --------------------------------------------------------
    // Estimated position
    // --------------------------------------------------------

    const double est_x =
      sample.estimate.pose.position.x;

    const double est_y =
      sample.estimate.pose.position.y;

    const double est_z =
      sample.estimate.pose.position.z;


    // --------------------------------------------------------
    // Per-axis errors
    // --------------------------------------------------------

    const double raw_error_x =
      raw_x - gt_x;

    const double raw_error_y =
      raw_y - gt_y;

    const double raw_error_z =
      raw_z - gt_z;


    const double est_error_x =
      est_x - gt_x;

    const double est_error_y =
      est_y - gt_y;

    const double est_error_z =
      est_z - gt_z;


    // --------------------------------------------------------
    // 3D Euclidean position errors
    // --------------------------------------------------------

    const double raw_position_error =
      std::sqrt(
        raw_error_x * raw_error_x +
        raw_error_y * raw_error_y +
        raw_error_z * raw_error_z);


    const double est_position_error =
      std::sqrt(
        est_error_x * est_error_x +
        est_error_y * est_error_y +
        est_error_z * est_error_z);


    // --------------------------------------------------------
    // RMSE accumulators
    // --------------------------------------------------------

    raw_squared_error_x_ +=
      raw_error_x * raw_error_x;

    raw_squared_error_y_ +=
      raw_error_y * raw_error_y;

    raw_squared_error_z_ +=
      raw_error_z * raw_error_z;


    estimate_squared_error_x_ +=
      est_error_x * est_error_x;

    estimate_squared_error_y_ +=
      est_error_y * est_error_y;

    estimate_squared_error_z_ +=
      est_error_z * est_error_z;


    // --------------------------------------------------------
    // Write synchronized sample to CSV
    // --------------------------------------------------------

    csv_file_
      << evaluation_sample_count_ << ","
      << timestamp_ns << ","

      << gt_x << ","
      << gt_y << ","
      << gt_z << ","

      << raw_x << ","
      << raw_y << ","
      << raw_z << ","

      << est_x << ","
      << est_y << ","
      << est_z << ","

      << raw_error_x << ","
      << raw_error_y << ","
      << raw_error_z << ","

      << est_error_x << ","
      << est_error_y << ","
      << est_error_z << ","

      << raw_position_error << ","
      << est_position_error
      << "\n";


    // Periodic flush + progress
    if (
      evaluation_sample_count_ %
      100 == 0)
    {
      csv_file_.flush();

      RCLCPP_INFO(
        this->get_logger(),
        "Evaluation progress: %zu / %zu matched samples",
        evaluation_sample_count_,
        EVALUATION_SAMPLES);
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


    const double raw_rmse_x =
      std::sqrt(
        raw_squared_error_x_ / n);

    const double raw_rmse_y =
      std::sqrt(
        raw_squared_error_y_ / n);

    const double raw_rmse_z =
      std::sqrt(
        raw_squared_error_z_ / n);


    const double est_rmse_x =
      std::sqrt(
        estimate_squared_error_x_ / n);

    const double est_rmse_y =
      std::sqrt(
        estimate_squared_error_y_ / n);

    const double est_rmse_z =
      std::sqrt(
        estimate_squared_error_z_ / n);


    const double raw_3d_rmse =
      std::sqrt(
        (
          raw_squared_error_x_ +
          raw_squared_error_y_ +
          raw_squared_error_z_
        ) / n);


    const double est_3d_rmse =
      std::sqrt(
        (
          estimate_squared_error_x_ +
          estimate_squared_error_y_ +
          estimate_squared_error_z_
        ) / n);


    const double improvement_percentage =
      100.0 *
      (
        raw_3d_rmse -
        est_3d_rmse
      ) /
      raw_3d_rmse;


    const bool x_pass =
      est_rmse_x < raw_rmse_x;

    const bool y_pass =
      est_rmse_y < raw_rmse_y;

    const bool z_pass =
      est_rmse_z < raw_rmse_z;

    const bool overall_pass =
      x_pass &&
      y_pass &&
      z_pass &&
      est_3d_rmse < raw_3d_rmse;


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.3 — QUANTITATIVE VALIDATION RESULT");

    RCLCPP_INFO(
      this->get_logger(),
      "Evaluation samples: %zu",
      evaluation_sample_count_);

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    RCLCPP_INFO(
      this->get_logger(),
      "RAW RMSE X : %.3f mm",
      raw_rmse_x * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "RAW RMSE Y : %.3f mm",
      raw_rmse_y * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "RAW RMSE Z : %.3f mm",
      raw_rmse_z * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "RAW 3D RMSE: %.3f mm",
      raw_3d_rmse * 1000.0);


    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    RCLCPP_INFO(
      this->get_logger(),
      "EST RMSE X : %.3f mm",
      est_rmse_x * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "EST RMSE Y : %.3f mm",
      est_rmse_y * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "EST RMSE Z : %.3f mm",
      est_rmse_z * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "EST 3D RMSE: %.3f mm",
      est_3d_rmse * 1000.0);


    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");

    RCLCPP_INFO(
      this->get_logger(),
      "3D RMSE reduction: %.2f %%",
      improvement_percentage);

    RCLCPP_INFO(
      this->get_logger(),
      "X improvement check: %s",
      x_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Y improvement check: %s",
      y_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "Z improvement check: %s",
      z_pass ? "PASS" : "FAIL");

    RCLCPP_INFO(
      this->get_logger(),
      "CSV saved: %s",
      CSV_OUTPUT_PATH.c_str());

    RCLCPP_INFO(
      this->get_logger(),
      "------------------------------------------------------------");


    if (overall_pass)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "PHASE 6.3 RMSE VALIDATION: PASS");
    }
    else
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "PHASE 6.3 RMSE VALIDATION: FAIL");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    ground_truth_subscriber_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    measurement_subscriber_;

  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    estimate_subscriber_;


  std::unordered_map<
    std::int64_t,
    PendingSample>
    pending_samples_;


  std::ofstream csv_file_;


  std::size_t matched_sample_count_{0};
  std::size_t evaluation_sample_count_{0};


  double raw_squared_error_x_{0.0};
  double raw_squared_error_y_{0.0};
  double raw_squared_error_z_{0.0};

  double estimate_squared_error_x_{0.0};
  double estimate_squared_error_y_{0.0};
  double estimate_squared_error_z_{0.0};


  bool validation_complete_{false};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseEstimationValidator>();

  rclcpp::spin(node);

  return 0;
}
