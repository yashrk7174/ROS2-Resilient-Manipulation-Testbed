#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;


// ============================================================
// Scalar Linear Kalman Filter
// ============================================================

class ScalarKalmanFilter
{
public:
  ScalarKalmanFilter(
    double process_noise_variance,
    double measurement_noise_variance,
    double initial_covariance)
  : process_noise_variance_(process_noise_variance),
    measurement_noise_variance_(measurement_noise_variance),
    covariance_(initial_covariance)
  {
  }


  void initialize(double measurement)
  {
    estimate_ = measurement;
    initialized_ = true;
    last_gain_ = 1.0;
  }


  bool initialized() const
  {
    return initialized_;
  }


  double predictedEstimate() const
  {
    return estimate_;
  }


  double predictedCovariance() const
  {
    return covariance_ + process_noise_variance_;
  }


  double innovationVariance() const
  {
    return
      predictedCovariance() +
      measurement_noise_variance_;
  }


  double update(double measurement)
  {
    const double predicted_estimate =
      predictedEstimate();

    const double predicted_covariance =
      predictedCovariance();

    last_gain_ =
      predicted_covariance /
      (
        predicted_covariance +
        measurement_noise_variance_
      );

    estimate_ =
      predicted_estimate +
      last_gain_ *
      (
        measurement -
        predicted_estimate
      );

    covariance_ =
      (1.0 - last_gain_) *
      predicted_covariance;

    return estimate_;
  }


  void predictionOnly()
  {
    // Random-walk model:
    // state value remains unchanged,
    // covariance grows because no measurement correction occurs.

    covariance_ =
      predictedCovariance();

    last_gain_ = 0.0;
  }


  double estimate() const
  {
    return estimate_;
  }


private:
  double process_noise_variance_;
  double measurement_noise_variance_;

  double estimate_{0.0};
  double covariance_{0.0};
  double last_gain_{0.0};

  bool initialized_{false};
};


// ============================================================
// Phase 6.6
// Robust Pose Filter
//
// Existing capability:
//   - Linear Kalman filtering
//   - 3D innovation gating
//
// New capability:
//   - stale-measurement rejection
//   - measurement-dropout watchdog
//   - prediction-only output during dropout
//
// Ground truth is NEVER used by estimator.
// ============================================================

class PoseFilter : public rclcpp::Node
{
public:
  PoseFilter()
  : Node("pose_filter"),
    x_filter_(
      PROCESS_NOISE_VARIANCE,
      MEASUREMENT_NOISE_VARIANCE,
      INITIAL_COVARIANCE),
    y_filter_(
      PROCESS_NOISE_VARIANCE,
      MEASUREMENT_NOISE_VARIANCE,
      INITIAL_COVARIANCE),
    z_filter_(
      PROCESS_NOISE_VARIANCE,
      MEASUREMENT_NOISE_VARIANCE,
      INITIAL_COVARIANCE)
  {
    measurement_subscriber_ =
      this->create_subscription<
        geometry_msgs::msg::PoseStamped>(
          "/object_pose_measurement",
          10,
          std::bind(
            &PoseFilter::measurementCallback,
            this,
            std::placeholders::_1));

    estimate_publisher_ =
      this->create_publisher<
        geometry_msgs::msg::PoseStamped>(
          "/object_pose_estimate",
          10);

    watchdog_timer_ =
      this->create_wall_timer(
        100ms,
        std::bind(
          &PoseFilter::watchdogCallback,
          this));


    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");

    RCLCPP_INFO(
      this->get_logger(),
      "PHASE 6.6 — ROBUST POSE FILTER");

    RCLCPP_INFO(
      this->get_logger(),
      "Kalman measurement sigma : %.1f mm",
      MEASUREMENT_NOISE_STDDEV * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "NIS threshold            : %.3f",
      NIS_THRESHOLD);

    RCLCPP_INFO(
      this->get_logger(),
      "Dropout timeout          : %.0f ms",
      DROPOUT_TIMEOUT_SECONDS * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Stale-data threshold     : %.0f ms",
      STALE_THRESHOLD_SECONDS * 1000.0);

    RCLCPP_INFO(
      this->get_logger(),
      "Ground truth is NOT used by estimator");

    RCLCPP_INFO(
      this->get_logger(),
      "============================================================");
  }


private:

  static constexpr double MEASUREMENT_NOISE_STDDEV =
    0.005;

  static constexpr double MEASUREMENT_NOISE_VARIANCE =
    MEASUREMENT_NOISE_STDDEV *
    MEASUREMENT_NOISE_STDDEV;

  static constexpr double PROCESS_NOISE_VARIANCE =
    1.0e-6;

  static constexpr double INITIAL_COVARIANCE =
    MEASUREMENT_NOISE_VARIANCE;

  static constexpr double NIS_THRESHOLD =
    11.345;

  // Allow normal 10 Hz timing jitter.
  static constexpr double DROPOUT_TIMEOUT_SECONDS =
    0.25;

  // Used in Phase 6.6B.
  static constexpr double STALE_THRESHOLD_SECONDS =
    0.30;


  void measurementCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr measurement)
  {
    const auto now =
      this->get_clock()->now();


    double receive_gap_seconds = 0.0;

    if (have_received_measurement_)
    {
      receive_gap_seconds =
        (
          now -
          last_measurement_receive_time_
        ).seconds();
    }


    // A message has physically arrived at this node.
    last_measurement_receive_time_ = now;

    have_received_measurement_ = true;

    ++measurement_count_;


    if (dropout_active_)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "MEASUREMENT RECOVERED | "
        "receive_gap=%.1f ms | "
        "dropout_events=%zu",
        receive_gap_seconds * 1000.0,
        dropout_event_count_);

      dropout_active_ = false;
      dropout_prediction_count_current_ = 0;
    }


    // --------------------------------------------------------
    // Measurement age
    // --------------------------------------------------------

    const rclcpp::Time measurement_stamp(
      measurement->header.stamp);

    double measurement_age_seconds =
      (
        now -
        measurement_stamp
      ).seconds();


    // Small negative values can occur from clock adjustment.
    if (measurement_age_seconds < 0.0)
    {
      measurement_age_seconds = 0.0;
    }


    // --------------------------------------------------------
    // Reject stale data
    // --------------------------------------------------------

    if (
      measurement_age_seconds >
      STALE_THRESHOLD_SECONDS)
    {
      ++stale_measurement_count_;

      if (filtersInitialized())
      {
        predictionOnly();

        publishCurrentEstimate(now);
      }


      RCLCPP_WARN(
        this->get_logger(),
        "STALE REJECTED | "
        "age=%.1f ms > %.1f ms | "
        "stale_total=%zu",
        measurement_age_seconds * 1000.0,
        STALE_THRESHOLD_SECONDS * 1000.0,
        stale_measurement_count_);

      return;
    }


    // --------------------------------------------------------
    // First valid measurement initializes estimator
    // --------------------------------------------------------

    if (!filtersInitialized())
    {
      x_filter_.initialize(
        measurement->pose.position.x);

      y_filter_.initialize(
        measurement->pose.position.y);

      z_filter_.initialize(
        measurement->pose.position.z);


      last_orientation_ =
        measurement->pose.orientation;

      last_frame_id_ =
        measurement->header.frame_id;


      publishEstimate(
        measurement->header,
        x_filter_.estimate(),
        y_filter_.estimate(),
        z_filter_.estimate());

      RCLCPP_INFO(
        this->get_logger(),
        "FILTER INITIALIZED | "
        "EST [%.3f %.3f %.3f]",
        x_filter_.estimate(),
        y_filter_.estimate(),
        z_filter_.estimate());

      return;
    }


    last_orientation_ =
      measurement->pose.orientation;

    last_frame_id_ =
      measurement->header.frame_id;


    // --------------------------------------------------------
    // Innovation
    // --------------------------------------------------------

    const double innovation_x =
      measurement->pose.position.x -
      x_filter_.predictedEstimate();

    const double innovation_y =
      measurement->pose.position.y -
      y_filter_.predictedEstimate();

    const double innovation_z =
      measurement->pose.position.z -
      z_filter_.predictedEstimate();


    const double s_x =
      x_filter_.innovationVariance();

    const double s_y =
      y_filter_.innovationVariance();

    const double s_z =
      z_filter_.innovationVariance();


    const double nis =
      (
        innovation_x * innovation_x / s_x
      )
      +
      (
        innovation_y * innovation_y / s_y
      )
      +
      (
        innovation_z * innovation_z / s_z
      );


    // --------------------------------------------------------
    // Outlier rejection from Phase 6.5
    // --------------------------------------------------------

    if (nis > NIS_THRESHOLD)
    {
      ++outlier_rejection_count_;

      predictionOnly();

      publishEstimate(
        measurement->header,
        x_filter_.estimate(),
        y_filter_.estimate(),
        z_filter_.estimate());


      RCLCPP_WARN(
        this->get_logger(),
        "OUTLIER REJECTED | "
        "NIS=%.2f > %.3f | "
        "RAW [%.3f %.3f %.3f] | "
        "EST [%.3f %.3f %.3f]",
        nis,
        NIS_THRESHOLD,
        measurement->pose.position.x,
        measurement->pose.position.y,
        measurement->pose.position.z,
        x_filter_.estimate(),
        y_filter_.estimate(),
        z_filter_.estimate());

      return;
    }


    // --------------------------------------------------------
    // Normal Kalman update
    // --------------------------------------------------------

    const double estimated_x =
      x_filter_.update(
        measurement->pose.position.x);

    const double estimated_y =
      y_filter_.update(
        measurement->pose.position.y);

    const double estimated_z =
      z_filter_.update(
        measurement->pose.position.z);


    publishEstimate(
      measurement->header,
      estimated_x,
      estimated_y,
      estimated_z);


    if (measurement_count_ % 10 == 0)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "UPDATE | measurement=%zu | "
        "age=%.1f ms | "
        "EST [%.3f %.3f %.3f]",
        measurement_count_,
        measurement_age_seconds * 1000.0,
        estimated_x,
        estimated_y,
        estimated_z);
    }
  }


  // ==========================================================
  // Missing-measurement watchdog
  // ==========================================================

  void watchdogCallback()
  {
    if (
      !have_received_measurement_ ||
      !filtersInitialized())
    {
      return;
    }


    const auto now =
      this->get_clock()->now();


    const double time_since_measurement =
      (
        now -
        last_measurement_receive_time_
      ).seconds();


    if (
      time_since_measurement <=
      DROPOUT_TIMEOUT_SECONDS)
    {
      return;
    }


    if (!dropout_active_)
    {
      dropout_active_ = true;

      ++dropout_event_count_;

      dropout_prediction_count_current_ = 0;


      RCLCPP_WARN(
        this->get_logger(),
        "DROPOUT DETECTED | "
        "no measurement for %.1f ms | "
        "event=%zu",
        time_since_measurement * 1000.0,
        dropout_event_count_);
    }


    predictionOnly();

    ++dropout_prediction_count_current_;
    ++dropout_prediction_count_total_;


    publishCurrentEstimate(now);


    RCLCPP_WARN(
      this->get_logger(),
      "PREDICTION ONLY | "
      "dropout_step=%zu | "
      "EST [%.3f %.3f %.3f]",
      dropout_prediction_count_current_,
      x_filter_.estimate(),
      y_filter_.estimate(),
      z_filter_.estimate());
  }


  void predictionOnly()
  {
    x_filter_.predictionOnly();
    y_filter_.predictionOnly();
    z_filter_.predictionOnly();
  }


  bool filtersInitialized() const
  {
    return
      x_filter_.initialized() &&
      y_filter_.initialized() &&
      z_filter_.initialized();
  }


  void publishCurrentEstimate(
    const rclcpp::Time & now)
  {
    std_msgs::msg::Header header;

    header.stamp = now;

    header.frame_id =
      last_frame_id_.empty()
      ? "world"
      : last_frame_id_;


    publishEstimate(
      header,
      x_filter_.estimate(),
      y_filter_.estimate(),
      z_filter_.estimate());
  }


  void publishEstimate(
    const std_msgs::msg::Header & header,
    double estimated_x,
    double estimated_y,
    double estimated_z)
  {
    geometry_msgs::msg::PoseStamped estimate;

    estimate.header = header;

    estimate.pose.position.x =
      estimated_x;

    estimate.pose.position.y =
      estimated_y;

    estimate.pose.position.z =
      estimated_z;

    estimate.pose.orientation =
      last_orientation_;


    estimate_publisher_->publish(
      estimate);
  }


  ScalarKalmanFilter x_filter_;
  ScalarKalmanFilter y_filter_;
  ScalarKalmanFilter z_filter_;


  rclcpp::Subscription<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    measurement_subscriber_;

  rclcpp::Publisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr
    estimate_publisher_;

  rclcpp::TimerBase::SharedPtr
    watchdog_timer_;


  rclcpp::Time
    last_measurement_receive_time_;


  geometry_msgs::msg::Quaternion
    last_orientation_;

  std::string
    last_frame_id_{"world"};


  bool have_received_measurement_{false};
  bool dropout_active_{false};


  std::size_t measurement_count_{0};

  std::size_t outlier_rejection_count_{0};
  std::size_t stale_measurement_count_{0};

  std::size_t dropout_event_count_{0};

  std::size_t dropout_prediction_count_current_{0};
  std::size_t dropout_prediction_count_total_{0};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<PoseFilter>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
