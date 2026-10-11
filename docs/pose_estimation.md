# Robust Object-Pose Estimation

## 1. Purpose

The manipulation system does not use the noisy object-pose measurement
directly for motion planning.

The estimation pipeline is:

Ground Truth
    ↓
Measurement uncertainty / faults
    ↓
/object_pose_measurement
    ↓
Robust Kalman Filter
    ↓
/object_pose_estimate
    ↓
Validity / freshness gate
    ↓
MoveIt manipulation planning


## 2. State Definition

Only object position is estimated.

For each Cartesian axis:

x ∈ {X, Y, Z}

a separate scalar Kalman filter is used.

Orientation is not estimated in the current project scope.


## 3. Process Model

The object position is modeled as a random walk:

x_k = x_(k-1) + w_k

where:

w_k ~ N(0, Q)

The predicted state is therefore:

x̂_k^- = x̂_(k-1)

and the predicted covariance is:

P_k^- = P_(k-1) + Q


## 4. Measurement Model

The measured object position is:

z_k = x_k + v_k

where:

v_k ~ N(0, R)

The measurement directly observes the position state.


## 5. Noise Parameters

Process-noise variance:

Q = 1 × 10^-6 m²

Measurement-noise standard deviation:

σ = 0.005 m

Therefore:

R = σ²

R = (0.005)²

R = 2.5 × 10^-5 m²


## 6. Innovation

Before correcting the estimate, the measurement residual is calculated:

y_k = z_k - x̂_k^-

The innovation covariance is:

S_k = P_k^- + R


## 7. Innovation / Outlier Gate

The three Cartesian innovations are combined using a normalized
innovation-squared statistic:

NIS =
(y_x² / S_x)
+
(y_y² / S_y)
+
(y_z² / S_z)

The project uses:

NIS threshold = 11.345

Measurements above this threshold are rejected before the Kalman update.


## 8. Kalman Gain

For an accepted measurement:

K_k = P_k^- / (P_k^- + R)

The steady-state Kalman gain for the verified project configuration
is approximately:

K ≈ 0.181


## 9. State Correction

For every accepted Cartesian measurement:

x̂_k =
x̂_k^-
+
K_k (z_k - x̂_k^-)

The covariance is updated as:

P_k =
(1 - K_k) P_k^-


## 10. Rejected / Missing Measurements

If a measurement is:

- an outlier,
- stale,
- unavailable during dropout,

the measurement correction is skipped.

The estimator remains in prediction-only mode.

For the random-walk model:

x̂_k = x̂_k^-

while the uncertainty grows through:

P_k^- = P_(k-1) + Q


## 11. Additional Robustness Logic

Raw stale-measurement rejection threshold:

300 ms

Measurement-dropout watchdog threshold:

250 ms

Planner-side estimator-output freshness threshold:

500 ms


## 12. Verified Gaussian-Noise Result

Validation dataset:

500 synchronized samples

Raw measurement 3D RMSE:

9.040 mm

Kalman-filtered estimate 3D RMSE:

2.784 mm

Reduction:

69.20 %


## 13. Outlier Validation

Controlled outlier scenario:

20 injected outliers

Detected:

20 / 20

Protected estimate corruptions:

0

This result applies to the controlled isolated outlier-validation scenario.


## 14. Known Limitation — Constant Bias

A constant +10 mm X-axis bias was not removed.

Reason:

The estimator state contains position only and has no explicit sensor-bias
state.

Therefore an unknown constant sensor offset cannot be distinguished from
a true position displacement.


## 15. Why a Linear Kalman Filter Was Used

The process model is linear:

x_k = x_(k-1) + w_k

and the measurement model is linear:

z_k = x_k + v_k

Therefore an EKF is unnecessary for the current position-only model.

Using a standard linear Kalman filter keeps the estimator mathematically
appropriate and easier to verify.


## 16. Estimator-to-Planner Interface

The estimator publishes:

/object_pose_estimate

Before MoveIt can use the estimate, the manipulation node validates:

1. frame matches the MoveIt planning frame,
2. X/Y/Z are finite,
3. estimator-output age is less than or equal to 500 ms.

Only after these checks pass is the estimated position converted into
pre-grasp, grasp and retreat targets.


## 17. Engineering Claim

This project implements a position-only robust Kalman filtering pipeline
with innovation-based outlier rejection, stale-data rejection, dropout
handling and downstream MoveIt integration.

MoveIt / OMPL provides the underlying motion planner.
No custom sampling-based motion planner is claimed.
