# ROS2 Resilient Manipulation & Motion Planning Testbed

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E.svg)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Motion%20Planning-2C5F9E.svg)](https://moveit.picknik.ai/)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![Validation](https://img.shields.io/badge/Engineering%20Validation-8%2F8%20PASS-success.svg)

**Collision-aware manipulation, robust object-pose estimation and bounded recovery under controlled perception and execution faults using ROS 2, MoveIt 2 and C++.**

This project investigates a practical robotics problem:

> **What happens when a manipulation system receives noisy, stale, delayed or corrupted object-pose information — and how can the software recover without hiding persistent failures?**

The testbed combines controlled perception-fault injection, Kalman filtering, outlier rejection, MoveIt 2 planning, manipulation sequencing, bounded recovery and automated engineering validation on a simulated Franka Emika Panda manipulator.

---

## Key results

| Metric / scenario | Baseline | Resilient system |
|---|---:|---:|
| 3D pose RMSE under Gaussian noise | 9.040 mm | **2.784 mm** |
| RMSE reduction | — | **69.20%** |
| Controlled injected outliers protected | 0 / 20 | **20 / 20** |
| Temporary stale pose | SAFE_ABORT | **RECOVERED** |
| Single planning failure | NO_RECOVERY | **RECOVERED on attempt 2** |
| Persistent planning failure | Abort after 1 attempt | **Bounded SAFE_ABORT after max. 2 attempts** |
| Automatic execution retry | 0 | **0** |
| Automated regression | — | **6 / 6 scenarios PASS** |
| Engineering requirements | — | **8 / 8 PASS** |

Pose-estimation RMSE was evaluated over **500 synchronized samples**.

The 20/20 outlier result refers only to the defined controlled validation scenario and is not a universal outlier-detection claim.

---

## System architecture

```text
Ground-truth object pose
          │
          ▼
Controlled perception uncertainty
          │
          ├── Gaussian noise
          ├── systematic bias
          ├── outliers
          ├── dropout
          └── delay
          │
          ▼
 /object_pose_measurement
          │
          ├─────────────────────────────┐
          │                             │
          ▼                             │
Robust pose estimator                   │
          │                             │
          ├── linear Kalman filter      │
          ├── NIS outlier gate          │
          ├── stale-sample rejection    │
          └── dropout watchdog          │
          │                             │
          ▼                             ▼
 /object_pose_estimate            Raw-pose baseline
          │                             │
          └──────────────┬──────────────┘
                         ▼
                 Pose validity gate
                         │
                         ▼
                  MoveIt 2 planning
                         │
                         ▼
                Manipulation mission
                         │
                         ▼
           Failure detection / recovery
```

---

## Manipulation mission

![Manipulation mission](docs/evidence/manipulation_mission_execution.png)

The mission executes:

```text
Ready
  ↓
Open gripper
  ↓
Estimate-driven pre-grasp
  ↓
Cartesian approach
  ↓
Close gripper
  ↓
Cartesian retreat
  ↓
Return to ready
  ↓
Mission verification
```

The pre-grasp, grasp and retreat targets are generated from the accepted object-pose input rather than from a fixed grasp position.

---

## Pose estimation

The current estimator uses three independent scalar random-walk Kalman filters for Cartesian position.

A standard linear Kalman filter is used because the implemented process and measurement models are linear; an EKF/UKF would add unnecessary complexity for the current scope.

Verified parameters:

| Parameter | Value |
|---|---:|
| Process-noise variance `Q` | `1e-6 m²` |
| Measurement-noise variance `R` | `2.5e-5 m²` |
| Initial covariance | `R` |
| Approx. steady-state Kalman gain | `0.181` |
| NIS rejection threshold | `11.345` |
| Stale raw-sample threshold | `300 ms` |
| Dropout watchdog | `250 ms` |

### Measured pose accuracy

| Component | Raw RMSE | Filtered RMSE |
|---|---:|---:|
| X | 5.050 mm | **1.651 mm** |
| Y | 5.240 mm | **1.772 mm** |
| Z | 5.363 mm | **1.373 mm** |
| **3D** | **9.040 mm** | **2.784 mm** |

![3D pose RMSE comparison](src/resilient_manipulation_core/test/phase10/results/phase10_pose_rmse_comparison.png)

![Pose RMSE by axis](src/resilient_manipulation_core/test/phase10/results/phase10_pose_rmse_by_axis.png)

Detailed estimator notes and equations:

[`docs/pose_estimation.md`](docs/pose_estimation.md)

---

## Controlled fault response

Recovery is intentionally bounded.

| Fault condition | Baseline | Resilient | Evidence |
|---|---|---|---|
| Temporary stale pose | SAFE_ABORT | **RECOVERED** | 1 recovery attempt, ~1516 ms |
| Persistent stale pose | SAFE_ABORT | **SAFE_ABORT** | bounded ~3 s recovery window |
| Single planning failure | NO_RECOVERY | **RECOVERED** | second planning attempt succeeds |
| Persistent planning failure | SAFE_ABORT | **SAFE_ABORT** | maximum 2 total attempts |
| Execution failure | SAFE_ABORT | **SAFE_ABORT** | automatic execution retry disabled |

![Controlled fault response](src/resilient_manipulation_core/test/phase10/results/phase10_fault_response_matrix.png)

The objective is not to force success after every failure. Persistent and execution-level faults are allowed to terminate through controlled safe-abort behavior.

---

## Baseline vs resilient configuration

Phase 10 uses the **same mission implementation with runtime ablation** instead of maintaining two different mission programs.

### Baseline

```text
Pose source       : /object_pose_measurement
Pose filtering    : disabled
Estimate recovery : disabled
Planning retry    : disabled
Execution retry   : disabled
```

### Resilient

```text
Pose source       : /object_pose_estimate
Pose filtering    : enabled
NIS outlier gate  : enabled
Stale protection  : enabled
Estimate recovery : enabled
Planning retry    : maximum 1 retry
Execution retry   : disabled
```

Execution failures are not blindly retried in either mode.

---

## Motion planning

Motion planning uses **MoveIt 2 and existing OMPL planners**.

The project does **not** implement or claim a custom RRT planner.

Implemented planning capabilities include:

- joint-space planning,
- Cartesian end-effector paths,
- collision-object insertion,
- collision-aware planning,
- multi-stage manipulation sequencing.

![Collision-aware planning](docs/evidence/collision_aware_planning_rviz.png)

---

## ROS 2 executables

| Executable | Role |
|---|---|
| `panda_state_monitor` | Monitors Panda joint-state feedback |
| `joint_space_planner` | Joint-space planning baseline |
| `cartesian_path_planner` | Cartesian end-effector path generation |
| `obstacle_avoidance_planner` | Collision-aware planning |
| `pose_uncertainty_injector` | Controlled perception-fault injection |
| `pose_filter` | Kalman filtering, NIS gating and stale-data protection |
| `pose_estimation_validator` | Raw vs estimated pose validation |
| `pose_bias_validator` | Systematic-bias evaluation |
| `pose_outlier_validator` | Controlled outlier validation |
| `pose_visualizer` | RViz diagnostics |
| `manipulation_mission` | Estimate-driven manipulation and bounded recovery |

Important interfaces:

```text
/object_pose_ground_truth
/object_pose_measurement
/object_pose_estimate
/manipulation_task_targets
/resilient_manipulation/pose_markers
/joint_states
```

Planning frame: `world`  
End-effector link: `panda_link8`

---

## Automated validation

### Phase 9 — regression testing

Six controlled scenarios:

```text
P9-S01  Nominal fresh-pose mission
P9-S02  Temporary stale → fresh recovery
P9-S03  Persistent stale → bounded safe abort
P9-S04  Single planning failure → recovery
P9-S05  Persistent planning failure → bounded safe abort
P9-S06  Execution failure → safe abort without retry
```

Final result:

```text
6 / 6 PASS
0 failures
```

Run:

```bash
python3 \
src/resilient_manipulation_core/test/phase9/batch_regression.py
```

### Phase 10 — engineering validation

Eight requirements cover nominal behavior, pose-estimation accuracy, controlled outlier rejection, transient and persistent perception faults, planning recovery and execution-failure handling.

Final result:

```text
8 / 8 requirements PASS
Overall result: PASS
```

Detailed evidence:

[`Phase 10 validation report`](src/resilient_manipulation_core/test/phase10/results/phase10_engineering_validation_report.md)

[`Requirement traceability`](src/resilient_manipulation_core/test/phase10/results/phase10_requirement_traceability.md)

---

## Build

### Requirements

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- `moveit_resources_panda_moveit_config`
- C++17 compiler
- Python 3
- colcon
- matplotlib

Build from the workspace root:

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select resilient_manipulation_core

source install/setup.bash
```

---

## Run

### 1. Start Panda + MoveIt 2 + RViz

```bash
ros2 launch \
moveit_resources_panda_moveit_config \
demo.launch.py
```

### 2. Load the validated execution profile

```bash
ros2 param load \
/moveit_simple_controller_manager \
src/resilient_manipulation_core/config/moveit_execution_profile.yaml
```

Validated parameters:

```text
allowed_execution_duration_scaling = 2.5
allowed_goal_duration_margin       = 1.0
```

### 3. Run the resilient mission

```bash
ros2 launch \
resilient_manipulation_core \
manipulation_mission.launch.py
```

### Baseline ablation

```bash
ros2 launch \
resilient_manipulation_core \
manipulation_mission.launch.py \
object_pose_topic:=/object_pose_measurement \
resilience_enabled:=false
```

---

## Repository structure

```text
.
├── analysis/                          Phase-6 engineering analysis
├── docs/
│   ├── evidence/                     Curated visual evidence
│   └── pose_estimation.md
├── results/
│   ├── phase6_estimation/
│   ├── phase6_outlier/
│   └── phase8_recovery/
└── src/
    └── resilient_manipulation_core/
        ├── config/
        ├── launch/
        ├── src/
        ├── test/
        │   ├── phase9/
        │   └── phase10/
        ├── CMakeLists.txt
        └── package.xml
```

---

## Demo and engineering evidence

| Manipulation mission | Collision-aware planning |
|:---:|:---:|
| ![Mission](docs/evidence/manipulation_mission_execution.png) | ![Collision planning](docs/evidence/collision_aware_planning_rviz.png) |

| Panda / MoveIt baseline | Stale-pose protection |
|:---:|:---:|
| ![Panda baseline](docs/evidence/panda_moveit_rviz_baseline.png) | ![Stale pose](docs/evidence/stale_pose_delay_detection_verified.png) |

▶️ [Watch the manipulation mission demo](docs/evidence/manipulation_mission_demo.mp4)

---

## Scope and limitations

This is a **simulation-based manipulation and validation testbed**.

Current boundaries:

- no physical robot validation,
- no camera or learned perception pipeline,
- object pose is synthetically generated for controlled experiments,
- pose orientation is not estimated by the Kalman filter,
- systematic measurement bias remains a demonstrated limitation,
- no custom motion-planning algorithm,
- RViz task markers are not a physics-based grasp object,
- reported results describe defined controlled validation scenarios rather than hardware reliability statistics.

---

## Engineering focus

The project demonstrates:

- ROS 2 / C++ system integration,
- MoveIt 2 manipulation planning,
- Kalman filtering,
- innovation/NIS-based outlier rejection,
- controlled fault injection,
- stale-data handling,
- bounded recovery,
- automated regression testing,
- baseline-vs-resilient ablation,
- quantitative verification,
- requirement-to-test traceability,
- Git-based configuration management.

---

## Status

**Phases 0–10: complete and verified**

**Phase 11: portfolio packaging and reproducibility in progress**

---

## Author

**Yash Khiste**  
M.Sc. Electrical Engineering and Information Technology  
Otto von Guericke University Magdeburg
