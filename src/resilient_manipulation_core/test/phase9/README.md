# Phase 9 — Automated Scenario Testing

## Objective

Phase 9 converts the manually verified resilience behaviours from Phase 8
into repeatable automated regression scenarios.

The test framework validates nominal manipulation, perception-staleness
handling, bounded planning recovery and safe execution-failure handling.

---

## Automated Scenarios

| Scenario | Verification Objective | Expected Behaviour |
|---|---|---|
| P9-S01 | Fresh nominal estimate | Mission completes successfully |
| P9-S02 | Stale estimate followed by fresh estimate | Recovery triggered, fresh estimate reacquired, mission continues |
| P9-S03 | Persistent stale estimate | Recovery timeout followed by SAFE_ABORT before manipulation planning |
| P9-S04 | Single planning failure | One bounded re-plan attempt followed by RECOVERED |
| P9-S05 | Persistent planning failure | Retry budget exhausted followed by SAFE_ABORT |
| P9-S06 | Execution failure | SAFE_ABORT with automatic execution retry disabled |

---

## Test Architecture

The Python scenario runner:

1. launches the manipulation mission,
2. synchronizes controlled estimate publication with mission readiness,
3. applies scenario-specific fault-injection hooks,
4. captures mission and helper process output,
5. distinguishes the ros2 launch wrapper exit code from the actual
   manipulation_mission child-process exit code,
6. checks required evidence markers,
7. checks forbidden markers where applicable,
8. evaluates helper-process status,
9. reports scenario-level PASS / FAIL.

The complete batch runner executes P9-S01 through P9-S06 sequentially and
generates machine-readable CSV and JSON regression summaries.

---

## Deterministic Estimate Injection

Controlled estimate scenarios use:

`controlled_estimate_publisher.py`

For stale-estimate tests, the helper starts only after the manipulation
mission reports that it is waiting for `/object_pose_estimate`.

P9-S02 continuously publishes the same stale timestamp before publishing a
new fresh estimate. This ensures that the stale condition is actually
observed before recovery is evaluated.

---

## MoveIt Execution Profile

Automated regression uses the project-owned MoveIt execution profile.

Runtime node:

`/moveit_simple_controller_manager`

Validated parameters:

- `trajectory_execution.allowed_execution_duration_scaling = 2.5`
- `trajectory_execution.allowed_goal_duration_margin = 1.0`

During Phase 9 debugging, loading these parameters on `/move_group` was found
not to update the active trajectory-execution timing configuration.

The active MoveIt trajectory execution manager therefore retained the
default execution limit and prematurely cancelled valid controller goals.

Applying the profile to `/moveit_simple_controller_manager` resolved the
issue.

---

## Running One Scenario

Example:

```bash
python3 \
src/resilient_manipulation_core/test/phase9/scenario_runner.py \
--scenario P9-S02
