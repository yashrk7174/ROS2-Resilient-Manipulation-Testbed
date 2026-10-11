# Phase 9 — Automated Scenario Testing

## Final Regression Report

- Total scenarios: 6
- Passed: 6
- Failed: 0
- Overall result: **PASS**
- Generated: 2026-10-11T16:57:51

## Scenario Results

| Scenario | Requirement | Result | Duration (s) |
|---|---|---:|---:|
| P9-S01 | Fresh nominal mission | PASS | 17.691 |
| P9-S02 | Stale estimate -> fresh estimate -> recovery | PASS | 14.839 |
| P9-S03 | Persistent stale estimate -> bounded SAFE_ABORT | PASS | 9.701 |
| P9-S04 | Single planning failure -> bounded retry -> RECOVERED | PASS | 12.151 |
| P9-S05 | Persistent planning failure -> retry exhausted -> SAFE_ABORT | PASS | 10.046 |
| P9-S06 | Execution failure -> no blind retry -> SAFE_ABORT | PASS | 8.125 |

## Verified Behaviors

- Nominal estimate-driven manipulation completes successfully.
- A stale estimate can recover when a new valid estimate arrives within the bounded recovery window.
- Persistent stale perception causes SAFE_ABORT before manipulation planning proceeds.
- A single injected planning failure triggers one bounded re-plan attempt and can recover.
- Persistent planning failure exhausts the single retry budget and causes SAFE_ABORT.
- Execution failure does not trigger blind automatic execution retry; the mission safely aborts.

## Test Architecture

The Phase-9 Python runner launches controlled scenario helpers and the manipulation mission, captures process output, checks expected and forbidden evidence markers, distinguishes the ros2 launch wrapper exit code from the actual manipulation_mission child-process exit code, and reports scenario-level PASS/FAIL.

## Engineering Note

During automation, an execution-monitor timing mismatch was identified: MoveIt could stop waiting while panda_arm_controller was still RUNNING and later reported successful goal completion. A project-owned MoveIt execution profile was therefore validated with allowed_execution_duration_scaling = 2.5 and allowed_goal_duration_margin = 1.0.

## Final Result

**Phase 9 automated regression: 6/6 scenarios PASS — PASS.**
