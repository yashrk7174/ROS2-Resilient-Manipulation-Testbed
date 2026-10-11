#!/usr/bin/env python3

import json
from pathlib import Path


SCENARIO_DESCRIPTIONS = {
    "P9-S01": "Fresh nominal mission",
    "P9-S02": "Stale estimate -> fresh estimate -> recovery",
    "P9-S03": "Persistent stale estimate -> bounded SAFE_ABORT",
    "P9-S04": "Single planning failure -> bounded retry -> RECOVERED",
    "P9-S05": "Persistent planning failure -> retry exhausted -> SAFE_ABORT",
    "P9-S06": "Execution failure -> no blind retry -> SAFE_ABORT",
}


def main():
    script_directory = (
        Path(__file__).resolve().parent
    )

    result_directory = (
        script_directory / "results"
    )

    json_path = (
        result_directory
        / "phase9_regression_summary.json"
    )

    report_path = (
        result_directory
        / "phase9_final_regression_report.md"
    )


    if not json_path.exists():
        raise FileNotFoundError(
            f"Regression summary not found: {json_path}"
        )


    data = json.loads(
        json_path.read_text(
            encoding="utf-8"
        )
    )


    lines = []

    lines.append(
        "# Phase 9 — Automated Scenario Testing"
    )

    lines.append("")
    lines.append(
        "## Final Regression Report"
    )

    lines.append("")
    lines.append(
        f"- Total scenarios: {data['total_scenarios']}"
    )

    lines.append(
        f"- Passed: {data['passed']}"
    )

    lines.append(
        f"- Failed: {data['failed']}"
    )

    lines.append(
        f"- Overall result: **{data['overall_status']}**"
    )

    lines.append(
        f"- Generated: {data['generated_at']}"
    )


    lines.append("")
    lines.append(
        "## Scenario Results"
    )

    lines.append("")
    lines.append(
        "| Scenario | Requirement | Result | Duration (s) |"
    )

    lines.append(
        "|---|---|---:|---:|"
    )


    for scenario in data["scenarios"]:
        scenario_id = scenario["scenario_id"]

        description = SCENARIO_DESCRIPTIONS.get(
            scenario_id,
            "Unknown scenario",
        )

        lines.append(
            f"| {scenario_id} | "
            f"{description} | "
            f"{scenario['status']} | "
            f"{scenario['duration_seconds']:.3f} |"
        )


    lines.append("")
    lines.append(
        "## Verified Behaviors"
    )

    lines.append("")

    lines.append(
        "- Nominal estimate-driven manipulation completes successfully."
    )

    lines.append(
        "- A stale estimate can recover when a new valid estimate arrives within the bounded recovery window."
    )

    lines.append(
        "- Persistent stale perception causes SAFE_ABORT before manipulation planning proceeds."
    )

    lines.append(
        "- A single injected planning failure triggers one bounded re-plan attempt and can recover."
    )

    lines.append(
        "- Persistent planning failure exhausts the single retry budget and causes SAFE_ABORT."
    )

    lines.append(
        "- Execution failure does not trigger blind automatic execution retry; the mission safely aborts."
    )


    lines.append("")
    lines.append(
        "## Test Architecture"
    )

    lines.append("")

    lines.append(
        "The Phase-9 Python runner launches controlled scenario helpers and the manipulation mission, captures process output, checks expected and forbidden evidence markers, distinguishes the ros2 launch wrapper exit code from the actual manipulation_mission child-process exit code, and reports scenario-level PASS/FAIL."
    )


    lines.append("")
    lines.append(
        "## Engineering Note"
    )

    lines.append("")

    lines.append(
        "During automation, an execution-monitor timing mismatch was identified: MoveIt could stop waiting while panda_arm_controller was still RUNNING and later reported successful goal completion. A project-owned MoveIt execution profile was therefore validated with allowed_execution_duration_scaling = 2.5 and allowed_goal_duration_margin = 1.0."
    )


    lines.append("")
    lines.append(
        "## Final Result"
    )

    lines.append("")

    lines.append(
        f"**Phase 9 automated regression: "
        f"{data['passed']}/{data['total_scenarios']} "
        f"scenarios PASS — {data['overall_status']}.**"
    )


    report_path.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


    print(
        f"Generated report: {report_path}"
    )


if __name__ == "__main__":
    main()
