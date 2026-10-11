#!/usr/bin/env python3

import argparse
import csv
import json
import subprocess
import sys
import time

from datetime import datetime
from pathlib import Path


SCENARIOS = [
    "P9-S01",
    "P9-S02",
    "P9-S03",
    "P9-S04",
    "P9-S05",
    "P9-S06",
]


def run_scenario(
    scenario_id: str,
    runner_path: Path,
):
    print()
    print("=" * 78)
    print(
        f"PHASE 9 BATCH — running {scenario_id}"
    )
    print("=" * 78)

    command = [
        sys.executable,
        str(runner_path),
        "--scenario",
        scenario_id,
    ]

    start_time = time.monotonic()

    process = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    duration_seconds = (
        time.monotonic()
        - start_time
    )

    output_text = process.stdout or ""

    print(
        output_text,
        end="",
    )

    passed = (
        process.returncode == 0
        and "AUTOMATED RESULT  : PASS"
        in output_text
    )

    return {
        "scenario_id": scenario_id,
        "status": (
            "PASS"
            if passed
            else "FAIL"
        ),
        "runner_exit_code": process.returncode,
        "duration_seconds": round(
            duration_seconds,
            3,
        ),
        "output": output_text,
    }


def write_csv(
    results,
    output_path: Path,
):
    with output_path.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as csv_file:

        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "scenario_id",
                "status",
                "runner_exit_code",
                "duration_seconds",
            ],
        )

        writer.writeheader()

        for result in results:
            writer.writerow(
                {
                    "scenario_id":
                        result["scenario_id"],

                    "status":
                        result["status"],

                    "runner_exit_code":
                        result["runner_exit_code"],

                    "duration_seconds":
                        result["duration_seconds"],
                }
            )


def write_json(
    results,
    output_path: Path,
):
    passed_count = sum(
        1
        for result in results
        if result["status"] == "PASS"
    )

    failed_count = (
        len(results)
        - passed_count
    )

    document = {
        "phase": "Phase 9",
        "test_suite":
            "Automated Scenario Regression",

        "generated_at":
            datetime.now().isoformat(
                timespec="seconds"
            ),

        "total_scenarios":
            len(results),

        "passed":
            passed_count,

        "failed":
            failed_count,

        "overall_status":
            (
                "PASS"
                if failed_count == 0
                else "FAIL"
            ),

        "scenarios": [
            {
                "scenario_id":
                    result["scenario_id"],

                "status":
                    result["status"],

                "runner_exit_code":
                    result[
                        "runner_exit_code"
                    ],

                "duration_seconds":
                    result[
                        "duration_seconds"
                    ],
            }
            for result in results
        ],
    }

    output_path.write_text(
        json.dumps(
            document,
            indent=2,
        ),
        encoding="utf-8",
    )


def print_summary(
    results,
):
    passed_count = sum(
        1
        for result in results
        if result["status"] == "PASS"
    )

    failed_count = (
        len(results)
        - passed_count
    )

    print()
    print("=" * 78)
    print("PHASE 9 AUTOMATED REGRESSION SUMMARY")
    print("=" * 78)

    for result in results:
        print(
            f"{result['scenario_id']}: "
            f"{result['status']} "
            f"({result['duration_seconds']:.3f} s)"
        )

    print("-" * 78)

    print(
        f"Total scenarios : {len(results)}"
    )

    print(
        f"Passed          : {passed_count}"
    )

    print(
        f"Failed          : {failed_count}"
    )

    print(
        "Overall result  : "
        + (
            "PASS"
            if failed_count == 0
            else "FAIL"
        )
    )

    print("=" * 78)

    return failed_count == 0


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Phase-9 complete automated "
            "regression runner."
        )
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help=(
            "Show batch configuration "
            "without running scenarios."
        ),
    )

    return parser.parse_args()


def main():
    arguments = parse_arguments()

    script_directory = (
        Path(__file__)
        .resolve()
        .parent
    )

    runner_path = (
        script_directory
        / "scenario_runner.py"
    )

    result_directory = (
        script_directory
        / "results"
    )


    if arguments.dry_run:
        print(
            "Phase-9 batch regression scenarios:"
        )

        for scenario_id in SCENARIOS:
            print(
                f"  - {scenario_id}"
            )

        print(
            "\nDRY RUN ONLY — "
            "no scenario was executed."
        )

        return 0


    result_directory.mkdir(
        parents=True,
        exist_ok=True,
    )


    results = []


    for scenario_id in SCENARIOS:
        result = run_scenario(
            scenario_id,
            runner_path,
        )

        results.append(
            result
        )


    csv_path = (
        result_directory
        / "phase9_regression_summary.csv"
    )

    json_path = (
        result_directory
        / "phase9_regression_summary.json"
    )


    write_csv(
        results,
        csv_path,
    )

    write_json(
        results,
        json_path,
    )


    overall_pass = print_summary(
        results
    )


    print()
    print(
        f"CSV evidence : {csv_path}"
    )

    print(
        f"JSON evidence: {json_path}"
    )


    return (
        0
        if overall_pass
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
