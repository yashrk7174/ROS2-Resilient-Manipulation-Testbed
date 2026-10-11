#!/usr/bin/env python3

import csv
import json
from pathlib import Path


PHASE10_DIR = Path(__file__).resolve().parent
RESULTS_DIR = PHASE10_DIR / "results"

MATRIX_PATH = PHASE10_DIR / "validation_matrix.csv"
SUMMARY_PATH = RESULTS_DIR / "phase10_validation_summary.csv"

JSON_PATH = RESULTS_DIR / "phase10_validation_summary.json"
REPORT_PATH = RESULTS_DIR / "phase10_engineering_validation_report.md"


def load_csv(path):
    with path.open("r", encoding="utf-8", newline="") as file:
        return list(csv.DictReader(file))


def index_by_test_id(rows):
    return {
        row["test_id"]: row
        for row in rows
    }


def require_test_ids(summary_rows):
    required_ids = {
        f"P10-V{number:02d}"
        for number in range(1, 9)
    }

    actual_ids = {
        row["test_id"]
        for row in summary_rows
    }

    missing = required_ids - actual_ids
    extra = actual_ids - required_ids

    if missing:
        raise RuntimeError(
            f"Missing Phase-10 tests: {sorted(missing)}"
        )

    if extra:
        raise RuntimeError(
            f"Unexpected Phase-10 tests: {sorted(extra)}"
        )


def verify_matrix_traceability(summary_rows):
    if not MATRIX_PATH.exists():
        print(
            "WARNING: validation_matrix.csv not found; "
            "matrix traceability check skipped."
        )
        return

    matrix_rows = load_csv(MATRIX_PATH)

    matrix_ids = {
        row["test_id"]
        for row in matrix_rows
    }

    summary_ids = {
        row["test_id"]
        for row in summary_rows
    }

    if matrix_ids != summary_ids:
        raise RuntimeError(
            "Validation matrix and result summary "
            "contain different test IDs."
        )


def calculate_metrics(rows_by_id):

    v02 = rows_by_id["P10-V02"]

    baseline_rmse = float(
        v02["baseline_metric"]
    )

    resilient_rmse = float(
        v02["resilient_metric"]
    )

    rmse_reduction_percent = (
        (baseline_rmse - resilient_rmse)
        / baseline_rmse
        * 100.0
    )

    transient_test_ids = [
        "P10-V04",
        "P10-V06",
    ]

    baseline_transient_recoveries = sum(
        int(rows_by_id[test_id]["baseline_metric"])
        for test_id in transient_test_ids
    )

    resilient_transient_recoveries = sum(
        int(rows_by_id[test_id]["resilient_metric"])
        for test_id in transient_test_ids
    )

    return {
        "baseline_rmse_3d_mm": baseline_rmse,
        "resilient_rmse_3d_mm": resilient_rmse,
        "rmse_reduction_percent": (
            rmse_reduction_percent
        ),
        "transient_recovery_tests": len(
            transient_test_ids
        ),
        "baseline_transient_recoveries": (
            baseline_transient_recoveries
        ),
        "resilient_transient_recoveries": (
            resilient_transient_recoveries
        ),
    }


def write_json(summary_rows, metrics):

    passed = sum(
        row["result"] == "PASS"
        for row in summary_rows
    )

    output = {
        "phase": "Phase 10 — Engineering Validation",
        "comparison": "Baseline vs Resilient",
        "tests_total": len(summary_rows),
        "tests_passed": passed,
        "tests_failed": len(summary_rows) - passed,
        "overall_result": (
            "PASS"
            if passed == len(summary_rows)
            else "FAIL"
        ),
        "metrics": metrics,
        "tests": summary_rows,
    }

    with JSON_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            output,
            file,
            indent=2,
        )

    return output


def write_markdown(summary_rows, metrics, output):

    lines = []

    lines.append(
        "# Phase 10 — Engineering Validation"
    )
    lines.append("")
    lines.append(
        "## Baseline vs Resilient System"
    )
    lines.append("")

    lines.append(
        f"**Overall result: "
        f"{output['overall_result']}**"
    )
    lines.append("")

    lines.append(
        f"Tests passed: "
        f"{output['tests_passed']}/"
        f"{output['tests_total']}"
    )
    lines.append("")

    lines.append(
        "## Key Quantitative Results"
    )
    lines.append("")

    lines.append(
        f"- Raw-pose 3D RMSE: "
        f"{metrics['baseline_rmse_3d_mm']:.3f} mm"
    )

    lines.append(
        f"- Kalman-filtered 3D RMSE: "
        f"{metrics['resilient_rmse_3d_mm']:.3f} mm"
    )

    lines.append(
        f"- 3D RMSE reduction: "
        f"{metrics['rmse_reduction_percent']:.2f}%"
    )

    lines.append(
        "- Controlled outliers protected: "
        "20/20 in the defined validation scenario"
    )

    lines.append(
        f"- Transient recovery scenarios completed: "
        f"baseline "
        f"{metrics['baseline_transient_recoveries']}/"
        f"{metrics['transient_recovery_tests']}, "
        f"resilient "
        f"{metrics['resilient_transient_recoveries']}/"
        f"{metrics['transient_recovery_tests']}"
    )

    lines.append(
        "- Execution failure automatic retry: "
        "disabled"
    )

    lines.append("")

    lines.append(
        "## Validation Matrix"
    )
    lines.append("")

    lines.append(
        "| Test | Scenario | Baseline | "
        "Resilient | Result |"
    )

    lines.append(
        "|---|---|---|---|---|"
    )

    for row in summary_rows:
        lines.append(
            f"| {row['test_id']} "
            f"| {row['scenario']} "
            f"| {row['baseline_outcome']} "
            f"| {row['resilient_outcome']} "
            f"| {row['result']} |"
        )

    lines.append("")
    lines.append(
        "## Engineering Interpretation"
    )
    lines.append("")

    lines.append(
        "The resilient configuration preserved nominal "
        "mission capability while improving pose accuracy "
        "and recovering from controlled transient stale-pose "
        "and planning failures."
    )

    lines.append("")

    lines.append(
        "Persistent perception and planning faults remained "
        "bounded and terminated through safe-abort behavior."
    )

    lines.append("")

    lines.append(
        "Execution failures were intentionally not retried "
        "automatically, preventing blind repetition of a "
        "trajectory after an execution-level failure."
    )

    lines.append("")

    lines.append(
        "Outlier performance refers specifically to the "
        "20 controlled injected outliers used in the "
        "defined validation scenario; it is not a universal "
        "100% outlier-detection claim."
    )

    lines.append("")

    lines.append(
        "## Evidence Provenance"
    )
    lines.append("")

    for row in summary_rows:
        lines.append(
            f"- **{row['test_id']}** — "
            f"{row['evidence_origin']}. "
            f"{row['notes']}."
        )

    REPORT_PATH.write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def main():

    if not SUMMARY_PATH.exists():
        raise FileNotFoundError(
            f"Missing result summary: {SUMMARY_PATH}"
        )

    summary_rows = load_csv(
        SUMMARY_PATH
    )

    require_test_ids(
        summary_rows
    )

    verify_matrix_traceability(
        summary_rows
    )

    failed_tests = [
        row["test_id"]
        for row in summary_rows
        if row["result"] != "PASS"
    ]

    if failed_tests:
        raise RuntimeError(
            f"Phase-10 validation contains failures: "
            f"{failed_tests}"
        )

    rows_by_id = index_by_test_id(
        summary_rows
    )

    metrics = calculate_metrics(
        rows_by_id
    )

    output = write_json(
        summary_rows,
        metrics,
    )

    write_markdown(
        summary_rows,
        metrics,
        output,
    )

    print(
        "============================================================"
    )

    print(
        " PHASE 10 ENGINEERING VALIDATION"
    )

    print(
        "============================================================"
    )

    print(
        f"Tests                : "
        f"{output['tests_passed']}/"
        f"{output['tests_total']} PASS"
    )

    print(
        f"Raw 3D RMSE          : "
        f"{metrics['baseline_rmse_3d_mm']:.3f} mm"
    )

    print(
        f"Resilient 3D RMSE    : "
        f"{metrics['resilient_rmse_3d_mm']:.3f} mm"
    )

    print(
        f"RMSE reduction       : "
        f"{metrics['rmse_reduction_percent']:.2f}%"
    )

    print(
        f"Transient recoveries : "
        f"baseline "
        f"{metrics['baseline_transient_recoveries']}/"
        f"{metrics['transient_recovery_tests']} | "
        f"resilient "
        f"{metrics['resilient_transient_recoveries']}/"
        f"{metrics['transient_recovery_tests']}"
    )

    print(
        f"Overall result        : "
        f"{output['overall_result']}"
    )

    print(
        "============================================================"
    )

    print(
        f"JSON   : {JSON_PATH}"
    )

    print(
        f"Report : {REPORT_PATH}"
    )


if __name__ == "__main__":
    main()
