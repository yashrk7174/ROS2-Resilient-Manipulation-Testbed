import csv
from pathlib import Path

import matplotlib.pyplot as plt


PROJECT_ROOT = Path(__file__).resolve().parents[1]

CSV_FILE = (
    PROJECT_ROOT
    / "results"
    / "phase6_outlier"
    / "phase6_outlier_validation_samples.csv"
)

OUTPUT_FILE = (
    PROJECT_ROOT
    / "results"
    / "phase6_outlier"
    / "phase6_outlier_rejection_timeseries.png"
)


samples = []
raw_error_mm = []
estimate_error_mm = []

outlier_samples = []
outlier_errors_mm = []


with CSV_FILE.open("r", newline="") as csv_file:
    reader = csv.DictReader(csv_file)

    for row in reader:

        sample = int(row["sample_index"])

        raw_error = (
            float(row["raw_position_error_m"])
            * 1000.0
        )

        estimate_error = (
            float(row["estimate_position_error_m"])
            * 1000.0
        )

        raw_outlier = (
            int(row["raw_outlier_detected"]) == 1
        )

        samples.append(sample)
        raw_error_mm.append(raw_error)
        estimate_error_mm.append(estimate_error)

        if raw_outlier:
            outlier_samples.append(sample)
            outlier_errors_mm.append(raw_error)


if not samples:
    raise RuntimeError("No samples found in CSV.")


# ============================================================
# Engineering validation plot
# ============================================================

plt.figure(figsize=(12, 6))

plt.plot(
    samples,
    raw_error_mm,
    label="Raw Measurement Error",
    linewidth=1.0
)

plt.plot(
    samples,
    estimate_error_mm,
    label="Gated Estimate Error",
    linewidth=1.5
)

plt.scatter(
    outlier_samples,
    outlier_errors_mm,
    label="Detected Outliers",
    s=25
)

plt.axhline(
    y=5.0,
    linestyle="--",
    linewidth=1.0,
    label="Estimate Error Limit = 5 mm"
)

plt.xlabel("Evaluation Sample")

plt.ylabel("3D Position Error (mm)")

plt.title(
    "Phase 6.5 — Innovation-Gated Outlier Rejection"
)

plt.grid(
    True,
    alpha=0.3
)

plt.legend()

plt.tight_layout()

plt.savefig(
    OUTPUT_FILE,
    dpi=200
)

plt.close()


print("")
print("============================================================")
print(" PHASE 6.5 OUTLIER VISUALIZATION")
print("============================================================")

print(
    f"Evaluation samples     : {len(samples)}"
)

print(
    f"Detected outliers      : {len(outlier_samples)}"
)

print(
    f"Maximum RAW error      : "
    f"{max(raw_error_mm):.3f} mm"
)

print(
    f"Maximum EST error      : "
    f"{max(estimate_error_mm):.3f} mm"
)

print(
    f"Output                 : {OUTPUT_FILE}"
)

print("============================================================")
