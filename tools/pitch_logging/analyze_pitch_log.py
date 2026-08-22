#!/usr/bin/env python3
"""Analyze an Ozone Data Sampling CSV for the single-pitch Shadow test.

The script accepts Ozone's expression headers as exported, including headers
containing ``->`` or parentheses. It prints a compact health report and, when
matplotlib is installed, writes a diagnostic PNG next to the input CSV.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from pathlib import Path


def norm(name: str) -> str:
    return re.sub(r"[^a-z0-9]", "", name.lower())


def resolve(headers: list[str], aliases: list[str], required: bool = False) -> str | None:
    by_norm = {norm(header): header for header in headers}
    for alias in aliases:
        hit = by_norm.get(norm(alias))
        if hit is not None:
            return hit
    if required:
        raise ValueError("missing required column; tried: " + ", ".join(aliases))
    return None


def as_float(value: str) -> float:
    value = value.strip()
    if not value:
        return math.nan
    return float(value)


def read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            raise ValueError("CSV has no header row")
        rows = list(reader)
    if not rows:
        raise ValueError("CSV has no data rows")
    return list(reader.fieldnames), rows


def finite_values(rows: list[dict[str, str]], column: str | None) -> list[float]:
    if column is None:
        return []
    values = [as_float(row.get(column, "")) for row in rows]
    return [value for value in values if math.isfinite(value)]


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (index - lower)


def describe(name: str, values: list[float]) -> None:
    if not values:
        print(f"{name:22s}: unavailable")
        return
    print(f"{name:22s}: min={min(values): .6g}, max={max(values): .6g}, "
          f"p50={percentile(values, 0.50): .6g}, p99={percentile(values, 0.99): .6g}")


def make_plot(path: Path, time: list[float], data: dict[str, list[float]]) -> Path | None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("plot                    : skipped (matplotlib is not installed)")
        return None

    def series(key: str) -> list[float]:
        values = data.get(key, [])
        return values if len(values) == len(time) else [math.nan] * len(time)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axes[0].plot(time, series("theta_ref"), label="theta ref")
    axes[0].plot(time, series("theta_meas"), label="theta measured")
    axes[0].set_ylabel("rad")
    axes[0].legend(loc="upper right")
    axes[0].grid(True)

    axes[1].plot(time, series("omega_meas"), label="gyro pitch")
    axes[1].plot(time, series("error"), label="angle error")
    axes[1].set_ylabel("rad/s or rad")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(time, series("tau_lqr"), label="LQR candidate")
    axes[2].plot(time, series("tau_cmd"), label="candidate after limits")
    axes[2].plot(time, series("tau_applied"), label="previous applied")
    axes[2].plot(time, series("tau_feedback"), label="motor feedback", alpha=0.7)
    axes[2].set_ylabel("Nm")
    axes[2].set_xlabel("time (s)")
    axes[2].legend(loc="upper right")
    axes[2].grid(True)

    fig.suptitle("Pitch Shadow test")
    fig.tight_layout()
    output = path.with_name(path.stem + "_pitch_report.png")
    fig.savefig(output, dpi=160)
    plt.close(fig)
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="Ozone Data Sampling CSV")
    args = parser.parse_args()

    if not args.csv.is_file():
        print(f"error: file not found: {args.csv}", file=sys.stderr)
        return 2

    try:
        headers, rows = read_csv(args.csv)
        columns = {
            "time": resolve(headers, ["Time", "time_s", "time", "timestamp"]),
            "theta_ref": resolve(headers, ["pitch_auto_lqr_theta_ref_debug"] , True),
            "theta_meas": resolve(headers, ["pitch_auto_lqr_theta_meas_debug"], True),
            "omega_meas": resolve(headers, ["pitch_auto_lqr_omega_meas_debug"], True),
            "error": resolve(headers, ["pitch_auto_lqr_err_debug"]),
            "tau_lqr": resolve(headers, ["pitch_auto_lqr_tau_lqr_debug"], True),
            "tau_cmd": resolve(headers, ["pitch_auto_lqr_tau_cmd_debug"], True),
            "tau_applied": resolve(headers, ["pitch_auto_lqr_tau_applied_debug", "pitch_motor->ctrl.tor_set"], True),
            "tau_feedback": resolve(headers, ["pitch_motor->measure.tor", "pitch_motor.measure.tor"]),
            "stage": resolve(headers, ["g_pitch_lqr_stage"]),
            "dt": resolve(headers, ["pitch_auto_lqr_dt_debug"]),
            "valid": resolve(headers, ["pitch_auto_lqr_output_valid_debug"]),
            "timing_fault": resolve(headers, ["pitch_auto_lqr_timing_fault_debug"]),
            "feedback_fault": resolve(headers, ["pitch_auto_lqr_feedback_fault_debug"]),
            "fallback": resolve(headers, ["pitch_auto_lqr_fallback_debug"]),
            "z1": resolve(headers, ["pitch_auto_lqr_eso_z1_debug"]),
            "z2": resolve(headers, ["pitch_auto_lqr_eso_z2_debug"]),
            "z3": resolve(headers, ["pitch_auto_lqr_eso_z3_debug"]),
        }
    except (ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        print("available columns:", file=sys.stderr)
        print("  " + "\n  ".join(headers), file=sys.stderr)
        return 2

    time = finite_values(rows, columns["time"])
    if len(time) != len(rows):
        time = [float(index) for index in range(len(rows))]
        print("warning: no usable time column; using row index")
    data = {key: finite_values(rows, column) for key, column in columns.items()}

    print(f"file                    : {args.csv}")
    print(f"rows                    : {len(rows)}")
    print(f"duration_s              : {time[-1] - time[0]:.6g}")
    if columns["dt"]:
        describe("dt_s", data["dt"])
    if columns["stage"]:
        stages = sorted(set(int(value) for value in data["stage"] if math.isfinite(value)))
        print(f"stages_seen             : {stages}")
    for key, label in (("valid", "invalid_output"), ("timing_fault", "timing_fault"),
                       ("feedback_fault", "feedback_fault"), ("fallback", "fallback")):
        values = data.get(key, [])
        if values:
            print(f"{label:22s}: count={sum(1 for value in values if value != 0.0)}")

    describe("angle_error_rad", data["error"])
    describe("theta_ref_rad", data["theta_ref"])
    describe("theta_meas_rad", data["theta_meas"])
    describe("omega_meas_rad_s", data["omega_meas"])
    describe("tau_lqr_nm", data["tau_lqr"])
    describe("tau_cmd_nm", data["tau_cmd"])
    describe("tau_applied_nm", data["tau_applied"])
    describe("eso_z3", data.get("z3", []))

    plot = make_plot(args.csv, time, data)
    if plot:
        print(f"plot                   : {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
