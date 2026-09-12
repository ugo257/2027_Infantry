#!/usr/bin/env python3
"""Check Yaw tracking and estimate actuator direction from an Ozone CSV."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.signal import savgol_filter


TIME = "Time"
ANGLE_REF = "yaw_lqr_angle_ref_debug"
ANGLE_MEASURE = "yaw_lqr_angle_measure_debug"
CURRENT_COMMAND = "yaw_lqr_current_command_debug"
CURRENT_MEASURE = "yaw_lqr_motor_current_debug"
ACTIVE = "yaw_lqr_active_debug"
VALID = "yaw_lqr_output_valid_debug"
LIMIT = "yaw_lqr_limit_debug"
RATE_MEASURE = "yaw_lqr_rate_measure_debug"
CURRENT_PRE_LIMIT = "yaw_lqr_current_pre_limit_debug"


def wrapped_error_deg(reference: np.ndarray, measure: np.ndarray) -> np.ndarray:
    return (reference - measure + 180.0) % 360.0 - 180.0


def load_valid_rows(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    required = [TIME, ANGLE_REF, ANGLE_MEASURE, CURRENT_MEASURE]
    missing = [name for name in required if name not in frame.columns]
    if missing:
        raise ValueError(f"missing columns: {', '.join(missing)}")

    numeric = required + [
        name
        for name in (
            CURRENT_COMMAND,
            ACTIVE,
            VALID,
            LIMIT,
            RATE_MEASURE,
            CURRENT_PRE_LIMIT,
        )
        if name in frame.columns
    ]
    for name in numeric:
        frame[name] = pd.to_numeric(frame[name], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan).dropna(subset=required)
    frame = frame.sort_values(TIME, kind="stable")
    frame = frame.loc[frame[TIME].diff().fillna(1.0) > 0.0].copy()
    return frame


def active_rows(frame: pd.DataFrame) -> pd.DataFrame:
    selected = frame
    if ACTIVE in selected and (selected[ACTIVE] == 1.0).any():
        selected = selected.loc[selected[ACTIVE] == 1.0]
    if VALID in selected and (selected[VALID] == 1.0).any():
        selected = selected.loc[selected[VALID] == 1.0]
    return selected


def tracking_metrics(frame: pd.DataFrame) -> dict[str, float]:
    selected = active_rows(frame)
    error = wrapped_error_deg(
        selected[ANGLE_REF].to_numpy(), selected[ANGLE_MEASURE].to_numpy()
    )
    metrics = {
        "samples": float(len(selected)),
        "rms_error_deg": float(np.sqrt(np.mean(error * error))),
        "p95_abs_error_deg": float(np.percentile(np.abs(error), 95.0)),
        "max_abs_error_deg": float(np.max(np.abs(error))),
        "measure_span_deg": float(
            selected[ANGLE_MEASURE].max() - selected[ANGLE_MEASURE].min()
        ),
    }
    if LIMIT in selected:
        metrics["limit_fraction"] = float(np.mean(selected[LIMIT] != 0.0))
    return metrics


def odd_window(sample_period_s: float, duration_s: float, count: int) -> int:
    window = max(7, int(round(duration_s / sample_period_s)))
    if window % 2 == 0:
        window += 1
    largest = count if count % 2 == 1 else count - 1
    return min(window, largest)


def estimate_current_accel_gain(frame: pd.DataFrame) -> tuple[float, float, float, int]:
    selected = frame.loc[
        frame[[TIME, ANGLE_MEASURE, CURRENT_MEASURE]].notna().all(axis=1)
    ]
    if len(selected) < 100:
        raise ValueError("not enough valid samples for actuator-direction estimate")

    time = selected[TIME].to_numpy(dtype=float)
    angle = np.unwrap(np.deg2rad(selected[ANGLE_MEASURE].to_numpy(dtype=float)))
    current = selected[CURRENT_MEASURE].to_numpy(dtype=float)
    dt = float(np.median(np.diff(time)))
    if not 0.0001 <= dt <= 0.05:
        raise ValueError(f"invalid median sample period: {dt:g} s")

    grid = np.arange(time[0], time[-1], dt)
    angle_grid = np.interp(grid, time, angle)
    current_grid = np.interp(grid, time, current)
    window = odd_window(dt, 0.12, len(grid))
    angle_smooth = savgol_filter(angle_grid, window, 3)
    omega = savgol_filter(angle_smooth, window, 3, deriv=1, delta=dt)
    alpha = savgol_filter(angle_smooth, window, 3, deriv=2, delta=dt)
    current_smooth = savgol_filter(current_grid, max(5, window // 3 | 1), 2)

    best_gain = 0.0
    best_damping = 0.0
    best_score = -np.inf
    best_lag = 0
    for lag_s in np.arange(0.0, 0.031, 0.002):
        lag = int(round(lag_s / dt))
        if lag == 0:
            y = alpha
            u = current_smooth
            w = omega
        else:
            y = alpha[lag:]
            u = current_smooth[:-lag]
            w = omega[lag:]
        design = np.column_stack((np.ones_like(u), w, np.sign(w), u))
        coefficient, *_ = np.linalg.lstsq(design, y, rcond=None)
        prediction = design @ coefficient
        residual = np.sum((y - prediction) ** 2)
        total = np.sum((y - np.mean(y)) ** 2)
        score = 1.0 - residual / total if total > 0.0 else -np.inf
        if score > best_score:
            best_gain = float(coefficient[-1])
            best_damping = float(coefficient[1])
            best_score = float(score)
            best_lag = int(round(lag_s * 1000.0))
    return best_gain, best_damping, best_score, best_lag


def dynamic_consistency_metrics(frame: pd.DataFrame) -> dict[str, float]:
    selected = active_rows(frame)
    time = selected[TIME].to_numpy(dtype=float)
    angle = np.unwrap(np.deg2rad(selected[ANGLE_MEASURE].to_numpy(dtype=float)))
    dt = float(np.median(np.diff(time)))
    grid = np.arange(time[0], time[-1], dt)
    angle_grid = np.interp(grid, time, angle)
    window = odd_window(dt, 0.12, len(grid))
    angle_rate = savgol_filter(angle_grid, window, 3, deriv=1, delta=dt)
    metrics: dict[str, float] = {}

    if RATE_MEASURE in selected:
        rate = np.interp(grid, time, selected[RATE_MEASURE].to_numpy(dtype=float))
        metrics["angle_rate_to_recorded_rate_corr"] = float(
            np.corrcoef(angle_rate, rate)[0, 1]
        )

    if CURRENT_COMMAND in selected and CURRENT_PRE_LIMIT in selected:
        command = np.interp(
            grid, time, selected[CURRENT_COMMAND].to_numpy(dtype=float)
        )
        pre_limit = np.interp(
            grid, time, selected[CURRENT_PRE_LIMIT].to_numpy(dtype=float)
        )
        current_limit = max(1.0, float(np.max(np.abs(command))))
        desired = np.clip(pre_limit, -current_limit, current_limit)
        meaningful = np.abs(desired) >= 100.0
        if np.any(meaningful):
            metrics["current_sign_mismatch_fraction"] = float(
                np.mean(command[meaningful] * desired[meaningful] < 0.0)
            )
        metrics["current_command_error_rms"] = float(
            np.sqrt(np.mean((command - desired) ** 2))
        )
        metrics["current_slew_p95_count_s"] = float(
            np.percentile(np.abs(np.diff(command) / dt), 95.0)
        )
    return metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--assert-tracking", action="store_true")
    parser.add_argument("--max-rms-deg", type=float, default=5.0)
    parser.add_argument("--max-p95-deg", type=float, default=10.0)
    parser.add_argument("--estimate-actuator-direction", action="store_true")
    parser.add_argument("--check-dynamics", action="store_true")
    args = parser.parse_args()

    frame = load_valid_rows(args.csv)
    metrics = tracking_metrics(frame)
    print(f"valid tracking samples: {int(metrics['samples'])}")
    for name in (
        "rms_error_deg",
        "p95_abs_error_deg",
        "max_abs_error_deg",
        "measure_span_deg",
        "limit_fraction",
    ):
        if name in metrics:
            print(f"{name}: {metrics[name]:.6g}")

    if args.estimate_actuator_direction:
        gain, damping, score, lag_ms = estimate_current_accel_gain(frame)
        print(f"current_to_yaw_accel_gain: {gain:.9g} rad/s^2/count")
        print(f"yaw_rate_to_accel_gain: {damping:.9g} 1/s")
        print(f"actuator_fit_r2: {score:.6g}")
        print(f"actuator_fit_lag_ms: {lag_ms}")

    if args.check_dynamics:
        for name, value in dynamic_consistency_metrics(frame).items():
            print(f"{name}: {value:.9g}")

    if args.assert_tracking:
        tracking_ok = (
            metrics["rms_error_deg"] <= args.max_rms_deg
            and metrics["p95_abs_error_deg"] <= args.max_p95_deg
        )
        print("TRACKING_CHECK: PASS" if tracking_ok else "TRACKING_CHECK: FAIL")
        return 0 if tracking_ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
