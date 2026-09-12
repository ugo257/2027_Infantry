#!/usr/bin/env python3
"""Small-signal Yaw replay model fitted from the accepted PID traces."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import numpy as np


ROBOT_DEF = Path(__file__).resolve().parents[2] / "application" / "robot_def.h"


def source_float(macro: str) -> float:
    text = ROBOT_DEF.read_text(encoding="utf-8")
    match = re.search(rf"^#define\s+{macro}\s+([^/\s]+)", text, re.MULTILINE)
    if match is None:
        raise ValueError(f"missing {macro} in {ROBOT_DEF}")
    literal = match.group(1).strip().strip("()").rstrip("fFuU")
    return float(literal)


def reference_rate(time_s: float) -> float:
    if 0.5 <= time_s < 1.5:
        return 1.5
    if 2.2 <= time_s < 3.2:
        return -1.5
    return 0.0


def simulate(args: argparse.Namespace) -> dict[str, float]:
    dt = 0.001
    duration = 5.0
    count = int(duration / dt)
    theta = 0.0
    omega = 0.0
    theta_ref = 0.0
    current = 0.0
    errors = np.zeros(count)
    angles = np.zeros(count)

    for index in range(count):
        time_s = index * dt
        rate_ref = reference_rate(time_s)
        theta_ref += rate_ref * dt
        error = theta - theta_ref
        controller_rate_ref = rate_ref if args.track_rate else 0.0
        torque = (
            -args.k_angle * error
            - args.k_rate * (omega - controller_rate_ref)
        )
        requested = float(
            np.clip(
                torque * args.torque_to_current,
                -args.current_limit,
                args.current_limit,
            )
        )
        if args.current_slew > 0.0:
            delta = float(
                np.clip(
                    requested - current,
                    -args.current_slew * dt,
                    args.current_slew * dt,
                )
            )
            current += delta
        else:
            current = requested

        # Median coefficients from the two accepted PID traces.
        alpha = -1.55 * omega - 0.00262 * current
        omega += alpha * dt
        theta += omega * dt
        errors[index] = math.degrees(theta_ref - theta)
        angles[index] = math.degrees(theta)

    active = errors[int(0.5 / dt) :]
    settled = errors[int(3.7 / dt) :]
    settled_angle = angles[int(3.7 / dt) :]
    return {
        "rms_error_deg": float(np.sqrt(np.mean(active * active))),
        "max_abs_error_deg": float(np.max(np.abs(active))),
        "settled_rms_error_deg": float(np.sqrt(np.mean(settled * settled))),
        "settled_angle_span_deg": float(np.ptp(settled_angle)),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--k-angle", type=float)
    parser.add_argument("--k-rate", type=float)
    parser.add_argument("--torque-to-current", type=float)
    parser.add_argument("--current-limit", type=float)
    parser.add_argument("--current-slew", type=float)
    parser.add_argument("--track-rate", action="store_true")
    parser.add_argument("--assert-stable-tracking", action="store_true")
    args = parser.parse_args()

    source_defaults = {
        "k_angle": "GIMBAL_YAW_LQR_K_ANGLE",
        "k_rate": "GIMBAL_YAW_LQR_K_RATE",
        "torque_to_current": "GIMBAL_YAW_LQR_TORQUE_TO_CURRENT",
        "current_limit": "GIMBAL_YAW_LQR_CURRENT_LIMIT",
        "current_slew": "GIMBAL_YAW_LQR_CURRENT_SLEW_RATE",
    }
    for attribute, macro in source_defaults.items():
        if getattr(args, attribute) is None:
            setattr(args, attribute, source_float(macro))

    metrics = simulate(args)
    for name, value in metrics.items():
        print(f"{name}: {value:.6g}")
    passed = (
        metrics["rms_error_deg"] <= 8.0
        and metrics["max_abs_error_deg"] <= 15.0
        and metrics["settled_rms_error_deg"] <= 2.0
        and metrics["settled_angle_span_deg"] <= 4.0
    )
    if args.assert_stable_tracking:
        print("CLOSED_LOOP_CHECK: PASS" if passed else "CLOSED_LOOP_CHECK: FAIL")
        return 0 if passed else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
