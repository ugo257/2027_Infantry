"""Generate periodic acceleration-constrained Yaw reference tables.

The raw target is a zero-centered sawtooth.  The generated trajectory solves
the periodic quadratic program

    min sum(Q_angle * (theta - theta_raw)^2 + R_accel * alpha^2)
    subject to theta'' = alpha, |alpha| <= alpha_max.

Only this development-time generator uses NumPy.  The STM32 interpolates the
generated angle/rate/acceleration tables and never runs a QP in its 1 kHz loop.
"""

from pathlib import Path

import numpy as np


POINTS = 128
BASE_PEAK_TO_PEAK_DEG = 20.0
FREQUENCIES_HZ = (3, 5)
ACCEL_LEVELS_RAD_S2 = (25.0, 50.0, 100.0, 200.0)
Q_ANGLE = 9.0e6
R_ACCEL = 1.0
MAX_ITERATIONS = 20000
CONVERGENCE_TOLERANCE = 1.0e-7


def project_box_zero_mean(values: np.ndarray, limit: float) -> np.ndarray:
    """Project onto |u| <= limit and sum(u) == 0."""
    lower = float(values.min() - limit)
    upper = float(values.max() + limit)
    for _ in range(40):
        offset = 0.5 * (lower + upper)
        total = np.clip(values - offset, -limit, limit).sum()
        if total > 0.0:
            lower = offset
        else:
            upper = offset
    return np.clip(values - 0.5 * (lower + upper), -limit, limit)


def periodic_position_map(point_count: int, dt_s: float) -> np.ndarray:
    """Return the linear map from periodic acceleration to centered angle."""
    mapping = np.zeros((point_count, point_count), dtype=np.float64)
    for column in range(point_count):
        accel = np.zeros(point_count, dtype=np.float64)
        accel[column] = 1.0
        accel_prefix = np.r_[0.0, np.cumsum(accel[:-1])]
        initial_rate = -dt_s / point_count * (
            accel_prefix.sum() + 0.5 * accel.sum()
        )
        rate = initial_rate + dt_s * accel_prefix
        angle = np.zeros(point_count, dtype=np.float64)
        for index in range(point_count - 1):
            angle[index + 1] = (
                angle[index]
                + dt_s * rate[index]
                + 0.5 * dt_s * dt_s * accel[index]
            )
        mapping[:, column] = angle - angle.mean()
    return mapping


def rotate_to_rising_zero(trajectory: np.ndarray) -> np.ndarray:
    """Choose phase zero at the rising, zero-centered planned angle crossing."""
    point_count = trajectory.shape[0]
    crossing = None
    for index in range(point_count):
        following = (index + 1) % point_count
        if (
            trajectory[index, 0] <= 0.0 < trajectory[following, 0]
            and trajectory[index, 1] > 0.0
        ):
            fraction = -trajectory[index, 0] / (
                trajectory[following, 0] - trajectory[index, 0]
            )
            crossing = index + fraction
            break
    if crossing is None:
        raise RuntimeError("planned trajectory has no rising zero crossing")

    phase = (crossing + np.arange(point_count)) % point_count
    lower = np.floor(phase).astype(int)
    fraction = phase - lower
    upper = (lower + 1) % point_count
    rotated = (
        trajectory[lower] * (1.0 - fraction[:, None])
        + trajectory[upper] * fraction[:, None]
    )
    rotated[0, 0] = 0.0
    return rotated


def solve_table(frequency_hz: float, accel_limit: float) -> np.ndarray:
    period_s = 1.0 / frequency_hz
    dt_s = period_s / POINTS
    position_map = periodic_position_map(POINTS, dt_s)
    regularization = R_ACCEL / Q_ANGLE
    hessian = position_map.T @ position_map + regularization * np.eye(POINTS)
    raw_angle = np.deg2rad(BASE_PEAK_TO_PEAK_DEG) * (
        np.arange(POINTS) / POINTS - 0.5
    )
    raw_angle -= raw_angle.mean()
    gradient_offset = -(position_map.T @ raw_angle)
    lipschitz = np.linalg.eigvalsh(hessian)[-1]

    accel = np.zeros(POINTS, dtype=np.float64)
    extrapolated = accel.copy()
    momentum = 1.0
    for _ in range(MAX_ITERATIONS):
        next_accel = project_box_zero_mean(
            extrapolated - (hessian @ extrapolated + gradient_offset) / lipschitz,
            accel_limit,
        )
        next_momentum = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * momentum * momentum))
        next_extrapolated = next_accel + (
            (momentum - 1.0) / next_momentum * (next_accel - accel)
        )
        if np.max(np.abs(next_accel - accel)) < CONVERGENCE_TOLERANCE:
            accel = next_accel
            break
        accel = next_accel
        extrapolated = next_extrapolated
        momentum = next_momentum
    else:
        raise RuntimeError(
            f"QP failed to converge: {frequency_hz} Hz, {accel_limit} rad/s^2"
        )

    angle = position_map @ accel
    accel_prefix = np.r_[0.0, np.cumsum(accel[:-1])]
    initial_rate = -dt_s / POINTS * (
        accel_prefix.sum() + 0.5 * accel.sum()
    )
    rate = initial_rate + dt_s * accel_prefix
    return rotate_to_rising_zero(np.column_stack((angle, rate, accel)))


def validate_table(
    trajectory: np.ndarray, frequency_hz: float, accel_limit: float
) -> None:
    if not np.isfinite(trajectory).all():
        raise RuntimeError("QP table contains a non-finite value")
    if abs(trajectory[0, 0]) > 1.0e-9 or trajectory[0, 1] <= 0.0:
        raise RuntimeError("QP table phase zero is not the rising zero crossing")
    if trajectory[:, 0].min() >= 0.0 or trajectory[:, 0].max() <= 0.0:
        raise RuntimeError("QP table angle does not cross zero")
    if trajectory[:, 1].min() >= 0.0 or trajectory[:, 1].max() <= 0.0:
        raise RuntimeError("QP table does not contain an early-deceleration return")
    if np.max(np.abs(trajectory[:, 2])) > accel_limit + 1.0e-5:
        raise RuntimeError("QP table violates its acceleration constraint")

    dt_s = 1.0 / (frequency_hz * POINTS)
    numerical_rate = (
        np.roll(trajectory[:, 0], -1) - np.roll(trajectory[:, 0], 1)
    ) / (2.0 * dt_s)
    if np.sqrt(np.mean((numerical_rate - trajectory[:, 1]) ** 2)) > 0.02:
        raise RuntimeError("QP table angle and rate are inconsistent")


def format_array(name: str, values: np.ndarray) -> str:
    def c_float(value: float) -> str:
        text = f"{value:.9g}"
        if "." not in text and "e" not in text:
            text += ".0"
        return text + "f"

    rows = []
    for level in values:
        formatted = [c_float(value) for value in level]
        chunks = [", ".join(formatted[i : i + 8]) for i in range(0, len(formatted), 8)]
        rows.append("    {\n        " + ",\n        ".join(chunks) + "\n    }")
    return (
        f"static const float {name}[YAW_TEST_QP_ACCEL_LEVEL_COUNT]"
        f"[YAW_TEST_QP_REFERENCE_POINTS] = {{\n"
        + ",\n".join(rows)
        + "\n};\n"
    )


def main() -> None:
    tables = {}
    for frequency in FREQUENCIES_HZ:
        frequency_tables = []
        for limit in ACCEL_LEVELS_RAD_S2:
            trajectory = solve_table(frequency, limit)
            validate_table(trajectory, frequency, limit)
            frequency_tables.append(trajectory)
        tables[frequency] = np.stack(frequency_tables)

    lines = [
        "#ifndef YAW_QP_REFERENCE_TABLES_H",
        "#define YAW_QP_REFERENCE_TABLES_H",
        "",
        "/* Generated by tools/yaw_lqr/generate_yaw_qp_reference_tables.py. */",
        f"#define YAW_TEST_QP_REFERENCE_POINTS {POINTS}u",
        f"#define YAW_TEST_QP_ACCEL_LEVEL_COUNT {len(ACCEL_LEVELS_RAD_S2)}u",
        f"#define YAW_TEST_QP_BASE_PEAK_TO_PEAK_DEG {BASE_PEAK_TO_PEAK_DEG:.1f}f",
        "",
        "static const float yaw_test_qp_accel_levels_rad_s2[] = {",
        "    " + ", ".join(f"{value:.1f}f" for value in ACCEL_LEVELS_RAD_S2),
        "};",
        "",
    ]
    for frequency, data in tables.items():
        lines.append(format_array(f"yaw_test_qp_{frequency}hz_angle_rad", data[:, :, 0]))
        lines.append(format_array(f"yaw_test_qp_{frequency}hz_rate_rad_s", data[:, :, 1]))
        lines.append(format_array(f"yaw_test_qp_{frequency}hz_accel_rad_s2", data[:, :, 2]))
    lines.extend(["#endif", ""])

    output = (
        Path(__file__).resolve().parents[2]
        / "application"
        / "test"
        / "yaw_qp_reference_tables.h"
    )
    output.write_text("\n".join(lines), encoding="ascii")
    print(output)


if __name__ == "__main__":
    main()
