# Yaw LQR staged commissioning

Yaw LQR is wired only into `GIMBAL_GYRO_MODE`. `GIMBAL_MOTOR_MODE` and the
default `YAW_LQR_STAGE_LEGACY` continue to use the existing PID chain.

## Runtime stages

| `yaw_lqr_stage` | Behavior |
|---:|---|
| 0 | Legacy PID only; Yaw LQR state is reset. |
| 1 | Shadow calculation and LESO observation; PID retains authority. |
| 2 | Protected low-current LQR takeover, limited to `+/-800` current counts. |
| 3 | Full configured-current LQR takeover, limited to `+/-3500` current counts. |
| 4 | Full LQR plus integral action. |
| 5 | Integral LQR plus active LESO compensation. |

Do not advance beyond stage 1 until `yaw_lqr_torque_to_current` direction and
magnitude are validated. The defaults are provisional and are not deployment
evidence.

## Ozone variables

The six paired reference/measurement traces are:

- angle: `yaw_lqr_angle_ref_debug`, `yaw_lqr_angle_measure_debug` (deg)
- rate: `yaw_lqr_rate_ref_debug`, `yaw_lqr_rate_measure_debug` (rad/s)
- torque: `yaw_lqr_torque_command_debug`, `yaw_lqr_torque_measure_debug` (nominal Nm)

`yaw_lqr_torque_measure_debug` is inferred from GM6020 current feedback using
the provisional signed conversion. It is not a torque-sensor measurement.
For normal remote angle regulation the rate reference is intentionally zero.

For commissioning, also inspect:

- `yaw_lqr_angle_ref_debug`, `yaw_lqr_angle_measure_debug`
- `yaw_lqr_rate_measure_debug`
- `yaw_lqr_current_pre_limit_debug`, `yaw_lqr_current_command_debug`
- `yaw_lqr_motor_current_debug` (GM6020 measured current)
- `yaw_lqr_torque_feedback_debug`, `yaw_lqr_torque_integral_debug`
- `yaw_lqr_eso_z3_debug`, `yaw_lqr_torque_eso_debug`
- `yaw_lqr_output_valid_debug`, `yaw_lqr_fallback_debug`
- `yaw_lqr_timing_fault_debug`, `yaw_lqr_feedback_fault_debug`
- `yaw_lqr_current_saturation_debug`, `yaw_lqr_current_slew_debug`
- `yaw_lqr_limit_debug`, `yaw_lqr_active_debug`

A positive target error must produce a current request that accelerates yaw in
the positive feedback direction. Correct the sign only through
`yaw_lqr_torque_to_current`. Stage 2 should be attempted with small remote
commands and an operator-ready return to stage 0. Current saturation sets the
limit diagnostic but remains in the bounded LQR path; invalid feedback,
parameters, or timing fall back to PID.

The current slew limit is `240000 count/s`. It keeps a finite transition rate
but allows a full `-3500` to `+3500` reversal in about 29 ms. The previous
`12000 count/s` value needed about 0.58 s for that reversal and introduced an
unstable phase lag in the fitted Yaw plant. It can be inspected or adjusted at
runtime through `yaw_lqr_current_slew_rate_s`.

Enable integral action only after stage 3 tracks without sustained saturation.
Enable active ESO compensation only after the shadow `z3` estimate is bounded,
repeatable, and has the expected disturbance sign.

The initial Yaw gains are PID-derived commissioning references, not identified
LQR gains. The installed coordinate needs negative GM6020 current for positive
Yaw torque, so `torque_to_current=-1000`. Ignoring both integrators and the
speed-loop acceleration term, the existing cascade gives:

- `k_angle = 2500 * 3 * RAD_TO_DEG / 1000 = 429.72`
- `k_rate = 2500 * (1 + 0.1 * RAD_TO_DEG) / 1000 = 16.82`

The external RS485 gyro is already rad/s and has the inverse sign of the
controlled Yaw Angle derivative; the chassis restores that sign once at the
LQR boundary.

## Double-up reference test

The two RC three-position switches in UP starts `YawTest`. Releasing either
switch stops the test immediately and returns the reference to the live remote
command. The test runs in `GIMBAL_GYRO_MODE` and replaces only the Yaw angle
reference; visual feedforward and SMC feedforward are cleared while it is
active.

In the two-board build, the gimbal board reads the switches and transmits the
explicit `yaw_test_double_up` flag; the chassis board receives it and runs the
Yaw test/controller. Both boards therefore need the matching control-frame
firmware for this test trigger.

The deterministic trajectory is:

1. Hold the current angle for 0.5 s.
2. Move the reference to 0 deg at 240 deg/s.
3. Apply 0.55 s holds at `-110, +110, -70, +70, -30, +30, 0` deg.
4. Run a 14 s, 60 deg amplitude chirp from 0.25 Hz to 1.50 Hz.
5. Hold 0 deg until the switches are released.

Test-only fields use the `g_yaw_test_*` prefix. Record
`g_yaw_test_state`, `g_yaw_test_target_debug`,
`g_yaw_test_target_velocity_debug`, `g_yaw_test_chirp_frequency_debug`,
`g_yaw_test_sample_valid`, and `g_yaw_test_finished` when the Ozone channel
budget permits.
