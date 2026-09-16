# Yaw LQR staged commissioning

Yaw LQR is wired only into `GIMBAL_GYRO_MODE`. `GIMBAL_MOTOR_MODE` and the
default `YAW_LQR_STAGE_LEGACY` continue to use the existing PID chain.

## Runtime stages

| `yaw_lqr_stage` | Behavior |
|---:|---|
| 0 | Legacy PID only; Yaw LQR state is reset. |
| 1 | Shadow calculation and LESO observation; PID retains authority. |
| 2 | Protected low-current LQR takeover, limited to `+/-800` current counts. |
| 3 | Full configured-current LQR takeover, limited to `+/-30000` command counts. |
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
For manual remote angle regulation the rate reference is generated from the
remote angle command; vision mode may provide its own visual velocity path.

For commissioning, also inspect:

- `yaw_lqr_control_dt_debug` (s), `yaw_lqr_current_to_accel_gain`
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
- `g_yaw_remote_ref_vel_enable`, `yaw_remote_ref_vel_debug`
- `yaw_remote_ref_vel_raw_debug`, `yaw_remote_ref_vel_accel_debug`

For ordinary manual remote control, the Yaw reference velocity is derived from
the degree-based remote angle command using the same filtered, bounded and
slew-limited pattern as the manual Pitch reference velocity. The LQR receives
this reference in `rad/s`. Vision mode continues to use its existing visual
velocity/current feedforward path, and the trajectory test supplies its own
planned reference velocity.

A positive target error must produce a current request that accelerates yaw in
the positive feedback direction. Correct the sign only through
`yaw_lqr_torque_to_current`. Stage 2 should be attempted with small remote
commands and an operator-ready return to stage 0. Current saturation sets the
limit diagnostic but remains in the bounded LQR path; invalid feedback,
parameters, or timing fall back to PID.

The stage 3-5 limit is `+/-30000`, matching the GM6020 command range documented
by the project motor module. The DJI send layer applies this wider limit only to
GM6020 instances; its other motor types retain their existing `+/-16384` clamp.
Stage 2 remains limited to `+/-800`. The current slew limit is `2070000 count/s`,
which keeps a finite transition rate while allowing a full `-30000` to `+30000`
reversal in about 29 ms. Leaving the former `240000 count/s` slew value in place
would stretch that reversal to 250 ms and hide much of the benefit of the higher
current limit. The slew rate can be inspected or adjusted at runtime through
`yaw_lqr_current_slew_rate_s`.

Enable integral action only after stage 3 tracks without sustained saturation.
Enable active ESO compensation only after the shadow `z3` estimate is bounded,
repeatable, and has the expected disturbance sign.

The observer uses the explicitly identified equivalent input gain
`yaw_lqr_current_to_accel_gain=-0.0092 rad/s^2/count`. This quantity maps the
previous applied GM6020 command to Yaw acceleration. It replaces the old ESO
input calculation based on the uncalibrated product of inertia and
torque-to-current gain. It does not change the PID-derived feedback gains.

Yaw test timing, observer integration, and current slew limiting use the same
DWT-measured Gimbal control period exposed as `yaw_lqr_control_dt_debug`.
Motor-feedback packet `dt` is not reused as task-loop time.

The initial Yaw gains are PID-derived commissioning references, not identified
LQR gains. The installed coordinate needs negative GM6020 current for positive
Yaw torque, so `torque_to_current=-1000`. Ignoring both integrators and the
speed-loop acceleration term, the existing cascade gives:

- `k_angle = 2500 * 3 * RAD_TO_DEG / 1000 = 429.72`
- `k_rate = 2500 * (1 + 0.1 * RAD_TO_DEG) / 1000 = 16.82`

The external RS485 gyro is already rad/s and has the inverse sign of the
controlled Yaw Angle derivative; the chassis restores that sign once at the
LQR boundary.

## Double-up trajectory tracking test

The two RC three-position switches in UP starts `YawTest`. Releasing either
switch stops the test immediately and returns the reference to the live remote
command. The test should be started with `yaw_lqr_stage=3`. It captures the
measured entry angle as the trajectory center and generates a periodic,
acceleration-limited QP reference around that center. Visual feedforward, SMC
feedforward, and manual remote velocity reference are cleared while it is
active. The coherent angle/rate/acceleration reference is passed to the LQR.
Acceleration feedforward uses the identified equivalent input gain
`current_ff=alpha_ref/current_to_accel_gain` and is enabled by default.

The test parameters are latched when double-up starts:

- `g_yaw_test_wave_frequency_hz`: selects the nearest standard table, `3.0` or `5.0`.
- `g_yaw_test_wave_peak_to_peak_deg`: raw sawtooth span, default `20 deg` (`+/-10 deg`).
- `g_yaw_test_max_accel_rad_s2`: hard planning constraint, default `50 rad/s^2`.
- `g_yaw_test_accel_ff_enable`: optional equivalent-current acceleration
  feedforward, default `1`.

The raw target is a zero-centered sawtooth: angle rises linearly through the
configured span and resets at the cycle boundary. A development-time periodic
QP minimizes angle tracking error plus acceleration effort using the same
`Q_angle=9e6`, `Q_rate=0`, `R_accel=1` structure as the referenced planner,
subject to `|alpha| <= alpha_max`. The generated tables contain mutually
consistent angle, rate, and acceleration. The firmware linearly interpolates
phase and acceleration-table levels in its 1 kHz loop; it does not solve a QP
on the STM32. The planned excursion can be less than the raw 20 deg span when
the hard acceleration constraint is active.

The test state machine does not abort on transient feedback, readiness, angle,
or rate diagnostics; those conditions are left to the recorded validity fields
and the operator during this model-identification pass.

In the two-board build, the gimbal board reads the switches and transmits the
explicit `yaw_test_double_up` flag; the chassis board receives it and runs the
Yaw test/controller. Both boards therefore need the matching control-frame
firmware for this test trigger.

The deterministic test sequence is:

1. Enter table phase zero through a `0.5 s` quintic transition. These samples
   are not marked valid.
2. Run the selected trajectory for `10 s`.
3. Continue holding the entry angle while double-up remains held.

Releasing either switch is the normal stop action and immediately returns control
to the live remote command.

The Gimbal task stack is 1024 bytes for this build. The previous 512-byte stack
had insufficient margin for the floating-point controller and an interrupt-time
FreeRTOS context save at the deepest previous test call site.

State 7 remains reserved as the abort code. The trajectory test does not enter it
for ordinary tracking excursions; validity, feedback, timing, current limits,
and fallback remain visible in the normal LQR diagnostics.

For trajectory tracking, record:

- `g_yaw_test_state`, `g_yaw_test_sample_valid`
- `g_yaw_test_target_debug`, `g_yaw_test_ref_rate_debug`
- `g_yaw_test_ref_accel_debug`, `g_yaw_test_wave_frequency_debug`
- `g_yaw_test_wave_accel_limit_debug`
- `g_yaw_test_planned_peak_to_peak_debug`
- `g_yaw_test_planner_transition_debug`
- `yaw_lqr_angle_ref_debug`, `yaw_lqr_angle_measure_debug`
- `yaw_lqr_rate_ref_debug`, `yaw_lqr_rate_measure_debug`
- `yaw_lqr_accel_ref_debug`
- `yaw_lqr_current_accel_ff_debug`
- `yaw_lqr_current_command_debug`, `yaw_lqr_motor_current_debug`
- `yaw_lqr_current_saturation_debug`
- `yaw_lqr_active_debug`, `yaw_lqr_output_valid_debug`, `yaw_lqr_fallback_debug`

Use only state 2 rows with `sample_valid=1`. Drop rows with saturation,
inactive LQR, invalid output, or fallback before calculating tracking error and
phase delay. State 1 is the quintic entry and state 3 is the post-test hold.
