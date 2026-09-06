# Pitch Gravity Identification Sampling

The pitch identification state machine performs a slow continuous scan from
-20 to +20 degrees and back again. It marks only moving, low-acceleration
samples as valid; endpoint dwell rows are deliberately invalid. Averaging the
forward and reverse scans offline suppresses direction-dependent friction.
Ozone Data Sampling records synchronized raw data; no on-board storage is
required.

## Ozone setup

Add these expressions to Watched Data and Data Sampling:

```text
g_pitch_lqr_stage
g_pitch_test_state
g_pitch_test_direction
g_pitch_test_sample_valid
g_pitch_test_sample_count
g_pitch_test_target_debug
g_pitch_test_target_velocity_debug
g_pitch_test_gyro_acceleration_debug
pitch_auto_lqr_output_valid_debug
pitch_auto_lqr_timing_fault_debug
pitch_auto_lqr_feedback_fault_debug
pitch_auto_lqr_theta_meas_debug
pitch_auto_lqr_omega_meas_debug
pitch_auto_lqr_tau_lqr_debug
pitch_auto_lqr_tau_cmd_debug
pitch_auto_lqr_tau_applied_debug
pitch_motor_pos_debug
pitch_motor_vel_debug
pitch_motor_torque_feedback_debug
pitch_motor_torque_command_debug
pitch_motor_feedback_state_debug
pitch_auto_lqr_limit_debug
```

The list above is exactly 22 sampled variables. Keep `g_pitch_test_enable`,
`g_pitch_test_abort`, and `gimbal_cmd_recv.gimbal_mode` in Watched Data for
control, but do not add them to the sampled list.

Use a Data Sampling rate of 100-200 Hz. Watch refresh can remain at 5 Hz. Do
not use the old `build/Ozone_DataSampling_260519.csv`: it contains leg/chassis
signals, not pitch signals.

## Fixed operator sequence

1. Secure the robot, remove ammunition, and confirm that the full +/-20 degree
   motion is mechanically safe.
2. Load the flashed ELF and press `Debug -> Go`.
3. Set `g_pitch_lqr_stage=2` (`PITCH_LQR_STAGE_LOW_TORQUE`). This keeps the
   identification path free of the existing gravity feedforward while retaining
   the conservative LQR limit. If a point saturates, do not use that point.
4. Confirm `gimbal_mode=1`, `output_valid=1`, and all fault/fallback flags are
   zero before starting.
5. Start Data Sampling, then set `g_pitch_test_enable=1` (or use the physical
   test switch). Keep the remote pitch command neutral.
6. Let the state machine run for at least one complete forward-and-reverse
   sweep. The target moves continuously at about 0.10 rad/s; endpoint dwell is
   only for reversal safety. `g_pitch_test_sample_valid=1` marks usable moving
   samples (gyro and motor velocity nonzero, estimated acceleration small).
7. Stop by setting `g_pitch_test_abort=1` or releasing the physical request.
   Stop immediately for wrong direction, noise, a limit approach, or loss of
   feedback.
8. Stop Data Sampling and export the CSV. Keep the parameter snapshot and note
   which points reached the torque limit.

For fitting, retain rows with `g_pitch_test_sample_valid=1`, split by the sign
of `g_pitch_test_target_velocity_debug` (or measured Pitch velocity), and
bin/interpolate both sweeps onto a common angle grid. `g_pitch_test_direction`
describes endpoint traversal, not necessarily the instantaneous motion sign
during the initial move. Compute the gravity estimate as the average of
matched forward and reverse torque values. Discard rows near the endpoints,
near a torque limit, or with obvious feedback faults.

The operator does not need to inspect the curves during the motion. Stop
immediately if the physical direction is wrong, the mechanism approaches a
limit, or the motor becomes noisy.

## Signal/feedback direction check

Before model identification, set `g_pitch_test_signal_check=1` and start the
same test request. The state machine then uses a +/-5 degree range at about
0.20 rad/s. Compare `g_pitch_test_target_velocity_debug` with
`pitch_auto_lqr_omega_meas_debug` and `pitch_motor_vel_debug`. The
`g_pitch_test_sign_fault` bit mask reports a Pitch-gyro mismatch in bit 0 and
a motor-velocity mismatch in bit 1. A zero flag is not sufficient evidence if
the measured velocity never exceeds the 0.03 rad/s check threshold, so inspect
the curves and the actual angle change as well.

## Dynamic J/B excitation

After the signal directions are confirmed, set `g_pitch_test_signal_check=0`
and `g_pitch_test_dynamic_enable=1` before starting the test request. The
state machine then runs eight cycles of a smooth trajectory:

```text
theta_ref = theta_center + 0.10*sin(2*pi*0.50*t) rad
omega_ref = 0.10*(2*pi*0.50)*cos(2*pi*0.50*t) rad/s
alpha_ref = -0.10*(2*pi*0.50)^2*sin(2*pi*0.50*t) rad/s^2
```

The center is captured from the measured angle and limited to +/-0.20 rad, so
the commanded range remains inside the +/-20 degree working range. Start with
the mechanism near its middle position. In this mode `g_pitch_test_state=6`
and `g_pitch_test_sample_valid=1` marks finite online samples; do not use the
old low-acceleration validity rule. Remove rows with
`pitch_auto_lqr_limit_debug=1`, feedback faults, startup, and the short stop
at the end before fitting. Confirm `g_pitch_test_mode_debug=2` after starting;
if it remains 0 or 1, the dynamic mode was not selected and the file must not
be used for J/B fitting.

## Offline report

From the repository root:

```powershell
python tools/pitch_logging/analyze_pitch_log.py path\to\Ozone_DataSampling_pitch.csv
```

The script prints duration, sample timing, fault counts, angle/velocity/torque
statistics and ESO disturbance range. If matplotlib is installed it also writes
`Ozone_DataSampling_pitch_pitch_report.png` beside the CSV.

The report is diagnostic only. It does not authorize changing the stage or
exporting gains. Send the CSV, printed report, and generated PNG before moving
to the gravity-enabled LQR stage.
