# Pitch Identification Test

Both RC switches up are the only start request. Release either switch to stop.
Before raising both switches, set `g_pitch_test_mode_select` in Ozone Watch:

```text
0 = normal constant-speed identification scan
1 = 2 Hz high-speed sine tracking evaluation
```

The default is 0. Mode 0 scans -15 to +15 degrees in both directions at 0.30,
0.50, and 0.70 rad/s for identification. Mode 1 is only for evaluating
high-speed tracking: it commands a zero-centered +/-20 degree sine at 2 Hz
for 24 cycles (two complete cycles per second). Its theoretical target speed
peak is 4.39 rad/s and target acceleration peak is 55.2 rad/s^2; the state
machine uses 8 rad/s and 160
rad/s^2 safety limits for this mode.

## States

```text
0 idle
1 two-second arm
2 move to -15 degrees
3 start hold
4 constant-speed sweep
5 endpoint hold
6 complete
7 safety abort
8 2 Hz sine excitation
9 sine zero alignment before excitation
```

## Operation

1. Rebuild and flash the latest EIDE target.
2. Put Pitch near its internal zero and release the Pitch stick.
3. Set `g_pitch_test_mode_select` to 0 or 1 in Ozone Watch.
4. Start Ozone Data Sampling at 200 Hz.
5. Move both RC switches up and keep them up.
6. Mode 0 completes at `g_pitch_test_state=6`; mode 1 completes at
   `g_pitch_test_state=6` after the sine cycles.
7. Release either switch, stop sampling, and export the CSV.

Only use rows satisfying all conditions below for fitting:

```text
g_pitch_test_state == 4 (mode 0) or 8 (mode 1)
g_pitch_test_sample_valid == 1
pitch_auto_lqr_output_valid_debug == 1
pitch_auto_lqr_feedback_fault_debug == 0
pitch_auto_lqr_timing_fault_debug == 0
pitch_auto_lqr_limit_debug == 0
```

Discard startup, arming, positioning, endpoint holds, completion, aborted,
invalid, and torque-limited rows.

## Ozone sampled variables (22)

```text
g_pitch_test_state
g_pitch_test_sample_valid
g_pitch_test_abort_reason
g_pitch_test_speed_index
g_pitch_test_direction
g_pitch_test_sweep_speed_debug
g_pitch_test_target_debug
g_pitch_test_target_velocity_debug
g_pitch_test_target_acceleration_debug
g_pitch_test_gyro_acceleration_debug
pitch_auto_lqr_theta_meas_debug
pitch_auto_lqr_omega_meas_debug
pitch_gyro_raw_0_debug
pitch_motor_pos_debug
pitch_motor_vel_debug
pitch_motor_torque_feedback_debug
pitch_auto_lqr_tau_lqr_debug
pitch_auto_lqr_tau_applied_debug
pitch_auto_lqr_output_valid_debug
pitch_auto_lqr_feedback_fault_debug
pitch_auto_lqr_timing_fault_debug
pitch_auto_lqr_limit_debug
```

`g_pitch_test_mode_select` is a Watch/control variable only; do not add it to
Data Sampling.
