# Pitch Shadow Sampling (方案 A)

This workflow uses Ozone Data Sampling. The firmware remains unchanged and
`g_pitch_lqr_stage` must stay at `1`; the old cascade still controls the motor.

## Ozone setup

Add these expressions to Watched Data and Data Sampling:

```text
g_pitch_lqr_stage
gimbal_cmd_recv.gimbal_mode
pitch_auto_lqr_dt_debug
pitch_auto_lqr_output_valid_debug
pitch_auto_lqr_timing_fault_debug
pitch_auto_lqr_feedback_fault_debug
pitch_auto_lqr_fallback_debug
pitch_auto_lqr_theta_ref_debug
pitch_auto_lqr_theta_meas_debug
pitch_auto_lqr_err_debug
pitch_auto_lqr_omega_meas_debug
pitch_auto_lqr_tau_lqr_debug
pitch_auto_lqr_tau_cmd_debug
pitch_auto_lqr_tau_applied_debug
pitch_motor->ctrl.tor_set
pitch_motor->measure.tor
pitch_auto_lqr_eso_z1_debug
pitch_auto_lqr_eso_z2_debug
pitch_auto_lqr_eso_z3_debug
pitch_auto_lqr_eso_w0_debug
```

Use a Data Sampling rate of 100-200 Hz. Watch refresh can remain at 5 Hz. Do
not use the old `build/Ozone_DataSampling_260519.csv`: it contains leg/chassis
signals, not pitch signals.

## Fixed operator sequence

1. Load the ELF that was flashed to the board and press `Debug -> Go`.
2. Confirm `g_pitch_lqr_stage=1`, `gimbal_mode=1`, `output_valid=1`, and all
   fault/fallback flags are zero.
3. Start Data Sampling and wait 3 seconds without moving the remote.
4. Move pitch slowly in one direction by about 2 degrees and hold 2 seconds.
5. Return to the starting point and hold 2 seconds.
6. Move slowly about 2 degrees in the opposite direction and hold 2 seconds.
7. Return to the starting point and hold 3 seconds.
8. Stop Data Sampling and export the CSV to this directory.

The operator does not need to inspect the curves during the motion. Stop
immediately if the physical direction is wrong, the mechanism approaches a
limit, or the motor becomes noisy.

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
to stage 2.
