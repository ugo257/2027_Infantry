# Yaw LQR offline tools

`yaw_qp_lqr_simulation.m` reads the reference tables actually compiled into
the firmware, plots the raw sawtooth and planned angle/rate/acceleration, and
replays the current Yaw LQR law against the identified plant model.

Run from MATLAB:

```matlab
cd('D:/2027_Infantry/tools/yaw_lqr')
yaw_qp_lqr_simulation
yaw_qp_lqr_simulation('FrequencyHz', 5)
```

The default simulation uses:

- `3 Hz`, raw `20 deg` peak-to-peak, `50 rad/s^2` planning limit;
- controller gains, current conversion, saturation, and slew macros read from
  `application/robot_def.h`;
- `alpha = current_to_accel_gain * current - 1.55 * omega` as the plant;
- the current firmware equivalent-current acceleration feedforward
  (`alpha_ref / current_to_accel_gain`) enabled, with a second trace showing
  the same controller with it disabled.

The `1.55 1/s` damping is the accepted small-signal replay value already used
by `yaw_lqr_closed_loop_regression.py`. It can be changed without touching the
firmware:

```matlab
yaw_qp_lqr_simulation('ViscousDampingPerS', 0)
```

PNG figures, a full trace CSV, and a metrics CSV are written to `results/`.
The simulation is a deterministic model check, not a replacement for the Ozone
test: it omits sensor noise, CAN delay, motor-current dynamics, friction, and
unmodelled load torque.
