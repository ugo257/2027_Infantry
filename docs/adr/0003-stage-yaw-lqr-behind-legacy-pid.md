# Stage direct-drive Yaw LQR behind the legacy PID

Yaw keeps the existing external angle and gyro coordinate used by the working
PID chain. The firmware converts external angle degrees to radians at the Yaw
LQR boundary. The RS485 gyro field already contains the gimbal INS gyro in
rad/s with the established inverse axis sign, so the chassis restores only the
sign. The GM6020 is coaxial, so no linkage Jacobian is used.

Yaw LQR computes generalized yaw torque and converts it once through a signed
`torque_to_current` gain into the GM6020 current reference. This gain is a
separate provisional parameter and is not inferred from PID gains. Because the
DJI motor module's software current PID is not configured for this actuator,
direct takeover uses its raw/open-loop current-command path; the motor reverse
flag is disabled while that path is active so direction is represented once in
`torque_to_current`. For the installed Yaw coordinate that conversion is
negative: positive generalized Yaw torque requires negative GM6020 current.

The runtime stages are `LEGACY`, `SHADOW`, `LOW_TORQUE`, `FULL_LQR`, `INTEGRAL`,
and `ESO_COMP`. The default remains `LEGACY`. Shadow computes diagnostics but
does not alter the established PID output. Low-torque and later stages disable
the cascade PID, visual feedforward, and SMC, and send only the protected LQR
current request. Invalid parameters, feedback, timing, or output cause a
same-tick fallback to the legacy PID path. Reaching the configured current
limit is normal actuator saturation and remains on the bounded LQR path.

## Consequences

Yaw does not receive gravity compensation or an explicit friction model.
Integral action and active LESO compensation are separate commissioning steps.
The existing `GIMBAL_MOTOR_MODE` encoder PID is unchanged. Runtime state and
diagnostic variables use a `yaw_lqr_` prefix so takeover state is visible in
Ozone. Hardware curve inspection remains necessary before advancing stages or
treating provisional parameters as deployable.
