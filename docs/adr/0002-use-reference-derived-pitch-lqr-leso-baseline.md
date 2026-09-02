# Use a reference-derived LQR and observer-only LESO baseline

The first deployable single-pitch controller follows the proven `D:\Infantry` organization: a second-order computed-acceleration LQR is mapped through the identified inertia, validated gravity and friction terms are added in the generalized Pitch coordinate, and one final conversion produces motor torque. The third-order bandwidth-parameterized LESO uses Pitch Angle correction, the measured gyro for initialization and control feedback, the previous applied torque as its known input, and measured-tick forward-Euler integration. Manual control initially runs the LESO for observation only. LQI and active ESO compensation remain offline comparison features until separate evidence justifies them.

The current hardware inspection confirms a non-coaxial crank/connecting-rod linkage between the DM motor and the Pitch body. Therefore the motor/Pitch conversion is not assumed to be unity: with motor angle `q` and controlled Pitch angle `theta`, the signed local linkage ratio is `lambda(q) = dq/dtheta`, and generalized Pitch torque is `tau_pitch = lambda(q) * tau_motor`. The corresponding rate mapping is `theta_dot = (dtheta/dq) * q_dot`. A unit ratio remains only a provisional diagnostic value until a measured monotonic linkage map is available.

This changes the commissioning order: measured `q <-> theta` mapping and its derivative must be validated before dynamic `J/B/Fc` identification or low-torque LQR takeover. Motor velocity may be recorded for linkage identification, but the controller's Pitch Rate remains the calibrated, axis-consistent gyro signal.

## Consequences

The original LQI plus dual-measurement DLQE design is no longer the deployment baseline. MATLAB must report both physical-torque and acceleration-form LQR gains and must reproduce the firmware LESO update order. Integral, Coulomb friction, and ESO compensation each require an independent validation step before they can affect motor torque.

The direct-drive simplification is explicitly rejected for the present mechanism. Until the linkage Jacobian is measured, force/torque and velocity comparisons involving motor variables are diagnostic only and must not be used to claim a valid Pitch-axis model.
