# Use a reference-derived LQR and observer-only LESO baseline

The first deployable single-pitch controller follows the proven `D:\Infantry` organization: a second-order computed-acceleration LQR is mapped through the identified inertia, validated gravity and friction terms are added in the generalized Pitch coordinate, and one final conversion produces motor torque. The third-order bandwidth-parameterized LESO uses Pitch Angle correction, the measured gyro for initialization and control feedback, the previous applied torque as its known input, and measured-tick forward-Euler integration. Manual control initially runs the LESO for observation only. LQI and active ESO compensation remain offline comparison features until separate evidence justifies them.

## Consequences

The original LQI plus dual-measurement DLQE design is no longer the deployment baseline. MATLAB must report both physical-torque and acceleration-form LQR gains and must reproduce the firmware LESO update order. Integral, Coulomb friction, and ESO compensation each require an independent validation step before they can affect motor torque.
