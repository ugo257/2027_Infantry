# Keep theoretical MATLAB and deployed firmware pitch control mathematically equivalent

The proven `D:\Infantry` controller is the first source for controller structure, equations, commissioning, safety, and telemetry patterns. Where it has no applicable single-pitch method, the reviewed single-pitch recommendation is used. MATLAB is a theoretical verification and consistency environment, not the authority for deployable gains or performance acceptance. For simulation conclusions to remain relevant, its state definitions, equations, observer, compensation ordering, limits, and reset behavior must still match the selected firmware method and be checked with shared input traces. Deployment decisions are made from the current robot's measured data, Ozone curves, protection behavior, and operator evaluation.

## Consequences

Every exported controller revision needs a MATLAB-versus-firmware replay check before Torque Takeover. A firmware-only compensation or a MATLAB-only dynamic is a parity failure, not an acceptable tuning difference. Passing MATLAB does not make a gain deployable; it only clears the theoretical verification gate.
