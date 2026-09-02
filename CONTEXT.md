# Gimbal Pitch Control

This context covers the single-axis gimbal pitch motion that aims the barrel at a commanded elevation while respecting the actuator and mechanical limits.

## Language

**Pitch Angle**:
The barrel/gimbal elevation relative to the world frame, measured by the gimbal INS. It is the primary controlled coordinate.
_Avoid_: Motor angle, linkage angle

**Pitch Command**:
The desired Pitch Angle produced by the active command source. Manual remote control is the first acceptance surface.
_Avoid_: Motor command, torque command

**Pitch Reference**:
The angle, rate, and acceleration target presented to the pitch controller. The initial manual-control baseline follows the Pitch Command without an additional angle lag and uses zero rate and acceleration targets.
_Avoid_: Motor reference, filtered command

**Manual Control Baseline**:
The first deployable single-pitch controller: second-order LQR in Pitch Angle and Pitch Rate, model feedforward, one final torque protection path, and observer-only Pitch LESO. Integral action and ESO compensation are disabled.
_Avoid_: LQI baseline, active disturbance compensation

**Pitch Acceleration Command**:
The computed-acceleration LQR result alpha_ref - k_theta e_theta - k_omega e_omega. The identified Pitch inertia maps it to generalized feedback torque before model terms are added.
_Avoid_: Motor acceleration, torque command

**Motor Angle**:
The DM actuator shaft position used to describe linkage geometry and actuator state; it is not the Pitch Command.
_Avoid_: Pitch angle

**Pitch Linkage Mechanism**:
The current hardware uses a motor crank, connecting rod, and Pitch-body attachment rather than a coaxial motor/Pitch joint. Motor angle and Pitch angle therefore have a position-dependent signed Jacobian.
_Avoid_: Direct-drive assumption, constant gear ratio

**Pitch Rate**:
The world-frame angular rate corresponding to Pitch Angle, measured from the calibrated and axis-mapped gimbal gyro.
_Avoid_: Angle-difference speed, motor speed

**Pitch-Rate Mapping**:
The local conversion from motor speed to controlled Pitch Rate, `theta_dot = (dtheta/dq) q_dot`. Motor velocity is not a substitute for Pitch Rate until this mapping and its sign have been validated.
_Avoid_: Motor speed equals Pitch Rate

**Pitch Torque Command**:
The single final torque request sent to the pitch motor after control, model compensation, and protection have been reconciled for the current control tick.
_Avoid_: Feedback torque, individual compensation term

**Applied Pitch Torque**:
The previous tick's Pitch Torque Command after every limit and slew constraint, used as the default actuator input known to the observer.
_Avoid_: Raw controller torque, reported motor torque

**Generalized Pitch Torque**:
Torque expressed in the Pitch Angle coordinate. The plant model uses this domain before a single linkage conversion to motor-shaft torque.
_Avoid_: Motor torque, unspecified torque

**Linkage Ratio**:
The signed local derivative of Motor Angle with respect to Pitch Angle, `dq/dtheta`, obtained from the measured monotonic linkage map. It is generally position-dependent for the crank-rod mechanism.
_Avoid_: Gear ratio, constant transmission ratio

**Equivalent Input Gain**:
The experimentally observable acceleration gain from the commanded actuator input after coordinate conversion. It may combine motor torque gain and Pitch inertia until either quantity is independently calibrated.
_Avoid_: Identified inertia, motor torque constant

**Pitch LESO**:
The third-order linear extended-state observer adopted from the proven project. Its states estimate Pitch Angle, Pitch Rate, and lumped acceleration disturbance, with gains parameterized by observer bandwidth as beta1 = 3 w0, beta2 = 3 w0^2, and beta3 = w0^3. It is corrected by Pitch Angle, initialized with the measured gyro rate, advanced by forward Euler using the validated measured tick time, and driven by the previous Applied Pitch Torque.
_Avoid_: Kalman filter, dual-measurement DLQE

**Observer-only ESO**:
A Pitch LESO whose estimates are recorded but whose disturbance estimate does not alter the Pitch Torque Command. This is the first manual-control ESO stage.
_Avoid_: ESO compensation

**Active ESO Compensation**:
The explicitly enabled use of an ESO disturbance estimate in the Pitch Torque Command after observer-only validation.
_Avoid_: ESO observation

**Validated Model Term**:
A gravity or friction term that reduces residual error on an independent Validation Dataset after its parameters have been fixed on the Fit Dataset.
_Avoid_: Better fit on training data, provisional parameter

**Pitch Control Snapshot**:
A coherent record of command, reference, measured state, controller terms, protection state, and timing from one control tick.
_Avoid_: Uncoordinated Live Watch variables

**Shadow Control**:
A commissioning state in which the candidate controller computes estimates and a hypothetical Pitch Torque Command but has no authority over the motor.
_Avoid_: Low-torque control, disabled telemetry

**Torque Takeover**:
The staged transfer of motor torque authority from the accepted controller to a candidate controller, advancing only when evidence from the current stage passes its safety and performance gates.
_Avoid_: Controller enable, immediate replacement

**Provisional Safety Envelope**:
The offline angle, speed, torque, and slew constraints used before the current robot's mechanical and actuator limits are measured. These values exercise protection logic but are not deployable hardware limits.
_Avoid_: Verified limit, copied reference-robot limit

**Pitch Error Band**:
The achievable steady tracking tolerance derived from stationary sensor noise and repeated mechanical positioning data.
_Avoid_: Mathematical zero error, arbitrary fixed tolerance

**Timing Fault**:
A control tick whose elapsed time violates the accepted discrete-controller timing window, invalidating dynamic-state advancement for that tick.
_Avoid_: Ordinary sampling jitter

**Takeover Evidence**:
The parameter snapshot, coherent control data, test description, and operator observations used to decide whether Torque Takeover advances, holds, or rolls back.
_Avoid_: Best-looking screenshot, verbal-only result

**Deployment Evidence**:
Evidence from the current robot's synchronized CSV data, Ozone curves, protection behavior, and operator evaluation. It is the authority for deployable gains and performance acceptance; MATLAB results provide theoretical support only.
_Avoid_: Simulation-only acceptance, reference-robot performance

**Linkage Identification Evidence**:
A measured set of paired Motor Angle/Pitch Angle samples covering the valid working range, including direction, monotonicity, local derivative, and singularity/dead-zone checks. It is required before torque takeover.
_Avoid_: Photograph-only geometry, assumed unit transmission

**Identification Snapshot**:
A valid, settled sample emitted by an armed identification state machine together with its experiment phase, protection state, and configuration.
_Avoid_: Arbitrary telemetry row, hand-picked point

**Fit Dataset**:
Recorded experiments permitted to determine model parameters.
_Avoid_: Validation data, all available logs

**Validation Dataset**:
Independent recorded experiments used only to test predictions from parameters already fixed from the Fit Dataset.
_Avoid_: Training data, refitting data
