function result = pitch_simulate(cfg, designs, scenario, controller_name, seed)
%PITCH_SIMULATE Simulate one controller against one pitch scenario.

if nargin < 5
    seed = cfg.random_seed;
end
rng(seed, 'twister');

[controller_cfg, true_cfg, jitter_fraction] = local_configs(cfg, scenario);
Ts = cfg.sim.Ts;
N = floor(scenario.duration_s / Ts) + 1;

fields = {'t_s', 'dt_s', 'raw_ref_rad', 'ref_rad', 'ref_omega_rad_s', ...
    'ref_alpha_rad_s2', 'theta_rad', 'omega_rad_s', 'theta_meas_rad', ...
    'gyro_meas_rad_s', 'tau_cmd_nm', 'tau_actual_nm', 'tau_meas_nm', ...
    'tau_ff_nm', 'tau_feedback_nm', 'tau_eso_nm', 'disturbance_nm', ...
    'eso_theta_rad', 'eso_omega_rad_s', 'eso_disturbance_rad_s2', ...
    'integral_rad_s', 'saturated', 'slew_limited'};
for idx = 1:numel(fields)
    trace.(fields{idx}) = nan(N, 1);
end

theta = scenario.base_rad;
omega = 0.0;
initial_terms = pitch_model_terms(theta, omega, true_cfg.model);
tau_equilibrium = initial_terms.gravity_generalized_nm / initial_terms.ratio;
tau_actual = tau_equilibrium;
tau_cmd_last = tau_equilibrium / max(true_cfg.model.torque_gain, eps);

delay_steps = max(0, round(true_cfg.model.actuator_delay_s / Ts));
delay_buffer = tau_cmd_last * ones(delay_steps + 1, 1);
ref_state = struct('theta', scenario.base_rad, 'omega', 0.0, 'alpha', 0.0);
ctrl = local_init_controller(controller_name, theta, omega, tau_cmd_last);
t = 0.0;
failed = false;

for k = 1:N
    dt = Ts * (1.0 + jitter_fraction * (2.0 * rand() - 1.0));
    if k == 1
        dt = Ts;
    end

    raw_ref = local_raw_reference(scenario, t);
    [ref_state, ref] = pitch_reference_step(ref_state, raw_ref, dt, controller_cfg);
    disturbance = local_disturbance(scenario, t) + true_cfg.model.disturbance_nm;

    theta_meas = theta + true_cfg.sensor.angle_bias_rad + ...
        true_cfg.sensor.angle_noise_std_rad * randn();
    if true_cfg.sensor.quantization_angle_rad > 0.0
        q = true_cfg.sensor.quantization_angle_rad;
        theta_meas = round(theta_meas / q) * q;
    end
    gyro_meas = omega + true_cfg.sensor.gyro_bias_rad_s + ...
        true_cfg.sensor.gyro_noise_std_rad_s * randn();
    tau_meas = tau_actual + true_cfg.sensor.torque_noise_std_nm * randn();

    [ctrl, control] = local_control_step(ctrl, controller_name, designs, ...
        controller_cfg, ref, theta_meas, gyro_meas, tau_meas, dt);

    tau_pre_limit = control.tau_ff_nm + control.tau_feedback_nm + control.tau_eso_nm;
    tau_limited = min(max(tau_pre_limit, -cfg.limits.torque_soft_nm), ...
        cfg.limits.torque_soft_nm);
    saturated = abs(tau_limited - tau_pre_limit) > 1.0e-9;
    tau_limited = min(max(tau_limited, -cfg.limits.torque_hard_nm), ...
        cfg.limits.torque_hard_nm);
    max_delta = cfg.limits.torque_slew_nm_s * dt;
    tau_cmd = min(max(tau_limited, tau_cmd_last - max_delta), ...
        tau_cmd_last + max_delta);
    slew_limited = abs(tau_cmd - tau_limited) > 1.0e-9;

    ctrl = local_antiwindup(ctrl, controller_name, tau_pre_limit, tau_limited, ...
        dt, designs, cfg);

    delay_buffer = [delay_buffer(2:end); tau_cmd];
    delayed_cmd = delay_buffer(1);
    if true_cfg.model.actuator_time_constant_s > 0.0
        tau_dot = (true_cfg.model.torque_gain * delayed_cmd - tau_actual) / ...
            true_cfg.model.actuator_time_constant_s;
        tau_actual = tau_actual + dt * tau_dot;
    else
        tau_actual = true_cfg.model.torque_gain * delayed_cmd;
    end

    dynamic_model = true_cfg.model;
    if t >= scenario.jump_time_s
        dynamic_model.J_kg_m2 = dynamic_model.J_kg_m2 * scenario.jump_J_scale;
    end
    terms = pitch_model_terms(theta, omega, dynamic_model);
    alpha = (terms.ratio * tau_actual - terms.coriolis_nm - ...
        terms.gravity_generalized_nm - terms.viscous_nm - ...
        terms.coulomb_generalized_nm + disturbance) / terms.inertia;
    omega = omega + dt * alpha;
    theta = theta + dt * omega;

    trace.t_s(k) = t;
    trace.dt_s(k) = dt;
    trace.raw_ref_rad(k) = raw_ref;
    trace.ref_rad(k) = ref.theta_rad;
    trace.ref_omega_rad_s(k) = ref.omega_rad_s;
    trace.ref_alpha_rad_s2(k) = ref.alpha_rad_s2;
    trace.theta_rad(k) = theta;
    trace.omega_rad_s(k) = omega;
    trace.theta_meas_rad(k) = theta_meas;
    trace.gyro_meas_rad_s(k) = gyro_meas;
    trace.tau_cmd_nm(k) = tau_cmd;
    trace.tau_actual_nm(k) = tau_actual;
    trace.tau_meas_nm(k) = tau_meas;
    trace.tau_ff_nm(k) = control.tau_ff_nm;
    trace.tau_feedback_nm(k) = control.tau_feedback_nm;
    trace.tau_eso_nm(k) = control.tau_eso_nm;
    trace.disturbance_nm(k) = disturbance;
    trace.eso_theta_rad(k) = control.eso_theta_rad;
    trace.eso_omega_rad_s(k) = control.eso_omega_rad_s;
    trace.eso_disturbance_rad_s2(k) = control.eso_disturbance_rad_s2;
    trace.integral_rad_s(k) = ctrl.integral;
    trace.saturated(k) = saturated;
    trace.slew_limited(k) = slew_limited;

    if any(~isfinite([theta, omega, tau_actual, tau_cmd])) || ...
            abs(theta) > cfg.sim.theta_safe_rad || ...
            abs(omega) > cfg.sim.omega_safe_rad_s
        failed = true;
        break;
    end

    tau_cmd_last = tau_cmd;
    ctrl.tau_cmd_last = tau_cmd;
    t = t + dt;
end

last = find(isfinite(trace.t_s), 1, 'last');
if isempty(last)
    last = 1;
end
names = fieldnames(trace);
for idx = 1:numel(names)
    trace.(names{idx}) = trace.(names{idx})(1:last);
end

metrics = pitch_step_metrics(trace.t_s, trace.raw_ref_rad, trace.theta_rad, ...
    trace.tau_cmd_nm, cfg);
metrics.metric_type = scenario.kind;
metrics.max_tracking_error_rad = max(abs(trace.theta_rad - trace.raw_ref_rad), ...
    [], 'omitnan');
metrics.final_error_rad = metrics.steady_error_rad;
metrics.frequency_gain = NaN;
metrics.phase_lag_deg = NaN;
metrics.analysis_start_s = scenario.start_s;
if ~strcmp(scenario.kind, 'step')
    tracking = pitch_tracking_metrics(trace.t_s, trace.raw_ref_rad, ...
        trace.theta_rad, scenario);
    metrics.settling_time_s = NaN;
    metrics.overshoot_rad = NaN;
    metrics.steady_error_rad = NaN;
    tracking_names = fieldnames(tracking);
    for idx = 1:numel(tracking_names)
        metrics.(tracking_names{idx}) = tracking.(tracking_names{idx});
    end
end
metrics.failed_safety = failed;
metrics.saturation_fraction = mean(trace.saturated > 0.5, 'omitnan');
metrics.slew_fraction = mean(trace.slew_limited > 0.5, 'omitnan');
metrics.max_integral_rad_s = max(abs(trace.integral_rad_s), [], 'omitnan');
metrics.stable = metrics.stable && ~failed;

result.controller = controller_name;
result.scenario = scenario;
result.trace = trace;
result.metrics = metrics;
end

function [controller_cfg, true_cfg, jitter_fraction] = local_configs(cfg, scenario)
controller_cfg = cfg;
true_cfg = cfg;
scales = scenario.model_scales;
true_cfg.model.J_kg_m2 = cfg.model.J_kg_m2 * scales.J;
true_cfg.model.B_nms_rad = cfg.model.B_nms_rad * scales.B;
true_cfg.model.coulomb_nm = cfg.model.coulomb_nm * scales.friction;
true_cfg.model.linkage_ratio = cfg.model.linkage_ratio * scales.linkage;
true_cfg.model.torque_gain = scales.torque_gain;
if isfinite(scales.delay_s)
    true_cfg.model.actuator_delay_s = scales.delay_s;
end
jitter_fraction = cfg.sim.jitter_fraction;

if strcmp(scenario.plant_kind, 'ideal')
    nominal_terms = pitch_model_terms(0.0, 0.0, controller_cfg.model);
    controller_cfg.model.J_kg_m2 = nominal_terms.inertia;
    controller_cfg.model.linkage_ratio = nominal_terms.ratio;
    controller_cfg.model.inertia_sin_fraction = 0.0;
    controller_cfg.model.linkage_sin_fraction = 0.0;
    controller_cfg.model.gravity_poly_nm = [0.0, 0.0, 0.0];
    controller_cfg.model.gravity_min_nm = 0.0;
    controller_cfg.model.gravity_max_nm = 0.0;
    controller_cfg.model.coulomb_nm = 0.0;
    true_cfg = controller_cfg;
    true_cfg.model.actuator_time_constant_s = 0.0;
    true_cfg.model.actuator_delay_s = 0.0;
    true_cfg.sensor.angle_noise_std_rad = 0.0;
    true_cfg.sensor.gyro_noise_std_rad_s = 0.0;
    true_cfg.sensor.torque_noise_std_nm = 0.0;
    jitter_fraction = 0.0;
end
end

function ctrl = local_init_controller(name, theta, omega, tau_cmd)
ctrl.integral = 0.0;
ctrl.tau_cmd_last = tau_cmd;
ctrl.eso_x = [theta; omega; 0.0];
ctrl.legacy_z = [theta; omega; 0.0];
ctrl.legacy_lqr_last = 0.0;
ctrl.observer_initialized = false;
ctrl.name = name;
end

function [ctrl, out] = local_control_step(ctrl, name, designs, cfg, ref, ...
    theta_meas, gyro_meas, tau_meas, dt)
out = struct('tau_ff_nm', 0.0, 'tau_feedback_nm', 0.0, ...
    'tau_eso_nm', 0.0, 'eso_theta_rad', theta_meas, ...
    'eso_omega_rad_s', gyro_meas, 'eso_disturbance_rad_s2', 0.0);

if strcmp(name, 'legacy')
    [ctrl, out] = local_legacy_step(ctrl, cfg, ref, theta_meas, gyro_meas, dt);
    return;
end

[ctrl, observer] = local_eso_step(ctrl, designs.eso, cfg, theta_meas, ...
    gyro_meas, tau_meas, dt);
use_eso = strcmp(name, 'lqi_eso');
if use_eso
    theta_feedback = observer.theta_rad;
    omega_feedback = observer.omega_rad_s;
else
    theta_feedback = theta_meas;
    omega_feedback = gyro_meas;
end

ctrl.integral = ctrl.integral + dt * (theta_feedback - ref.theta_rad);
ctrl.integral = min(max(ctrl.integral, -cfg.limits.integral_rad_s), ...
    cfg.limits.integral_rad_s);
error_state = [theta_feedback - ref.theta_rad; ...
               omega_feedback - ref.omega_rad_s; ctrl.integral];
ctrl.last_error_theta = error_state(1);
out.tau_feedback_nm = -designs.lqi.K * error_state;
out.tau_feedback_nm = min(max(out.tau_feedback_nm, ...
    -cfg.limits.feedback_torque_limit_nm), ...
    cfg.limits.feedback_torque_limit_nm);

terms_ref = pitch_model_terms(ref.theta_rad, ref.omega_rad_s, cfg.model);
inertial_ff = terms_ref.inertia * ref.alpha_rad_s2 / ...
    max(terms_ref.ratio * cfg.model.torque_gain, eps);
inertial_ff = min(max(inertial_ff, ...
    -cfg.limits.inertial_feedforward_limit_nm), ...
    cfg.limits.inertial_feedforward_limit_nm);
% Deployment baseline explicitly compensates gravity only. Viscous/Coulomb
% friction and other unmodeled effects remain in the plant disturbance d.
generalized_ff = terms_ref.gravity_generalized_nm;
out.tau_ff_nm = inertial_ff + generalized_ff / ...
    max(terms_ref.ratio * cfg.model.torque_gain, eps);

if use_eso
    out.tau_eso_nm = -terms_ref.inertia * observer.disturbance_rad_s2 / ...
        max(terms_ref.ratio * cfg.model.torque_gain, eps);
    out.tau_eso_nm = min(max(out.tau_eso_nm, -cfg.limits.eso_comp_nm), ...
        cfg.limits.eso_comp_nm);
end
out.eso_theta_rad = observer.theta_rad;
out.eso_omega_rad_s = observer.omega_rad_s;
out.eso_disturbance_rad_s2 = observer.disturbance_rad_s2;
end

function [ctrl, observer] = local_eso_step(ctrl, eso, cfg, theta_meas, ...
    gyro_meas, tau_meas, dt)
if ~ctrl.observer_initialized
    ctrl.eso_x = [theta_meas; gyro_meas; 0.0];
    ctrl.observer_initialized = true;
end

x = ctrl.eso_x;
terms = pitch_model_terms(x(1), x(2), cfg.model);
known_nonlinear_accel = -terms.gravity_generalized_nm / ...
    terms.inertia;
x_pred = eso.Ad * x + eso.Bd * tau_meas;
x_pred(2) = x_pred(2) + dt * known_nonlinear_accel;
x_pred(1) = x_pred(1) + 0.5 * dt^2 * known_nonlinear_accel;

if eso.use_dual_measurement
    innovation = [theta_meas; gyro_meas] - eso.C_dual * x_pred;
    x = x_pred + eso.L_dual * innovation;
else
    innovation = theta_meas - eso.C_angle * x_pred;
    x = x_pred + eso.L_angle * innovation;
end
ctrl.eso_x = x;
observer.theta_rad = x(1);
observer.omega_rad_s = x(2);
observer.disturbance_rad_s2 = x(3);
end

function [ctrl, out] = local_legacy_step(ctrl, cfg, ref, theta, omega, dt)
w0 = cfg.legacy.eso_bandwidth_rad_s;
b0 = 1.0 / cfg.model.J_kg_m2;
e_obs = theta - ctrl.legacy_z(1);
z1_dot = ctrl.legacy_z(2) + 3.0 * w0 * e_obs;
z2_dot = -(cfg.model.B_nms_rad / cfg.model.J_kg_m2) * ctrl.legacy_z(2) + ...
    b0 * ctrl.legacy_lqr_last + ctrl.legacy_z(3) + 3.0 * w0^2 * e_obs;
z3_dot = w0^3 * e_obs;
ctrl.legacy_z = ctrl.legacy_z + dt * [z1_dot; z2_dot; z3_dot];

e_theta = theta - ref.theta_rad;
e_omega = omega - ref.omega_rad_s;
ctrl.integral = ctrl.integral + dt * e_theta;
ctrl.integral = min(max(ctrl.integral, -cfg.legacy.integral_limit_rad_s), ...
    cfg.legacy.integral_limit_rad_s);
tau_dynamic_ff = cfg.model.J_kg_m2 * ref.alpha_rad_s2 + ...
    cfg.model.B_nms_rad * ref.omega_rad_s + cfg.model.coulomb_nm * ...
    tanh(ref.omega_rad_s / max(cfg.model.coulomb_smooth_rad_s, eps));
tau_lqr = tau_dynamic_ff - cfg.legacy.k_theta * e_theta - ...
    cfg.legacy.k_omega * e_omega - cfg.legacy.k_i * ctrl.integral;
tau_eso = -cfg.legacy.eso_comp_gain * ctrl.legacy_z(3) / b0;
tau_eso = min(max(tau_eso, -cfg.legacy.eso_comp_limit_nm), ...
    cfg.legacy.eso_comp_limit_nm);
terms = pitch_model_terms(ref.theta_rad, ref.omega_rad_s, cfg.model);
gravity_motor = terms.gravity_generalized_nm / max(terms.ratio, eps);

out.tau_ff_nm = gravity_motor;
out.tau_feedback_nm = tau_lqr - cfg.legacy.speed_pid_kp * omega;
out.tau_eso_nm = tau_eso;
out.eso_theta_rad = ctrl.legacy_z(1);
out.eso_omega_rad_s = ctrl.legacy_z(2);
out.eso_disturbance_rad_s2 = ctrl.legacy_z(3);
ctrl.legacy_lqr_last = tau_lqr + tau_eso;
end

function ctrl = local_antiwindup(ctrl, name, u_pre, u_applied, dt, ~, cfg)
if strcmp(name, 'legacy')
    return;
end
if (u_pre > u_applied && ctrl.last_error_theta < 0.0) || ...
        (u_pre < u_applied && ctrl.last_error_theta > 0.0)
    ctrl.integral = ctrl.integral - dt * ctrl.last_error_theta;
end
ctrl.integral = min(max(ctrl.integral, -cfg.limits.integral_rad_s), ...
    cfg.limits.integral_rad_s);
end

function raw_ref = local_raw_reference(scenario, t)
switch scenario.kind
    case 'step'
        raw_ref = scenario.base_rad + scenario.amplitude_rad * (t >= scenario.start_s);
    case 'ramp'
        progress = max(0.0, t - scenario.start_s) * scenario.ramp_rate_rad_s;
        progress = min(progress, abs(scenario.amplitude_rad));
        raw_ref = scenario.base_rad + sign(scenario.amplitude_rad) * progress;
    case 'sine'
        phase_t = max(0.0, t - scenario.start_s);
        raw_ref = scenario.base_rad;
        if t >= scenario.start_s
            raw_ref = raw_ref + scenario.amplitude_rad * ...
                sin(2.0 * pi * scenario.frequency_hz * phase_t);
        end
    otherwise
        error('pitch:scenario:UnknownKind', 'Unknown scenario kind: %s', scenario.kind);
end
end

function disturbance = local_disturbance(scenario, t)
active = t >= scenario.disturbance_start_s && t < scenario.disturbance_end_s;
disturbance = scenario.disturbance_nm * active;
end
