function design = pitch_design_lqi(cfg, linear)
%PITCH_DESIGN_LQI Tune a discrete LQI gain from Bryson-rule candidates.

Aaug = [linear.A, zeros(2, 1); 1.0, 0.0, 0.0];
Baug = [linear.B; 0.0];
Eaug = [0.0; 0.0; -1.0];
aug_d = c2d(ss(Aaug, [Baug, Eaug], eye(3), zeros(3, 2)), ...
    cfg.sim.Ts, 'zoh');
Ad = aug_d.A;
Bd = aug_d.B(:, 1);
Ed = aug_d.B(:, 2);

q_base = [1.0 / cfg.design.theta_error_max_rad^2, ...
          1.0 / cfg.design.omega_error_max_rad_s^2, ...
          1.0 / cfg.design.integral_error_max_rad_s^2];
r_base = 1.0 / cfg.limits.feedback_torque_budget_nm^2;

best.score = inf;
best.K = [NaN, NaN, NaN];
best.metrics = struct();
best.scales = [NaN, NaN, NaN, NaN];
feasible_best = best;
feasible_best.settling_time_s = inf;
candidate_count = numel(cfg.design.lqi_theta_scales) * ...
    numel(cfg.design.lqi_omega_scales) * ...
    numel(cfg.design.lqi_integral_scales) * ...
    numel(cfg.design.lqi_effort_scales);
candidate_rows = nan(candidate_count, 14);
candidate_index = 0;

for st = cfg.design.lqi_theta_scales
    for sw = cfg.design.lqi_omega_scales
        for si = cfg.design.lqi_integral_scales
            for sr = cfg.design.lqi_effort_scales
                Q = diag(q_base .* [st, sw, si]);
                R = r_base * sr;
                [K, ~, poles] = dlqr(Ad, Bd, Q, R);
                candidate = local_evaluate_candidate(Ad, Bd, K, cfg, linear);
                candidate.max_pole_abs = max(abs(poles));
                score = local_score(candidate, cfg);
                candidate_index = candidate_index + 1;
                candidate_rows(candidate_index, :) = [st, sw, si, sr, K, ...
                    candidate.settling_time_s, candidate.overshoot_rad, ...
                    candidate.steady_error_rad, candidate.peak_torque_nm, ...
                    candidate.max_pole_abs, candidate.stable, score];
                if score < best.score
                    best.score = score;
                    best.K = K;
                    best.metrics = candidate;
                    best.scales = [st, sw, si, sr];
                    best.Q = Q;
                    best.R = R;
                    best.poles = poles;
                end
                if local_meets_targets(candidate, cfg) && ...
                        candidate.settling_time_s < feasible_best.settling_time_s
                    feasible_best.score = score;
                    feasible_best.K = K;
                    feasible_best.metrics = candidate;
                    feasible_best.scales = [st, sw, si, sr];
                    feasible_best.Q = Q;
                    feasible_best.R = R;
                    feasible_best.poles = poles;
                    feasible_best.settling_time_s = candidate.settling_time_s;
                end
            end
        end
    end
end

if all(isfinite(feasible_best.K))
    best = rmfield(feasible_best, 'settling_time_s');
end

if any(~isfinite(best.K))
    error('pitch:lqi:NoDesign', 'No finite LQI candidate was found.');
end

design = best;
design.Ad = Ad;
design.Bd = Bd;
design.Ed = Ed;
design.Aaug_c = Aaug;
design.Baug_c = Baug;
design.provisional = true;
design.candidates = array2table(candidate_rows, 'VariableNames', { ...
    'theta_scale', 'omega_scale', 'integral_scale', 'effort_scale', ...
    'k_theta', 'k_omega', 'k_integral', 'settling_time_s', ...
    'overshoot_rad', 'steady_error_rad', 'peak_torque_nm', ...
    'max_pole_abs', 'stable', 'score'});
end

function metrics = local_evaluate_candidate(Ad, Bd, K, cfg, linear)
N = round(1.0 / cfg.sim.Ts) + 1;
raw_step = deg2rad(5.0);
plant_state = zeros(2, 1);
integral = 0.0;
ref_state = struct('theta', 0.0, 'omega', 0.0, 'alpha', 0.0);
command_last = 0.0;
y = zeros(N, 1);
u = zeros(N, 1);
raw_reference = zeros(N, 1);
for k = 1:N
    t = (k - 1) * cfg.sim.Ts;
    raw_reference(k) = raw_step * (t >= 0.10);
    [ref_state, ref] = pitch_reference_step(ref_state, raw_reference(k), ...
        cfg.sim.Ts, cfg);
    error_theta = plant_state(1) - ref.theta_rad;
    error_omega = plant_state(2) - ref.omega_rad_s;
    integral = integral + cfg.sim.Ts * error_theta;
    integral = min(max(integral, -cfg.limits.integral_rad_s), ...
        cfg.limits.integral_rad_s);
    feedback_torque = -K * [error_theta; error_omega; integral];
    feedback_torque = min(max(feedback_torque, ...
        -cfg.limits.feedback_torque_limit_nm), ...
        cfg.limits.feedback_torque_limit_nm);
    inertial_feedforward = linear.terms.inertia * ref.alpha_rad_s2 / ...
        linear.terms.ratio;
    inertial_feedforward = min(max(inertial_feedforward, ...
        -cfg.limits.inertial_feedforward_limit_nm), ...
        cfg.limits.inertial_feedforward_limit_nm);
    % Friction/damping is part of the disturbance d. Only gravity is an
    % explicit model compensation in the deployment baseline.
    feedforward_torque = inertial_feedforward;
    unsaturated = feedforward_torque + feedback_torque;
    limited = min(max(unsaturated, -cfg.limits.torque_soft_nm), ...
        cfg.limits.torque_soft_nm);
    max_delta = cfg.limits.torque_slew_nm_s * cfg.sim.Ts;
    u(k) = min(max(limited, command_last - max_delta), command_last + max_delta);
    if (unsaturated > limited && error_theta < 0.0) || ...
            (unsaturated < limited && error_theta > 0.0)
        integral = integral - cfg.sim.Ts * error_theta;
    end
    plant_state = Ad(1:2, 1:2) * plant_state + Bd(1:2) * u(k);
    y(k) = plant_state(1);
    command_last = u(k);
end
t = (0:N-1)' * cfg.sim.Ts;
metrics = pitch_step_metrics(t, raw_reference, y, u, cfg);
end

function passed = local_meets_targets(metrics, cfg)
passed = metrics.stable && ...
    metrics.settling_time_s <= cfg.design.target_settle_s && ...
    metrics.overshoot_rad <= cfg.design.target_overshoot_rad && ...
    metrics.steady_error_rad <= cfg.design.target_steady_error_rad;
end

function score = local_score(m, cfg)
settling_time = m.settling_time_s;
if ~isfinite(settling_time)
    settling_time = 2.0;
end
overshoot = m.overshoot_rad;
if ~isfinite(overshoot)
    overshoot = deg2rad(10.0);
end
steady_error = m.steady_error_rad;
if ~isfinite(steady_error)
    steady_error = deg2rad(10.0);
end
score = settling_time / cfg.design.target_settle_s + ...
        8.0 * overshoot / cfg.design.target_overshoot_rad + ...
        4.0 * steady_error / cfg.design.target_steady_error_rad + ...
        0.02 * m.peak_torque_nm / cfg.limits.torque_soft_nm;
if ~m.stable
    score = score + 1.0e6;
end
end
