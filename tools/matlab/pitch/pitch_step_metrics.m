function metrics = pitch_step_metrics(t_s, reference_rad, theta_rad, torque_nm, cfg)
%PITCH_STEP_METRICS Calculate robust step-response and safety metrics.

t_s = t_s(:);
reference_rad = reference_rad(:);
theta_rad = theta_rad(:);
torque_nm = torque_nm(:);
finite = isfinite(t_s) & isfinite(reference_rad) & isfinite(theta_rad);

metrics.stable = all(isfinite(theta_rad)) && all(isfinite(torque_nm)) && ...
    any(finite) && max(abs(theta_rad(finite))) <= cfg.sim.theta_safe_rad;
metrics.settling_time_s = inf;
metrics.overshoot_rad = inf;
metrics.steady_error_rad = inf;
metrics.peak_torque_nm = max(abs(torque_nm), [], 'omitnan');
metrics.rms_tracking_error_rad = inf;

if nnz(finite) < 5
    return;
end
t = t_s(finite);
r = reference_rad(finite);
y = theta_rad(finite);

changes = find(abs(diff(r)) > deg2rad(0.05));
if isempty(changes)
    start_idx = 1;
else
    start_idx = changes(1) + 1;
end
initial = y(max(1, start_idx - 1));
target = median(r(max(start_idx, numel(r) - max(5, round(0.05 / cfg.sim.Ts))):end));
direction = sign(target - initial);
if direction == 0.0
    direction = 1.0;
end

post_y = y(start_idx:end);
post_t = t(start_idx:end);
metrics.rms_tracking_error_rad = sqrt(mean((post_y - r(start_idx:end)).^2));
if direction > 0.0
    metrics.overshoot_rad = max(0.0, max(post_y) - target);
else
    metrics.overshoot_rad = max(0.0, target - min(post_y));
end

tail_count = max(5, round(0.05 / cfg.sim.Ts));
tail_start = max(start_idx, numel(y) - tail_count + 1);
metrics.steady_error_rad = abs(mean(y(tail_start:end)) - target);

tolerance = max(0.02 * abs(target - initial), cfg.design.target_steady_error_rad);
inside = abs(post_y - target) <= tolerance;
for idx = 1:numel(inside)
    if all(inside(idx:end))
        metrics.settling_time_s = post_t(idx) - post_t(1);
        break;
    end
end
end
