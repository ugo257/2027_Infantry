function mc = pitch_run_monte_carlo(cfg, designs, run_count)
%PITCH_RUN_MONTE_CARLO Robustness sweep with deterministic random seeds.

if nargin < 3
    run_count = cfg.monte_carlo.runs;
end
rng(cfg.random_seed, 'twister');

template = pitch_make_scenarios(cfg);
base = template(strcmp({template.name}, 'step_up_center'));
base.duration_s = cfg.monte_carlo.duration_s;

records = repmat(struct( ...
    'seed', 0, 'base_rad', 0.0, 'step_rad', 0.0, ...
    'J_scale', 1.0, 'B_scale', 1.0, 'friction_scale', 1.0, ...
    'linkage_scale', 1.0, 'torque_gain', 1.0, 'delay_s', 0.0, ...
    'angle_noise_std_rad', 0.0, 'gyro_noise_std_rad_s', 0.0, ...
    'stable', false, 'passed', false, 'settling_time_s', inf, ...
    'overshoot_rad', inf, 'steady_error_rad', inf, ...
    'peak_torque_nm', inf, 'saturation_fraction', 1.0, ...
    'slew_fraction', 1.0, 'max_integral_rad_s', inf, ...
    'failed_safety', true, 'score', inf), run_count, 1);

for idx = 1:run_count
    sample_seed = cfg.random_seed + idx;
    scenario = base;
    scenario.name = sprintf('mc_%04d', idx);
    scenario.amplitude_rad = cfg.monte_carlo.step_rad * sign(rand() - 0.5);
    if scenario.amplitude_rad == 0.0
        scenario.amplitude_rad = cfg.monte_carlo.step_rad;
    end
    target_margin_rad = deg2rad(1.0);
    max_base_rad = cfg.sim.theta_safe_rad - ...
        abs(scenario.amplitude_rad) - target_margin_rad;
    scenario.base_rad = local_uniform(-max_base_rad, max_base_rad);
    scenario.model_scales.J = local_log_uniform(cfg.uncertainty.J_scale);
    scenario.model_scales.B = local_log_uniform(cfg.uncertainty.B_scale);
    scenario.model_scales.friction = local_log_uniform(cfg.uncertainty.friction_scale);
    scenario.model_scales.linkage = local_uniform_range(cfg.uncertainty.linkage_scale);
    scenario.model_scales.torque_gain = local_uniform_range(cfg.uncertainty.torque_gain);
    scenario.model_scales.delay_s = local_uniform_range(cfg.uncertainty.delay_s);

    sample_cfg = cfg;
    sample_cfg.sensor.angle_noise_std_rad = ...
        local_uniform_range(cfg.uncertainty.angle_noise_std_rad);
    sample_cfg.sensor.gyro_noise_std_rad_s = ...
        local_uniform_range(cfg.uncertainty.gyro_noise_std_rad_s);

    result = pitch_simulate(sample_cfg, designs, scenario, 'lqi_eso', sample_seed);
    m = result.metrics;
    passed = m.stable && m.settling_time_s <= cfg.design.target_settle_s && ...
        m.overshoot_rad <= cfg.design.target_overshoot_rad && ...
        m.steady_error_rad <= cfg.design.target_steady_error_rad;

    records(idx).seed = sample_seed;
    records(idx).base_rad = scenario.base_rad;
    records(idx).step_rad = scenario.amplitude_rad;
    records(idx).J_scale = scenario.model_scales.J;
    records(idx).B_scale = scenario.model_scales.B;
    records(idx).friction_scale = scenario.model_scales.friction;
    records(idx).linkage_scale = scenario.model_scales.linkage;
    records(idx).torque_gain = scenario.model_scales.torque_gain;
    records(idx).delay_s = scenario.model_scales.delay_s;
    records(idx).angle_noise_std_rad = sample_cfg.sensor.angle_noise_std_rad;
    records(idx).gyro_noise_std_rad_s = sample_cfg.sensor.gyro_noise_std_rad_s;
    records(idx).stable = m.stable;
    records(idx).passed = passed;
    records(idx).settling_time_s = m.settling_time_s;
    records(idx).overshoot_rad = m.overshoot_rad;
    records(idx).steady_error_rad = m.steady_error_rad;
    records(idx).peak_torque_nm = m.peak_torque_nm;
    records(idx).saturation_fraction = m.saturation_fraction;
    records(idx).slew_fraction = m.slew_fraction;
    records(idx).max_integral_rad_s = m.max_integral_rad_s;
    records(idx).failed_safety = m.failed_safety;
    records(idx).score = local_failure_score(m, cfg);
end

scores = [records.score];
[~, order] = sort(scores, 'descend');
worst_count = min(cfg.monte_carlo.store_worst_count, run_count);
worst_indices = order(1:worst_count);

mc.records = records;
mc.run_count = run_count;
mc.stable_count = nnz([records.stable]);
mc.pass_count = nnz([records.passed]);
mc.stability_rate = mc.stable_count / run_count;
mc.pass_rate = mc.pass_count / run_count;
mc.worst_indices = worst_indices;
mc.worst_records = records(worst_indices);
mc.quantiles.settling_time_s = local_quantiles([records.settling_time_s], [0.5, 0.95, 0.99]);
mc.quantiles.overshoot_deg = rad2deg(local_quantiles([records.overshoot_rad], [0.5, 0.95, 0.99]));
mc.quantiles.steady_error_deg = rad2deg(local_quantiles([records.steady_error_rad], [0.5, 0.95, 0.99]));
mc.quantiles.peak_torque_nm = local_quantiles([records.peak_torque_nm], [0.5, 0.95, 0.99]);
mc.quantiles.saturation_fraction = local_quantiles( ...
    [records.saturation_fraction], [0.5, 0.95, 0.99]);
mc.quantiles.max_integral_rad_s = local_quantiles( ...
    [records.max_integral_rad_s], [0.5, 0.95, 0.99]);
end

function x = local_uniform(lo, hi)
x = lo + (hi - lo) * rand();
end

function x = local_uniform_range(bounds)
x = local_uniform(bounds(1), bounds(2));
end

function x = local_log_uniform(bounds)
x = exp(local_uniform(log(bounds(1)), log(bounds(2))));
end

function score = local_failure_score(m, cfg)
if ~m.stable
    score = 1.0e9 + 1.0e6 * double(m.failed_safety);
    return;
end
if isfinite(m.settling_time_s)
    unsettled_penalty = 0.0;
    settling_time_s = m.settling_time_s;
else
    unsettled_penalty = 1.0e6;
    settling_time_s = cfg.monte_carlo.duration_s;
end
score = unsettled_penalty + ...
    100.0 * max(0.0, settling_time_s - cfg.design.target_settle_s) + ...
    1000.0 * max(0.0, m.overshoot_rad - cfg.design.target_overshoot_rad) + ...
    1000.0 * max(0.0, m.steady_error_rad - cfg.design.target_steady_error_rad) + ...
    0.1 * m.saturation_fraction;
end

function q = local_quantiles(values, probabilities)
values = values(isfinite(values));
if isempty(values)
    q = nan(size(probabilities));
else
    q = quantile(values, probabilities);
end
end
