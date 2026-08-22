function scenarios = pitch_make_scenarios(cfg)
%PITCH_MAKE_SCENARIOS Deterministic validation scenario catalog.

empty_scales = struct('J', 1.0, 'B', 1.0, 'friction', 1.0, ...
    'linkage', 1.0, 'torque_gain', 1.0, 'delay_s', NaN);

scenarios = repmat(struct( ...
    'name', '', 'kind', '', 'plant_kind', 'realistic', ...
    'duration_s', cfg.sim.duration_s, 'base_rad', 0.0, ...
    'start_s', 0.10, 'amplitude_rad', deg2rad(5.0), ...
    'frequency_hz', 0.0, 'ramp_rate_rad_s', 0.0, ...
    'disturbance_start_s', 0.0, 'disturbance_end_s', 0.0, ...
    'disturbance_nm', 0.0, 'jump_time_s', inf, ...
    'jump_J_scale', 1.0, 'model_scales', empty_scales), 0, 1);

scenarios(end + 1) = local_scenario('ideal_step_up', 'step', 'ideal', ...
    0.8, 0.0, 0.10, deg2rad(5.0), empty_scales);
scenarios(end + 1) = local_scenario('step_up_center', 'step', 'realistic', ...
    1.0, 0.0, 0.10, deg2rad(5.0), empty_scales);
scenarios(end + 1) = local_scenario('step_down_center', 'step', 'realistic', ...
    1.0, 0.0, 0.10, -deg2rad(5.0), empty_scales);
scenarios(end + 1) = local_scenario('step_up_low_angle', 'step', 'realistic', ...
    1.0, -0.22, 0.10, deg2rad(5.0), empty_scales);
scenarios(end + 1) = local_scenario('step_down_high_angle', 'step', 'realistic', ...
    1.0, 0.22, 0.10, -deg2rad(5.0), empty_scales);

ramp = local_scenario('remote_ramp', 'ramp', 'realistic', ...
    1.2, -deg2rad(4.0), 0.10, deg2rad(8.0), empty_scales);
ramp.ramp_rate_rad_s = deg2rad(18.0);
scenarios(end + 1) = ramp;

for frequency_hz = [0.5, 1.0, 2.0, 3.0]
    sine_case = local_scenario(sprintf('sine_%g_hz', frequency_hz), ...
        'sine', 'realistic', 2.5, 0.0, 0.25, deg2rad(2.0), empty_scales);
    sine_case.frequency_hz = frequency_hz;
    scenarios(end + 1) = sine_case; %#ok<AGROW>
end

disturbance = local_scenario('disturbance_pulse', 'step', 'realistic', ...
    1.2, 0.0, 0.10, deg2rad(3.0), empty_scales);
disturbance.disturbance_start_s = 0.45;
disturbance.disturbance_end_s = 0.52;
disturbance.disturbance_nm = 0.30;
scenarios(end + 1) = disturbance;

jump_case = local_scenario('inertia_jump', 'step', 'realistic', ...
    1.2, 0.0, 0.10, deg2rad(4.0), empty_scales);
jump_case.jump_time_s = 0.45;
jump_case.jump_J_scale = 1.8;
scenarios(end + 1) = jump_case;

delay_stress = local_scenario('delay_stress_5ms', 'step', 'realistic', ...
    1.2, 0.0, 0.10, -deg2rad(5.0), empty_scales);
delay_stress.model_scales.delay_s = 0.005;
scenarios(end + 1) = delay_stress;
end

function scenario = local_scenario(name, kind, plant_kind, duration_s, ...
    base_rad, start_s, amplitude_rad, scales)
scenario.name = name;
scenario.kind = kind;
scenario.plant_kind = plant_kind;
scenario.duration_s = duration_s;
scenario.base_rad = base_rad;
scenario.start_s = start_s;
scenario.amplitude_rad = amplitude_rad;
scenario.frequency_hz = 0.0;
scenario.ramp_rate_rad_s = 0.0;
scenario.disturbance_start_s = 0.0;
scenario.disturbance_end_s = 0.0;
scenario.disturbance_nm = 0.0;
scenario.jump_time_s = inf;
scenario.jump_J_scale = 1.0;
scenario.model_scales = scales;
end
