function [state, out] = pitch_reference_step(state, raw_target_rad, dt_s, cfg)
%PITCH_REFERENCE_STEP Critically damped reference model with hard limits.

wn = cfg.reference.bandwidth_rad_s;
if ~isfield(state, 'alpha')
    state.alpha = 0.0;
end
state.alpha = wn^2 * (raw_target_rad - state.theta) - ...
    2.0 * wn * state.omega;
state.alpha = min(max(state.alpha, -cfg.reference.acceleration_limit_rad_s2), ...
    cfg.reference.acceleration_limit_rad_s2);

state.omega = state.omega + dt_s * state.alpha;
state.omega = min(max(state.omega, -cfg.reference.velocity_limit_rad_s), ...
    cfg.reference.velocity_limit_rad_s);
state.theta = state.theta + dt_s * state.omega;

out.theta_rad = state.theta;
out.omega_rad_s = state.omega;
out.alpha_rad_s2 = state.alpha;
end
