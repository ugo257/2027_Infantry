function metrics = pitch_tracking_metrics(t_s, reference_rad, theta_rad, scenario)
%PITCH_TRACKING_METRICS Metrics for ramp and sinusoidal tracking scenarios.

t_s = t_s(:);
reference_rad = reference_rad(:);
theta_rad = theta_rad(:);
finite = isfinite(t_s) & isfinite(reference_rad) & isfinite(theta_rad);

metrics.rms_tracking_error_rad = inf;
metrics.max_tracking_error_rad = inf;
metrics.final_error_rad = inf;
metrics.frequency_gain = NaN;
metrics.phase_lag_deg = NaN;
metrics.analysis_start_s = NaN;

if nnz(finite) < 5
    return;
end

t = t_s(finite);
r = reference_rad(finite);
y = theta_rad(finite);
analysis_start = scenario.start_s;
if strcmp(scenario.kind, 'sine') && scenario.frequency_hz > 0.0
    analysis_start = scenario.start_s + 0.5 / scenario.frequency_hz;
end
use = t >= analysis_start;
if nnz(use) < 5
    use = t >= scenario.start_s;
end
if nnz(use) < 5
    return;
end

t_fit = t(use) - scenario.start_s;
r_fit = r(use);
y_fit = y(use);
error_rad = y_fit - r_fit;
metrics.rms_tracking_error_rad = sqrt(mean(error_rad.^2));
metrics.max_tracking_error_rad = max(abs(error_rad));
tail_start = max(1, numel(error_rad) - 49);
metrics.final_error_rad = abs(mean(error_rad(tail_start:end)));
metrics.analysis_start_s = t(find(use, 1, 'first'));

if strcmp(scenario.kind, 'sine') && scenario.frequency_hz > 0.0
    omega = 2.0 * pi * scenario.frequency_hz;
    basis = [sin(omega * t_fit), cos(omega * t_fit), ones(size(t_fit))];
    reference_fit = basis \ r_fit;
    output_fit = basis \ y_fit;
    reference_amplitude = hypot(reference_fit(1), reference_fit(2));
    output_amplitude = hypot(output_fit(1), output_fit(2));
    metrics.frequency_gain = output_amplitude / max(reference_amplitude, eps);
    reference_phase = atan2(reference_fit(2), reference_fit(1));
    output_phase = atan2(output_fit(2), output_fit(1));
    metrics.phase_lag_deg = mod(rad2deg(reference_phase - output_phase) + ...
        180.0, 360.0) - 180.0;
end
end
