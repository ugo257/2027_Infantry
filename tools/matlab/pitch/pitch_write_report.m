function pitch_write_report(bundle, report_file)
%PITCH_WRITE_REPORT Write a compact Markdown validation report.

fid = fopen(report_file, 'w');
if fid < 0
    error('pitch:report:OpenFailed', 'Cannot open report file: %s', report_file);
end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '# Pitch Offline Simulation Report\n\n');
fprintf(fid, '- Generated: %s\n', string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));
fprintf(fid, '- Parameter status: **PROVISIONAL / NOT FOR DEPLOYMENT**\n');
fprintf(fid, '- Sample time: %.3f ms\n', 1e3 * bundle.cfg.sim.Ts);
fprintf(fid, '- LQI gain: `[%.6g %.6g %.6g]`\n', bundle.designs.lqi.K);
fprintf(fid, '- LQI search scales: `[%.3g %.3g %.3g %.3g]`\n\n', bundle.designs.lqi.scales);
fprintf(fid, '- LQI feedback torque limit: %.3f Nm\n', bundle.cfg.limits.feedback_torque_limit_nm);
fprintf(fid, '- Inertial feedforward limit: %.3f Nm\n', bundle.cfg.limits.inertial_feedforward_limit_nm);
fprintf(fid, '- Provisional actuator time constant/delay: %.3f / %.3f ms\n\n', ...
    1e3 * bundle.cfg.model.actuator_time_constant_s, ...
    1e3 * bundle.cfg.model.actuator_delay_s);

fprintf(fid, '## Structural Checks\n\n');
fprintf(fid, '| Check | Result |\n|---|---:|\n');
fprintf(fid, '| Controllable | %d |\n', bundle.checks.controllable);
fprintf(fid, '| Observable | %d |\n', bundle.checks.observable);
fprintf(fid, '| LQI stable | %d |\n', bundle.checks.lqi_stable);
fprintf(fid, '| Angle-only ESO stable | %d |\n', bundle.checks.eso_angle_stable);
fprintf(fid, '| Dual-measurement ESO stable | %d |\n', bundle.checks.eso_dual_stable);
fprintf(fid, '| Angle-only ESO dynamic test stable | %d |\n', ...
    bundle.checks.eso_angle_dynamic_stable);
fprintf(fid, '| Angle-only ESO angle RMS error (deg) | %.4f |\n', ...
    rad2deg(bundle.checks.eso_angle_theta_rms_error_rad));
fprintf(fid, '| Angle-only ESO rate RMS error (rad/s) | %.4f |\n', ...
    bundle.checks.eso_angle_omega_rms_error_rad_s);
fprintf(fid, '| Ideal tracking metrics passed | %d |\n', bundle.checks.ideal_meets_tracking);
fprintf(fid, '| Saturation recovery passed | %d |\n', ...
    bundle.checks.saturation_recovered);
fprintf(fid, '| CSV identification interface passed | %d |\n', bundle.checks.identification_interface);
fprintf(fid, '| Equilibrium residual (Nm) | %.3e |\n\n', bundle.checks.max_equilibrium_error_nm);

fprintf(fid, '## Scenario Results\n\n');
fprintf(fid, '| Scenario | Controller | Stable | Settle (ms) | Overshoot (deg) | Steady error (deg) | RMS error (deg) | Peak torque (Nm) | Saturation |\n');
fprintf(fid, '|---|---|---:|---:|---:|---:|---:|---:|---:|\n');
for idx = 1:numel(bundle.scenario_results)
    entry = bundle.scenario_results(idx);
    if ~strcmp(entry.result.scenario.kind, 'step')
        continue;
    end
    m = entry.result.metrics;
    fprintf(fid, '| %s | %s | %d | %.1f | %.4f | %.4f | %.4f | %.3f | %.3f |\n', ...
        entry.scenario_name, entry.controller, m.stable, 1e3 * m.settling_time_s, ...
        rad2deg(m.overshoot_rad), rad2deg(m.steady_error_rad), ...
        rad2deg(m.rms_tracking_error_rad), m.peak_torque_nm, m.saturation_fraction);
end

fprintf(fid, '\n## Continuous Tracking Results\n\n');
fprintf(fid, '| Scenario | Controller | Stable | RMS error (deg) | Max error (deg) | Final error (deg) | Gain | Phase lag (deg) |\n');
fprintf(fid, '|---|---|---:|---:|---:|---:|---:|---:|\n');
for idx = 1:numel(bundle.scenario_results)
    entry = bundle.scenario_results(idx);
    if strcmp(entry.result.scenario.kind, 'step')
        continue;
    end
    m = entry.result.metrics;
    fprintf(fid, '| %s | %s | %d | %.4f | %.4f | %.4f | %.4f | %.2f |\n', ...
        entry.scenario_name, entry.controller, m.stable, ...
        rad2deg(m.rms_tracking_error_rad), ...
        rad2deg(m.max_tracking_error_rad), rad2deg(m.final_error_rad), ...
        m.frequency_gain, m.phase_lag_deg);
end

mc = bundle.monte_carlo;
fprintf(fid, '\n## Monte Carlo\n\n');
fprintf(fid, '- Runs: %d\n', mc.run_count);
fprintf(fid, '- Stability rate: %.2f%%\n', 100.0 * mc.stability_rate);
fprintf(fid, '- Full metric pass rate: %.2f%%\n', 100.0 * mc.pass_rate);
fprintf(fid, '- Settling time p50/p95/p99 (ms): %.1f / %.1f / %.1f\n', ...
    1e3 * mc.quantiles.settling_time_s);
fprintf(fid, '- Overshoot p50/p95/p99 (deg): %.4f / %.4f / %.4f\n', ...
    mc.quantiles.overshoot_deg);
fprintf(fid, '- Steady error p50/p95/p99 (deg): %.4f / %.4f / %.4f\n', ...
    mc.quantiles.steady_error_deg);
fprintf(fid, '- Peak torque p50/p95/p99 (Nm): %.3f / %.3f / %.3f\n', ...
    mc.quantiles.peak_torque_nm);
fprintf(fid, '- Saturation fraction p50/p95/p99: %.4f / %.4f / %.4f\n', ...
    mc.quantiles.saturation_fraction);
fprintf(fid, '- Integral magnitude p50/p95/p99 (rad*s): %.5f / %.5f / %.5f\n', ...
    mc.quantiles.max_integral_rad_s);

fprintf(fid, '\n### Worst Samples\n\n');
fprintf(fid, '| Seed | Stable | Pass | J | B | Friction | Linkage | Torque gain | Delay (ms) | Settle (ms) | Overshoot (deg) | Error (deg) | Saturation |\n');
fprintf(fid, '|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for idx = 1:numel(mc.worst_records)
    w = mc.worst_records(idx);
    fprintf(fid, '| %d | %d | %d | %.3f | %.3f | %.3f | %.3f | %.3f | %.2f | %.1f | %.4f | %.4f | %.4f |\n', ...
        w.seed, w.stable, w.passed, w.J_scale, w.B_scale, ...
        w.friction_scale, w.linkage_scale, w.torque_gain, 1e3 * w.delay_s, ...
        1e3 * w.settling_time_s, rad2deg(w.overshoot_rad), ...
        rad2deg(w.steady_error_rad), w.saturation_fraction);
end

fprintf(fid, '\n## Interpretation\n\n');
fprintf(fid, 'These results validate software structure and provisional robustness only. ');
fprintf(fid, 'They are not identified robot parameters and must not be exported to firmware ');
fprintf(fid, 'until linkage, inertia, actuator, friction, and sensor data are measured.\n');
fprintf(fid, 'The full metric pass rate applies all nominal step targets to the broad ');
fprintf(fid, 'pre-identification uncertainty envelope simultaneously; it is not a ');
fprintf(fid, 'deployment acceptance result.\n');
end
