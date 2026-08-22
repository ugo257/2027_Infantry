function figure_files = pitch_plot_results(bundle, output_dir, save_figures)
%PITCH_PLOT_RESULTS Generate deterministic validation figures.

if nargin < 3
    save_figures = true;
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
figure_files = strings(0, 1);

selected = {'step_up_center', 'disturbance_pulse'};
for s = 1:numel(selected)
    entries = bundle.scenario_results(strcmp({bundle.scenario_results.scenario_name}, selected{s}));
    if isempty(entries)
        continue;
    end
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 760]);
    layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, strrep(selected{s}, '_', '\_'));

    nexttile;
    hold on;
    first_trace = entries(1).result.trace;
    plot(first_trace.t_s, rad2deg(first_trace.raw_ref_rad), 'k--', 'LineWidth', 1.2, ...
        'DisplayName', 'raw reference');
    plot(first_trace.t_s, rad2deg(first_trace.ref_rad), 'Color', [0.4, 0.4, 0.4], ...
        'LineWidth', 1.2, 'DisplayName', 'planned reference');
    for idx = 1:numel(entries)
        tr = entries(idx).result.trace;
        plot(tr.t_s, rad2deg(tr.theta_rad), 'LineWidth', 1.2, ...
            'DisplayName', entries(idx).controller);
    end
    ylabel('Angle (deg)'); grid on; legend('Location', 'best');

    nexttile;
    hold on;
    for idx = 1:numel(entries)
        tr = entries(idx).result.trace;
        plot(tr.t_s, tr.tau_cmd_nm, 'LineWidth', 1.1, ...
            'DisplayName', entries(idx).controller);
    end
    yline(bundle.cfg.limits.torque_soft_nm, 'k:');
    yline(-bundle.cfg.limits.torque_soft_nm, 'k:');
    ylabel('Command torque (Nm)'); grid on;

    eso_entry = entries(strcmp({entries.controller}, 'lqi_eso'));
    nexttile;
    if ~isempty(eso_entry)
        tr = eso_entry(1).result.trace;
        plot(tr.t_s, tr.tau_ff_nm, 'DisplayName', 'feedforward'); hold on;
        plot(tr.t_s, tr.tau_feedback_nm, 'DisplayName', 'LQI');
        plot(tr.t_s, tr.tau_eso_nm, 'DisplayName', 'ESO');
        ylabel('Torque parts (Nm)'); xlabel('Time (s)'); grid on;
        legend('Location', 'best');
    end

    if save_figures
        filename = fullfile(output_dir, selected{s} + ".png");
        exportgraphics(fig, filename, 'Resolution', 160);
        figure_files(end + 1, 1) = string(filename); %#ok<AGROW>
    end
    close(fig);
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1050, 720]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('Monte Carlo (%d runs)', bundle.monte_carlo.run_count));
records = bundle.monte_carlo.records;
nexttile; histogram([records.settling_time_s], 40); xline(bundle.cfg.design.target_settle_s, 'r--');
xlabel('Settling time (s)'); grid on;
nexttile; histogram(rad2deg([records.overshoot_rad]), 40); xline(rad2deg(bundle.cfg.design.target_overshoot_rad), 'r--');
xlabel('Overshoot (deg)'); grid on;
nexttile; histogram(rad2deg([records.steady_error_rad]), 40); xline(rad2deg(bundle.cfg.design.target_steady_error_rad), 'r--');
xlabel('Steady error (deg)'); grid on;
nexttile; histogram([records.saturation_fraction], 40); xlabel('Saturation fraction'); grid on;
if save_figures
    filename = fullfile(output_dir, 'monte_carlo.png');
    exportgraphics(fig, filename, 'Resolution', 160);
    figure_files(end + 1, 1) = string(filename);
end
close(fig);

eso_entries = bundle.scenario_results(strcmp( ...
    {bundle.scenario_results.scenario_name}, 'disturbance_pulse') & strcmp( ...
    {bundle.scenario_results.controller}, 'lqi_eso'));
if ~isempty(eso_entries)
    tr = eso_entries(1).result.trace;
    terms = pitch_model_terms(tr.theta_rad, tr.omega_rad_s, bundle.cfg.model);
    external_accel = tr.disturbance_nm ./ terms.inertia;
    fig = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100, 100, 1100, 760]);
    layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(layout, 'ESO state and lumped-disturbance estimation');

    nexttile;
    plot(tr.t_s, rad2deg(tr.theta_rad), 'LineWidth', 1.2, ...
        'DisplayName', 'true angle'); hold on;
    plot(tr.t_s, rad2deg(tr.eso_theta_rad), '--', 'LineWidth', 1.1, ...
        'DisplayName', 'ESO angle');
    ylabel('Angle (deg)'); grid on; legend('Location', 'best');

    nexttile;
    plot(tr.t_s, tr.omega_rad_s, 'LineWidth', 1.2, ...
        'DisplayName', 'true rate'); hold on;
    plot(tr.t_s, tr.eso_omega_rad_s, '--', 'LineWidth', 1.1, ...
        'DisplayName', 'ESO rate');
    ylabel('Rate (rad/s)'); grid on; legend('Location', 'best');

    nexttile;
    plot(tr.t_s, external_accel, 'LineWidth', 1.2, ...
        'DisplayName', 'external-only equivalent'); hold on;
    plot(tr.t_s, tr.eso_disturbance_rad_s2, '--', 'LineWidth', 1.1, ...
        'DisplayName', 'ESO lumped estimate');
    ylabel('Disturbance (rad/s^2)'); xlabel('Time (s)'); grid on;
    legend('Location', 'best');

    if save_figures
        filename = fullfile(output_dir, 'eso_estimation.png');
        exportgraphics(fig, filename, 'Resolution', 160);
        figure_files(end + 1, 1) = string(filename);
    end
    close(fig);
end

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 900, 620]);
Acl = bundle.designs.lqi.Ad - bundle.designs.lqi.Bd * bundle.designs.lqi.K;
reference_input = bundle.designs.lqi.Ed + ...
    bundle.designs.lqi.Bd * bundle.designs.lqi.K(1);
closed_loop = ss(Acl, reference_input, [1.0, 0.0, 0.0], 0.0, bundle.cfg.sim.Ts);
bode(closed_loop); grid on;
title('Nominal LQI closed-loop frequency response');
if save_figures
    filename = fullfile(output_dir, 'frequency_response.png');
    exportgraphics(fig, filename, 'Resolution', 160);
    figure_files(end + 1, 1) = string(filename);
end
close(fig);
end
