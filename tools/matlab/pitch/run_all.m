function bundle = run_all(options)
%RUN_ALL Build, simulate, validate, and report the offline pitch design.

defaults.monte_carlo_runs = 1000;
defaults.save_figures = true;
defaults.output_dir = "";
if nargin < 1
    options = struct();
end
if ~isstruct(options) || ~isscalar(options)
    error('pitch:run:InvalidOptions', 'options must be a scalar struct.');
end
names = fieldnames(defaults);
for idx = 1:numel(names)
    name = names{idx};
    if ~isfield(options, name)
        options.(name) = defaults.(name);
    end
end
validateattributes(options.monte_carlo_runs, {'double'}, ...
    {'scalar', 'integer', 'positive'});
validateattributes(options.save_figures, {'logical'}, {'scalar'});
options.output_dir = string(options.output_dir);

root_dir = fileparts(mfilename('fullpath'));
addpath(root_dir);
if strlength(options.output_dir) == 0
    output_dir = fullfile(root_dir, 'results');
else
    output_dir = char(options.output_dir);
end
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

fprintf('Pitch offline simulation: loading provisional configuration...\n');
cfg = pitch_default_config();
cfg.monte_carlo.runs = options.monte_carlo_runs;
linear = pitch_linearize_model(cfg, 0.0);
designs.lqi = pitch_design_lqi(cfg, linear);
designs.eso = pitch_design_eso(cfg, linear);
checks = pitch_self_test(cfg, linear, designs);

fprintf('Running deterministic scenario matrix...\n');
scenarios = pitch_make_scenarios(cfg);
controllers = {'legacy', 'lqi', 'lqi_eso'};
scenario_results = repmat(struct('scenario_name', '', 'controller', '', ...
    'result', struct()), numel(scenarios) * numel(controllers), 1);
entry = 0;
for scenario_idx = 1:numel(scenarios)
    for controller_idx = 1:numel(controllers)
        entry = entry + 1;
        controller = controllers{controller_idx};
        scenario_results(entry).scenario_name = scenarios(scenario_idx).name;
        scenario_results(entry).controller = controller;
        scenario_results(entry).result = pitch_simulate(cfg, designs, ...
            scenarios(scenario_idx), controller, ...
            cfg.random_seed + 1000 * scenario_idx + controller_idx);
    end
end

fprintf('Running Monte Carlo (%d cases)...\n', options.monte_carlo_runs);
monte_carlo = pitch_run_monte_carlo(cfg, designs, options.monte_carlo_runs);

bundle.cfg = cfg;
bundle.linear = linear;
bundle.designs = designs;
bundle.checks = checks;
bundle.scenarios = scenarios;
bundle.scenario_results = scenario_results;
bundle.monte_carlo = monte_carlo;
bundle.log_schema = pitch_log_schema();
bundle.meta.generated_at = datetime('now');
bundle.meta.provisional = true;
bundle.meta.deployment_allowed = false;

fprintf('Writing figures and report...\n');
bundle.figure_files = pitch_plot_results(bundle, output_dir, options.save_figures);
bundle.report_file = string(fullfile(output_dir, 'report.md'));
pitch_write_report(bundle, bundle.report_file);
save(fullfile(output_dir, 'pitch_offline_results.mat'), 'bundle', '-v7.3');

fprintf('Done. Stability %.2f%%, full metric pass %.2f%%.\n', ...
    100.0 * monte_carlo.stability_rate, 100.0 * monte_carlo.pass_rate);
fprintf('Report: %s\n', bundle.report_file);
fprintf('WARNING: Parameters remain provisional and must not be deployed.\n');
end
