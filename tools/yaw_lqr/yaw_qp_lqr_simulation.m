function results = yaw_qp_lqr_simulation(varargin)
%YAW_QP_LQR_SIMULATION Plot the firmware Yaw reference and simulate its LQR.
%
% Examples:
%   yaw_qp_lqr_simulation
%   yaw_qp_lqr_simulation('FrequencyHz', 5, 'DurationS', 3)
%
% The reference is parsed from application/test/yaw_qp_reference_tables.h,
% so the plotted angle/rate/acceleration is the data interpolated by the
% firmware. This file is an offline tool and is not part of the EIDE build.

parser = inputParser;
addParameter(parser, 'FrequencyHz', 3, @(x) isnumeric(x) && isscalar(x));
addParameter(parser, 'RawPeakToPeakDeg', 20, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'AccelLimitRadS2', 50, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'DurationS', 3, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'ControlDtS', 0.001, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, 'ViscousDampingPerS', 1.55, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(parser, 'OutputDir', '', @(x) ischar(x) || isstring(x));
parse(parser, varargin{:});
options = parser.Results;

toolDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(toolDir));
if strlength(string(options.OutputDir)) == 0
    outputDir = fullfile(toolDir, 'results');
else
    outputDir = char(options.OutputDir);
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

tableHeader = fullfile(repoRoot, 'application', 'test', 'yaw_qp_reference_tables.h');
robotDef = fullfile(repoRoot, 'application', 'robot_def.h');
reference = loadFirmwareReference(tableHeader, options);
controller = loadControllerConfig(robotDef, options.ViscousDampingPerS);

t = (0:options.ControlDtS:options.DurationS).';
reference = sampleReference(reference, t);
simulationWithFF = simulateController(t, reference, controller, true);
simulationWithoutFF = simulateController(t, reference, controller, false);

metricsWithFF = trackingMetrics(t, reference, simulationWithFF);
metricsWithoutFF = trackingMetrics(t, reference, simulationWithoutFF);

tag = sprintf('%dhz_%gdeg_%gacc', reference.frequencyHz, ...
    options.RawPeakToPeakDeg, options.AccelLimitRadS2);
referencePng = fullfile(outputDir, ['yaw_qp_reference_' tag '.png']);
simulationPng = fullfile(outputDir, ['yaw_lqr_simulation_' tag '.png']);
traceCsv = fullfile(outputDir, ['yaw_lqr_simulation_' tag '.csv']);
metricsCsv = fullfile(outputDir, ['yaw_lqr_metrics_' tag '.csv']);

plotReference(reference, t, referencePng);
plotSimulation(reference, t, simulationWithFF, simulationWithoutFF, simulationPng);

trace = table(t, rad2deg(reference.rawAngleRad), rad2deg(reference.angleRad), ...
    reference.rateRadS, reference.accelRadS2, ...
    rad2deg(simulationWithFF.angleRad), simulationWithFF.rateRadS, ...
    simulationWithFF.currentCount, rad2deg(simulationWithFF.errorRad), ...
    rad2deg(simulationWithoutFF.angleRad), simulationWithoutFF.rateRadS, ...
    simulationWithoutFF.currentCount, rad2deg(simulationWithoutFF.errorRad), ...
    'VariableNames', {'Time_s', 'RawTarget_deg', 'Reference_deg', ...
    'ReferenceRate_rad_s', 'ReferenceAccel_rad_s2', ...
    'ActualFFOn_deg', 'ActualRateFFOn_rad_s', 'CurrentFFOn_count', ...
    'ErrorFFOn_deg', 'ActualFFOff_deg', 'ActualRateFFOff_rad_s', ...
    'CurrentFFOff_count', 'ErrorFFOff_deg'});
writetable(trace, traceCsv);

metrics = table(["feedforward_on"; "feedforward_off"], ...
    [metricsWithFF.rmseDeg; metricsWithoutFF.rmseDeg], ...
    [metricsWithFF.p95AbsDeg; metricsWithoutFF.p95AbsDeg], ...
    [metricsWithFF.maxAbsDeg; metricsWithoutFF.maxAbsDeg], ...
    [metricsWithFF.lagMs; metricsWithoutFF.lagMs], ...
    [metricsWithFF.phaseDeg; metricsWithoutFF.phaseDeg], ...
    [metricsWithFF.currentPeak; metricsWithoutFF.currentPeak], ...
    [metricsWithFF.saturationPercent; metricsWithoutFF.saturationPercent], ...
    [metricsWithFF.slewPercent; metricsWithoutFF.slewPercent], ...
    'VariableNames', {'Case', 'RMSE_deg', 'P95AbsError_deg', ...
    'MaxAbsError_deg', 'Lag_ms', 'Phase_deg', 'PeakCurrent_count', ...
    'Saturation_percent', 'SlewLimited_percent'});
writetable(metrics, metricsCsv);

fprintf('\nYaw QP reference: %g Hz, raw span %g deg, requested amax %g rad/s^2\n', ...
    reference.frequencyHz, options.RawPeakToPeakDeg, options.AccelLimitRadS2);
fprintf('Firmware effective amax: %.3f rad/s^2\n', reference.effectiveAccelLimit);
fprintf('Planned peak-to-peak: %.3f deg\n', reference.plannedPeakToPeakDeg);
fprintf('Plant: alpha = %.6g * current - %.6g * omega\n', ...
    controller.currentToAccel, controller.viscousDamping);
disp(metrics);
fprintf('Reference plot: %s\n', referencePng);
fprintf('Simulation plot: %s\n', simulationPng);
fprintf('Trace CSV: %s\n', traceCsv);
fprintf('Metrics CSV: %s\n\n', metricsCsv);

results = struct('reference', reference, 'controller', controller, ...
    'withFeedforward', simulationWithFF, 'withoutFeedforward', simulationWithoutFF, ...
    'metrics', metrics, 'referencePlot', referencePng, ...
    'simulationPlot', simulationPng, 'traceCsv', traceCsv, ...
    'metricsCsv', metricsCsv);
end

function reference = loadFirmwareReference(headerPath, options)
text = fileread(headerPath);
pointCount = parseDefine(text, 'YAW_TEST_QP_REFERENCE_POINTS');
levelCount = parseDefine(text, 'YAW_TEST_QP_ACCEL_LEVEL_COUNT');
baseSpanDeg = parseDefine(text, 'YAW_TEST_QP_BASE_PEAK_TO_PEAK_DEG');
accelLevels = parseCArray(text, 'yaw_test_qp_accel_levels_rad_s2', 1, levelCount);

if abs(options.FrequencyHz - 3) <= abs(options.FrequencyHz - 5)
    frequencyHz = 3;
else
    frequencyHz = 5;
end
prefix = sprintf('yaw_test_qp_%dhz_', frequencyHz);
angleTables = parseCArray(text, [prefix 'angle_rad'], levelCount, pointCount);
rateTables = parseCArray(text, [prefix 'rate_rad_s'], levelCount, pointCount);
accelTables = parseCArray(text, [prefix 'accel_rad_s2'], levelCount, pointCount);

amplitudeScale = options.RawPeakToPeakDeg / baseSpanDeg;
normalizedAccel = options.AccelLimitRadS2 / amplitudeScale;
if normalizedAccel < accelLevels(1)
    amplitudeScale = amplitudeScale * normalizedAccel / accelLevels(1);
    normalizedAccel = accelLevels(1);
end
if normalizedAccel >= accelLevels(end)
    lowerIndex = levelCount;
    upperIndex = levelCount;
    blend = 0;
else
    upperIndex = find(accelLevels >= normalizedAccel, 1, 'first');
    lowerIndex = max(1, upperIndex - 1);
    if lowerIndex == upperIndex
        blend = 0;
    else
        blend = (normalizedAccel - accelLevels(lowerIndex)) / ...
            (accelLevels(upperIndex) - accelLevels(lowerIndex));
    end
end

angle = amplitudeScale * ((1 - blend) * angleTables(lowerIndex, :) + ...
    blend * angleTables(upperIndex, :));
rate = amplitudeScale * ((1 - blend) * rateTables(lowerIndex, :) + ...
    blend * rateTables(upperIndex, :));
accel = amplitudeScale * ((1 - blend) * accelTables(lowerIndex, :) + ...
    blend * accelTables(upperIndex, :));

% The generator rotates the planned table to its rising zero crossing. Recover
% the matching raw-sawtooth phase for a truthful overlay in the offline plot.
phaseGrid = (0:pointCount - 1) / pointCount;
candidateOffsets = (0:pointCount * 64 - 1) / (pointCount * 64);
rawSpanRad = deg2rad(options.RawPeakToPeakDeg);
bestRawOffset = 0;
bestRawError = Inf;
for candidate = candidateOffsets
    rawCandidate = rawSpanRad * (mod(phaseGrid + candidate, 1) - 0.5);
    candidateError = mean((angle - rawCandidate).^2);
    if candidateError < bestRawError
        bestRawError = candidateError;
        bestRawOffset = candidate;
    end
end

reference = struct('frequencyHz', frequencyHz, 'pointCount', pointCount, ...
    'angleTableRad', angle, 'rateTableRadS', rate, ...
    'accelTableRadS2', accel, ...
    'effectiveAccelLimit', amplitudeScale * ...
        ((1 - blend) * accelLevels(lowerIndex) + blend * accelLevels(upperIndex)), ...
    'plannedPeakToPeakDeg', rad2deg(max(angle) - min(angle)), ...
    'rawPeakToPeakDeg', options.RawPeakToPeakDeg, ...
    'rawPhaseOffset', bestRawOffset);
end

function reference = sampleReference(reference, t)
phase = mod(t * reference.frequencyHz, 1);
grid = (0:reference.pointCount) / reference.pointCount;
reference.angleRad = interp1(grid, ...
    [reference.angleTableRad reference.angleTableRad(1)], phase, 'linear');
reference.rateRadS = interp1(grid, ...
    [reference.rateTableRadS reference.rateTableRadS(1)], phase, 'linear');
reference.accelRadS2 = interp1(grid, ...
    [reference.accelTableRadS2 reference.accelTableRadS2(1)], phase, 'linear');
rawSpanRad = deg2rad(reference.rawPeakToPeakDeg);
rawPhase = mod(phase + reference.rawPhaseOffset, 1);
reference.rawAngleRad = rawSpanRad * (rawPhase - 0.5);
reference.rawRateRadS = rawSpanRad * reference.frequencyHz * ones(size(t));
end

function controller = loadControllerConfig(robotDefPath, viscousDamping)
text = fileread(robotDefPath);
controller = struct( ...
    'inertia', parseDefine(text, 'GIMBAL_YAW_LQR_INERTIA'), ...
    'kAngle', parseDefine(text, 'GIMBAL_YAW_LQR_K_ANGLE'), ...
    'kRate', parseDefine(text, 'GIMBAL_YAW_LQR_K_RATE'), ...
    'torqueToCurrent', parseDefine(text, 'GIMBAL_YAW_LQR_TORQUE_TO_CURRENT'), ...
    'currentToAccel', parseDefine(text, 'GIMBAL_YAW_LQR_CURRENT_TO_ACCEL_GAIN'), ...
    'currentLimit', parseDefine(text, 'GIMBAL_YAW_LQR_CURRENT_LIMIT'), ...
    'currentSlew', parseDefine(text, 'GIMBAL_YAW_LQR_CURRENT_SLEW_RATE'), ...
    'viscousDamping', viscousDamping);
end

function simulation = simulateController(t, reference, controller, feedforwardEnabled)
count = numel(t);
dt = median(diff(t));
angle = zeros(count, 1);
rate = zeros(count, 1);
accel = zeros(count, 1);
current = zeros(count, 1);
currentPre = zeros(count, 1);
saturation = false(count, 1);
slewLimited = false(count, 1);
angle(1) = reference.angleRad(1);
rate(1) = reference.rateRadS(1);
appliedCurrent = 0;

for index = 1:count
    angleError = wrapPi(angle(index) - reference.angleRad(index));
    rateError = rate(index) - reference.rateRadS(index);
    torque = -controller.kAngle * angleError - controller.kRate * rateError;
    accelFeedforwardCurrent = 0;
    if feedforwardEnabled
        accelFeedforwardCurrent = reference.accelRadS2(index) / ...
            controller.currentToAccel;
    end
    currentPre(index) = torque * controller.torqueToCurrent + ...
        accelFeedforwardCurrent;
    limited = min(max(currentPre(index), -controller.currentLimit), controller.currentLimit);
    saturation(index) = limited ~= currentPre(index);
    maxDelta = controller.currentSlew * dt;
    command = min(max(limited, appliedCurrent - maxDelta), appliedCurrent + maxDelta);
    slewLimited(index) = abs(command - limited) > 1.0e-6;
    current(index) = command;
    appliedCurrent = command;
    accel(index) = controller.currentToAccel * appliedCurrent - ...
        controller.viscousDamping * rate(index);

    if index < count
        angle(index + 1) = angle(index) + rate(index) * dt + 0.5 * accel(index) * dt^2;
        rate(index + 1) = rate(index) + accel(index) * dt;
    end
end

simulation = struct('angleRad', angle, 'rateRadS', rate, ...
    'accelRadS2', accel, 'currentCount', current, ...
    'currentPreLimitCount', currentPre, 'saturation', saturation, ...
    'slewLimited', slewLimited, ...
    'errorRad', wrapPi(reference.angleRad - angle), ...
    'feedforwardEnabled', feedforwardEnabled);
end

function metrics = trackingMetrics(t, reference, simulation)
settleTime = max(1.0, 3.0 / reference.frequencyHz);
selected = t >= settleTime;
errorDeg = rad2deg(simulation.errorRad(selected));
absError = abs(errorDeg);
lagS = estimateLag(t(selected), reference.angleRad(selected), ...
    simulation.angleRad(selected), reference.frequencyHz);
metrics = struct( ...
    'rmseDeg', sqrt(mean(errorDeg.^2)), ...
    'p95AbsDeg', percentile(absError, 95), ...
    'maxAbsDeg', max(absError), ...
    'lagMs', 1000 * lagS, ...
    'phaseDeg', 360 * reference.frequencyHz * lagS, ...
    'currentPeak', max(abs(simulation.currentCount(selected))), ...
    'saturationPercent', 100 * mean(simulation.saturation(selected)), ...
    'slewPercent', 100 * mean(simulation.slewLimited(selected)));
end

function lagS = estimateLag(t, reference, measured, frequencyHz)
dt = median(diff(t));
maximumLag = min(0.1, 0.45 / frequencyHz);
candidateLags = (-maximumLag:dt:maximumLag).';
bestCorrelation = -Inf;
lagS = 0;
for lag = candidateLags.'
    shifted = interp1(t, measured, t + lag, 'linear', NaN);
    valid = isfinite(shifted);
    if nnz(valid) < 20
        continue;
    end
    matrix = corrcoef(reference(valid), shifted(valid));
    if numel(matrix) >= 4 && matrix(1, 2) > bestCorrelation
        bestCorrelation = matrix(1, 2);
        lagS = lag;
    end
end
end

function plotReference(reference, t, outputPath)
cycles = min(t(end), 2 / reference.frequencyHz);
selected = t <= cycles;
figureHandle = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 1100 820]);
layout = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t(selected), rad2deg(reference.rawAngleRad(selected)), '--', ...
    'Color', [0.85 0.33 0.10], 'LineWidth', 1.1);
hold on;
plot(t(selected), rad2deg(reference.angleRad(selected)), ...
    'Color', [0 0.45 0.74], 'LineWidth', 1.6);
grid on; ylabel('Angle (deg)'); legend('Raw sawtooth', 'QP planned', 'Location', 'best');

nexttile;
plot(t(selected), reference.rateRadS(selected), ...
    'Color', [0.47 0.67 0.19], 'LineWidth', 1.5);
hold on;
plot(t(selected), reference.rawRateRadS(selected), ':', ...
    'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
grid on; ylabel('Rate (rad/s)'); legend('QP planned', 'Raw ramp rate', 'Location', 'best');

nexttile;
plot(t(selected), reference.accelRadS2(selected), ...
    'Color', [0.49 0.18 0.56], 'LineWidth', 1.4);
hold on;
yline(reference.effectiveAccelLimit, '--k');
yline(-reference.effectiveAccelLimit, '--k');
grid on; ylabel('Accel (rad/s^2)'); xlabel('Time (s)');
title(layout, sprintf('Firmware Yaw QP reference: %g Hz, raw %g deg, planned %.2f deg p-p', ...
    reference.frequencyHz, reference.rawPeakToPeakDeg, reference.plannedPeakToPeakDeg));
exportgraphics(figureHandle, outputPath, 'Resolution', 170);
close(figureHandle);
end

function plotSimulation(reference, t, withFF, withoutFF, outputPath)
figureHandle = figure('Visible', 'off', 'Color', 'white', 'Position', [100 100 1150 900]);
layout = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, rad2deg(reference.angleRad), 'k--', 'LineWidth', 1.1); hold on;
plot(t, rad2deg(withFF.angleRad), 'Color', [0 0.45 0.74], 'LineWidth', 1.3);
plot(t, rad2deg(withoutFF.angleRad), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
grid on; ylabel('Angle (deg)');
legend('Reference', 'Existing accel FF on', 'Accel FF off', 'Location', 'best');

nexttile;
plot(t, reference.rateRadS, 'k--', 'LineWidth', 1.0); hold on;
plot(t, withFF.rateRadS, 'Color', [0 0.45 0.74], 'LineWidth', 1.2);
plot(t, withoutFF.rateRadS, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
grid on; ylabel('Rate (rad/s)');

nexttile;
plot(t, rad2deg(withFF.errorRad), 'Color', [0 0.45 0.74], 'LineWidth', 1.1); hold on;
plot(t, rad2deg(withoutFF.errorRad), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.0);
grid on; ylabel('Ref - actual (deg)');

nexttile;
plot(t, withFF.currentCount, 'Color', [0.49 0.18 0.56], 'LineWidth', 1.1); hold on;
plot(t, withoutFF.currentCount, 'Color', [0.47 0.67 0.19], 'LineWidth', 1.0);
grid on; ylabel('Current count'); xlabel('Time (s)');
title(layout, 'Yaw LQR replay using current firmware gains and limits');
exportgraphics(figureHandle, outputPath, 'Resolution', 170);
close(figureHandle);
end

function value = parseDefine(text, name)
expression = ['(?m)^\s*#define\s+' regexptranslate('escape', name) '\s+([^\r\n/]+)'];
token = regexp(text, expression, 'tokens', 'once');
if isempty(token)
    error('Missing define %s', name);
end
number = regexp(token{1}, '[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', ...
    'match', 'once');
value = str2double(number);
end

function values = parseCArray(text, symbol, rows, columns)
expression = ['(?s)static\s+const\s+float\s+' ...
    regexptranslate('escape', symbol) '[^=]*=\s*\{(.*?)\};'];
token = regexp(text, expression, 'tokens', 'once');
if isempty(token)
    error('Missing C array %s', symbol);
end
numbers = regexp(token{1}, ...
    '[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?(?=f?(?:\s*[,}]|\s*$))', ...
    'match');
numericValues = str2double(numbers);
if numel(numericValues) ~= rows * columns
    error('Array %s has %d values; expected %d', ...
        symbol, numel(numericValues), rows * columns);
end
values = reshape(numericValues, columns, rows).';
end

function wrapped = wrapPi(angle)
wrapped = mod(angle + pi, 2 * pi) - pi;
end

function value = percentile(values, percentage)
values = sort(values(:));
position = 1 + (numel(values) - 1) * percentage / 100;
lower = floor(position);
upper = ceil(position);
if lower == upper
    value = values(lower);
else
    value = values(lower) + (position - lower) * (values(upper) - values(lower));
end
end
