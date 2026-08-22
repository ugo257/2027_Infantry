function log = pitch_import_log(csv_file)
%PITCH_IMPORT_LOG Import and validate a future target-timestamped CSV log.

arguments
    csv_file (1, 1) string
end
if ~isfile(csv_file)
    error('pitch:log:NotFound', 'CSV log does not exist: %s', csv_file);
end

options = detectImportOptions(csv_file, 'VariableNamingRule', 'preserve');
raw = readtable(csv_file, options);
schema = pitch_log_schema();
required_names = schema.name(schema.required);
missing = setdiff(required_names, string(raw.Properties.VariableNames));
if ~isempty(missing)
    error('pitch:log:MissingColumns', 'Missing required columns: %s', ...
        strjoin(missing, ', '));
end

log = raw;
log.experiment_phase = string(log.experiment_phase);
numeric_names = setdiff(required_names, "experiment_phase");
for idx = 1:numel(numeric_names)
    name = numeric_names(idx);
    values = log.(name);
    if ~isnumeric(values) || any(~isfinite(values))
        error('pitch:log:InvalidColumn', ...
            'Column %s must contain finite numeric values.', name);
    end
end

if any(diff(double(log.sample_index)) <= 0)
    error('pitch:log:Sequence', 'sample_index must be strictly increasing.');
end
if any(diff(double(log.time_us)) <= 0)
    error('pitch:log:Timestamp', 'time_us must be strictly increasing.');
end

log.time_s = (double(log.time_us) - double(log.time_us(1))) * 1.0e-6;
log.dt_s = [NaN; diff(log.time_s)];
end

