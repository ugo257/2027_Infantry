function identified = pitch_identify_from_log(log_or_file, cfg)
%PITCH_IDENTIFY_FROM_LOG Estimate candidate parameters from labelled data.
% The output is always provisional until independent validation is supplied.

if nargin < 2
    cfg = pitch_default_config();
end
if ischar(log_or_file) || isstring(log_or_file)
    log = pitch_import_log(string(log_or_file));
    source = string(log_or_file);
elseif istable(log_or_file)
    log = log_or_file;
    source = "MATLAB table";
else
    error('pitch:id:InvalidInput', 'Input must be a CSV path or imported table.');
end
if height(log) < 10
    error('pitch:id:TooShort', 'At least 10 samples are required.');
end

phase = lower(string(log.experiment_phase));
identified.meta.source = source;
identified.meta.provisional = true;
identified.meta.validated = false;
identified.meta.message = ...
    "Candidate estimates only; independent validation is required before deployment.";

stationary = contains(phase, "stationary");
identified.noise.available = nnz(stationary) >= 20;
if identified.noise.available
    angle = double(log.imu_angle_rad(stationary));
    gyro = [double(log.gyro_x_rad_s(stationary)), ...
            double(log.gyro_y_rad_s(stationary)), ...
            double(log.gyro_z_rad_s(stationary))];
    torque = double(log.torque_feedback_nm(stationary));
    identified.noise.angle_variance_rad2 = var(angle, 1);
    identified.noise.gyro_covariance = cov(gyro, 1);
    identified.noise.torque_variance_nm2 = var(torque, 1);
end

sweep = contains(phase, "linkage_sweep");
identified.linkage.available = nnz(sweep) >= 50;
if identified.linkage.available
    theta = double(log.imu_angle_rad(sweep));
    phi = double(log.motor_angle_rad(sweep));
    coefficients = polyfit(theta, phi, 3);
    prediction = polyval(coefficients, theta);
    derivative_coefficients = polyder(coefficients);
    theta_grid = linspace(min(theta), max(theta), 101);
    derivative = polyval(derivative_coefficients, theta_grid);
    identified.linkage.phi_from_theta_poly = coefficients;
    identified.linkage.rmse_rad = sqrt(mean((prediction - phi).^2));
    identified.linkage.theta_range_rad = [min(theta), max(theta)];
    identified.linkage.monotonic = all(derivative > 0.0) || all(derivative < 0.0);
    identified.linkage.ratio_range = [min(derivative), max(derivative)];
end

static_hold = contains(phase, "static_hold");
identified.gravity.available = nnz(static_hold) >= 30;
if identified.gravity.available
    theta = double(log.imu_angle_rad(static_hold));
    torque = double(log.torque_feedback_nm(static_hold));
    coefficients = polyfit(theta, torque, 2);
    prediction = polyval(coefficients, theta);
    identified.gravity.motor_torque_poly_nm = coefficients;
    identified.gravity.rmse_nm = sqrt(mean((prediction - torque).^2));
    identified.gravity.theta_range_rad = [min(theta), max(theta)];
end

constant_velocity = contains(phase, "constant_velocity");
identified.friction.available = nnz(constant_velocity) >= 50;
if identified.friction.available
    omega = double(log.motor_velocity_rad_s(constant_velocity));
    torque = double(log.torque_feedback_nm(constant_velocity));
    if identified.gravity.available
        theta = double(log.imu_angle_rad(constant_velocity));
        torque = torque - polyval(identified.gravity.motor_torque_poly_nm, theta);
    end
    smooth = cfg.model.coulomb_smooth_rad_s;
    regressors = [omega, tanh(omega / max(smooth, eps))];
    coefficients = regressors \ torque;
    residual = torque - regressors * coefficients;
    identified.friction.viscous_nms_rad = coefficients(1);
    identified.friction.coulomb_nm = coefficients(2);
    identified.friction.rmse_nm = sqrt(mean(residual.^2));
end

dynamic = contains(phase, "dynamic");
identified.dynamic.available = nnz(dynamic) >= 200;
identified.dynamic.message = ...
    "Dynamic grey-box fitting requires calibrated gyro selection and is intentionally gated.";
end

