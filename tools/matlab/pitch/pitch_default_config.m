function cfg = pitch_default_config()
%PITCH_DEFAULT_CONFIG Provisional offline model and design configuration.
% All physical values in this file require identification before deployment.

cfg.meta.name = 'pitch_offline_provisional';
cfg.meta.provisional = true;
cfg.meta.source = 'Firmware constants plus explicit uncertainty bounds';
cfg.random_seed = 20270802;

cfg.sim.Ts = 1.0e-3;
cfg.sim.duration_s = 1.0;
cfg.sim.jitter_fraction = 0.03;
cfg.sim.theta_safe_rad = deg2rad(18.0);
cfg.sim.omega_safe_rad_s = 8.0;

cfg.model.J_kg_m2 = 0.020;
cfg.model.inertia_sin_fraction = 0.10;
cfg.model.inertia_phase_rad = 0.0;
% True-plant disturbance descriptors. They remain in d and are not used as
% explicit friction feedforward by the deployment controller.
cfg.model.B_nms_rad = 0.120;
cfg.model.coulomb_nm = 0.080;
cfg.model.coulomb_smooth_rad_s = 0.080;
cfg.model.linkage_ratio = 1.0;
cfg.model.linkage_sin_fraction = 0.15;
cfg.model.linkage_phase_rad = 0.10;
cfg.model.motor_center_rad = 1.970;
cfg.model.gravity_poly_nm = [0.752579, -0.540475, 0.275000];
cfg.model.gravity_min_nm = 0.080;
cfg.model.gravity_max_nm = 0.550;
cfg.model.torque_gain = 1.0;
cfg.model.actuator_time_constant_s = 0.001;
cfg.model.actuator_delay_s = 0.000;
cfg.model.disturbance_nm = 0.0;

cfg.sensor.angle_bias_rad = 0.0;
cfg.sensor.angle_noise_std_rad = deg2rad(0.025);
cfg.sensor.gyro_bias_rad_s = 0.0;
cfg.sensor.gyro_noise_std_rad_s = 0.025;
cfg.sensor.torque_noise_std_nm = 0.010;
cfg.sensor.quantization_angle_rad = 0.0;

cfg.limits.torque_soft_nm = 1.60;
cfg.limits.torque_hard_nm = 3.20;
cfg.limits.torque_slew_nm_s = 60.0;
cfg.limits.feedback_torque_budget_nm = 1.20;
cfg.limits.feedback_torque_limit_nm = 0.80;
cfg.limits.inertial_feedforward_limit_nm = 0.80;
cfg.limits.integral_rad_s = 0.10;
cfg.limits.antiwindup_gain = 8.0;
cfg.limits.eso_comp_nm = 0.60;

cfg.reference.bandwidth_rad_s = 40.0;
cfg.reference.velocity_limit_rad_s = 4.0;
cfg.reference.acceleration_limit_rad_s2 = 120.0;

cfg.design.theta_error_max_rad = deg2rad(5.0);
cfg.design.omega_error_max_rad_s = 3.0;
cfg.design.integral_error_max_rad_s = 0.10;
cfg.design.target_settle_s = 0.150;
cfg.design.target_overshoot_rad = deg2rad(0.05);
cfg.design.target_steady_error_rad = deg2rad(0.05);
cfg.design.lqi_theta_scales = [0.25, 1.0, 4.0, 16.0];
cfg.design.lqi_omega_scales = [0.25, 1.0, 4.0, 16.0, 64.0];
cfg.design.lqi_integral_scales = [0.01, 0.10, 1.0, 10.0];
cfg.design.lqi_effort_scales = [0.25, 1.0, 4.0];

cfg.eso.process_noise_diag = [1.0e-10, 2.0e-5, 2.0e-2];
cfg.eso.disturbance_decay_rad_s = 0.5;
cfg.eso.use_dual_measurement = true;

cfg.legacy.k_theta = 18.0;
cfg.legacy.k_omega = 0.25;
cfg.legacy.k_i = 1.20;
cfg.legacy.integral_limit_rad_s = 0.040;
cfg.legacy.eso_bandwidth_rad_s = 28.0;
cfg.legacy.eso_comp_gain = 0.20;
cfg.legacy.eso_comp_limit_nm = 0.25;
cfg.legacy.speed_pid_kp = 1.20;

cfg.uncertainty.J_scale = [0.5, 2.0];
cfg.uncertainty.B_scale = [0.3, 3.0];
cfg.uncertainty.friction_scale = [0.3, 3.0];
cfg.uncertainty.linkage_scale = [0.6, 1.4];
cfg.uncertainty.torque_gain = [0.7, 1.3];
cfg.uncertainty.delay_s = [0.0, 0.005];
cfg.uncertainty.angle_noise_std_rad = deg2rad([0.01, 0.10]);
cfg.uncertainty.gyro_noise_std_rad_s = [0.01, 0.10];

cfg.monte_carlo.runs = 1000;
cfg.monte_carlo.duration_s = 0.8;
cfg.monte_carlo.step_rad = deg2rad(5.0);
cfg.monte_carlo.store_worst_count = 5;
end
