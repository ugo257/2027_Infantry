function design = pitch_design_eso(cfg, linear)
%PITCH_DESIGN_ESO Design angle-only and angle+gyro discrete ESO gains.

decay = cfg.eso.disturbance_decay_rad_s;
A = [0.0, 1.0, 0.0; ...
     0.0, -cfg.model.B_nms_rad / linear.terms.inertia, 1.0; ...
     0.0, 0.0, -decay];
B = [0.0; linear.terms.ratio / linear.terms.inertia; 0.0];
aug_d = c2d(ss(A, B, eye(3), zeros(3, 1)), cfg.sim.Ts, 'zoh');
Ad = aug_d.A;
Bd = aug_d.B;

Qn = diag(cfg.eso.process_noise_diag);
C_angle = [1.0, 0.0, 0.0];
R_angle = max(cfg.sensor.angle_noise_std_rad^2, 1.0e-12);
[L_angle, P_angle] = dlqe(Ad, eye(3), C_angle, Qn, R_angle);

C_dual = [1.0, 0.0, 0.0; 0.0, 1.0, 0.0];
R_dual = diag([max(cfg.sensor.angle_noise_std_rad^2, 1.0e-12), ...
               max(cfg.sensor.gyro_noise_std_rad_s^2, 1.0e-10)]);
[L_dual, P_dual] = dlqe(Ad, eye(3), C_dual, Qn, R_dual);

design.Ad = Ad;
design.Bd = Bd;
design.C_angle = C_angle;
design.C_dual = C_dual;
design.L_angle = L_angle;
design.L_dual = L_dual;
design.P_angle = P_angle;
design.P_dual = P_dual;
design.use_dual_measurement = cfg.eso.use_dual_measurement;
design.provisional = true;
end

